package posemap

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"sync"
)

// Map 是一份带持久化文件的二维轨迹与路标地图。零值不可用，须经 Create
// 或 Open 得到。单个 Map 可被多个 goroutine 并发使用。
type Map struct {
	mu     sync.Mutex
	path   string
	state  *mapState
	closed bool
}

// mapState 是地图的全部内存状态。
type mapState struct {
	config      Config
	trajectory  []Pose // 按时间严格递增；trajectory[0] 为初始位姿
	landmarks   map[string]*landmarkState
	segments    map[string]*segmentRecord
	sources     []*frameSource // sources[k] 对应 trajectory[k+1]；长度恒为 len(trajectory)-1
	corrections []*CorrectionRecord
	corrIndex   map[string]string // 校正标识 -> 首次内容指纹
}

// frameSource 是生成某一帧的原始输入，供回环校正重放路标观测与累计
// 原运动方差。该切片中的 nil 元素表示对应帧来自不携带逐帧依据的旧
// 文件，涉及它的历史范围不能校正。
type frameSource struct {
	moveVariance float64
	observations []Observation
}

// landmarkState 是路标在地图坐标下的聚合状态。
//
// legacy* 只用于打开不携带逐帧观测来源的旧文件：旧文件中每一帧的观测
// 无法重放，因此旧文件里该路标的全部观测贡献视为固定的 legacyCount 次
// 平均，不随后续校正改变；旧文件之后新导入且依据完整的观测再做增量
// 平均。由本版本创建/导入的路标 legacyCount 为 0，全部观测均可重放。
type landmarkState struct {
	x           float64
	y           float64
	count       int
	legacyCount int
	legacyMX    float64
	legacyMY    float64
}

// segmentRecord 记录一个已成功保存的段：内容指纹与首次导入结果，
// 供相同内容重复导入时直接返回。
type segmentRecord struct {
	hash   string
	result ImportResult
}

// ImportResult 是一次成功段导入的结果：本段末位姿及方差，以及本段
// 涉及（观测到）的全部路标标识（去重、按标识排序）。
type ImportResult struct {
	EndPose     Pose
	LandmarkIDs []string
}

// Create 在 path 处创建一份新地图并写入初始文件。path 已存在时返回
// 错误，以免覆盖既有数据（既有地图请用 Open）。
func Create(path string, cfg Config) (*Map, error) {
	if err := validateConfig(cfg); err != nil {
		return nil, err
	}
	st := &mapState{
		config:     cfg,
		trajectory: []Pose{{Time: cfg.InitialTime, X: cfg.InitialX, Y: cfg.InitialY, Heading: cfg.InitialHeading, Variance: cfg.InitialVariance}},
		landmarks:  make(map[string]*landmarkState),
		segments:   make(map[string]*segmentRecord),
		corrIndex:  make(map[string]string),
	}
	m := &Map{path: path, state: st}
	// O_EXCL：目标已存在即失败，绝不当作新地图覆盖。
	if err := m.saveExclusive(); err != nil {
		return nil, err
	}
	return m, nil
}

// Open 打开 path 处由 Create/导入生成的地图文件。文件不存在时返回
// 包装了 fs.ErrNotExist 的错误；文件损坏返回可 errors.Is(ErrCorrupt)
// 的错误；格式版本不支持返回 ErrUnsupportedVersion。Open 绝不覆盖
// 无法识别的文件。
func Open(path string) (*Map, error) {
	st, err := loadFile(path)
	if err != nil {
		return nil, err
	}
	return &Map{path: path, state: st}, nil
}

// Path 返回地图对应的本地数据文件路径。
func (m *Map) Path() string {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.path
}

// Close 关闭地图。每次成功导入都已落盘，Close 本身不修改数据文件；
// 关闭后再调用其他方法返回 ErrClosed。
func (m *Map) Close() error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.closed = true
	return nil
}

func validateConfig(cfg Config) error {
	switch {
	case !isFinite(cfg.InitialX) || !isFinite(cfg.InitialY) || !isFinite(cfg.InitialHeading):
		return fmt.Errorf("%w: non-finite initial pose", ErrInvalidConfig)
	case !isFinite(cfg.InitialVariance) || cfg.InitialVariance < 0:
		return fmt.Errorf("%w: initial variance must be non-negative and finite", ErrInvalidConfig)
	case cfg.MaxInterval <= 0:
		return fmt.Errorf("%w: max interval must be positive", ErrInvalidConfig)
	case !isFinite(cfg.MergeDistance) || cfg.MergeDistance <= 0:
		return fmt.Errorf("%w: merge distance must be positive and finite", ErrInvalidConfig)
	default:
		return nil
	}
}

// ImportSegment 导入一批带非空段标识的有序帧。
//
// 整段要么全部生效（含落盘），要么因可区分的 *RejectError 被整段拒绝，
// 此前已提交的位姿、路标、观测次数不受影响。已成功保存的段标识再次以
// 相同内容导入时，直接返回首次导入的结果且不改变当前数据；同一标识
// 对应不同内容则以 RejectDuplicateMismatch 拒绝。
func (m *Map) ImportSegment(seg Segment) (ImportResult, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return ImportResult{}, ErrClosed
	}
	st := m.state

	if seg.ID == "" {
		return ImportResult{}, &RejectError{Kind: RejectEmptyID}
	}
	if len(seg.Frames) == 0 {
		return ImportResult{}, &RejectError{Kind: RejectEmptySegment}
	}

	// 已保存段：相同内容直接返回首次结果；不同内容明确拒绝。
	// 不做任何校验或状态变更，即使其后又导入过其他段。
	if rec, ok := st.segments[seg.ID]; ok {
		if canonicalHash(seg.Frames) == rec.hash {
			return cloneResult(rec.result), nil
		}
		return ImportResult{}, &RejectError{Kind: RejectDuplicateMismatch, Landmark: seg.ID, HasLandmark: true}
	}

	// ---- 在暂存区上校验并推演，真实状态此时保持不变 ----
	cur := st.trajectory[len(st.trajectory)-1]
	prevTime := cur.Time
	newPoses := make([]Pose, 0, len(seg.Frames))
	staged := make(map[string]*landmarkState) // 本段触及路标的暂存聚合
	touched := make(map[string]struct{})

	reject := func(kind string, frame int) (ImportResult, error) {
		return ImportResult{}, &RejectError{Kind: kind, Frame: frame, HasFrame: true}
	}

	for i, f := range seg.Frames {
		// 时间：严格递增；首帧必须晚于初始时间或上一段末帧。
		if f.Time <= prevTime {
			return reject(RejectTimeOrder, i)
		}
		// 相邻位姿间隔上限。
		if f.Time-prevTime > m.state.config.MaxInterval {
			return reject(RejectInterval, i)
		}
		// 运动数值有限、方差非负。
		if !isFinite(f.DX) || !isFinite(f.DY) || !isFinite(f.DHeading) || !isFinite(f.MoveVariance) {
			return reject(RejectNonFinite, i)
		}
		if f.MoveVariance < 0 {
			return reject(RejectNegativeVariance, i)
		}
		// 观测标识非空、观测位置有限。
		for _, ob := range f.Observations {
			if ob.ID == "" {
				return reject(RejectEmptyID, i)
			}
			if !isFinite(ob.X) || !isFinite(ob.Y) {
				return ImportResult{}, &RejectError{Kind: RejectNonFinite, Frame: i, HasFrame: true, Landmark: ob.ID, HasLandmark: true}
			}
		}

		// 运动：先按运动前朝向把平移转换到地图坐标，再更新朝向并归一。
		nx, ny := localToMap(cur.X, cur.Y, cur.Heading, f.DX, f.DY)
		cur = Pose{
			Time:     f.Time,
			X:        nx,
			Y:        ny,
			Heading:  normalizeAngle(cur.Heading + f.DHeading),
			Variance: cur.Variance + f.MoveVariance,
		}
		newPoses = append(newPoses, cur)

		// 观测按运动完成后的位姿转换到地图坐标，同帧按输入次序处理。
		for _, ob := range f.Observations {
			mx, my := localToMap(cur.X, cur.Y, cur.Heading, ob.X, ob.Y)
			lm := staged[ob.ID]
			if lm == nil {
				lm = st.landmarks[ob.ID]
				if lm != nil {
					// 复制已提交路标，避免提前修改真实状态。
					cp := *lm
					lm = &cp
				}
			}
			if lm != nil {
				if math.Hypot(mx-lm.x, my-lm.y) > m.state.config.MergeDistance {
					return ImportResult{}, &RejectError{Kind: RejectLandmarkConflict, Frame: i, HasFrame: true, Landmark: ob.ID, HasLandmark: true}
				}
				lm.x = (lm.x*float64(lm.count) + mx) / float64(lm.count+1)
				lm.y = (lm.y*float64(lm.count) + my) / float64(lm.count+1)
				lm.count++
			} else {
				lm = &landmarkState{x: mx, y: my, count: 1}
			}
			staged[ob.ID] = lm
			touched[ob.ID] = struct{}{}
		}

		prevTime = f.Time
	}

	ids := make([]string, 0, len(touched))
	for id := range touched {
		ids = append(ids, id)
	}
	sort.Strings(ids)
	res := ImportResult{EndPose: cur, LandmarkIDs: ids}

	// 逐帧依据（原运动方差与原始观测）与位姿一一对应，供以后的回环
	// 校正重放。
	newSources := make([]*frameSource, len(seg.Frames))
	for i, f := range seg.Frames {
		obs := append([]Observation(nil), f.Observations...)
		newSources[i] = &frameSource{moveVariance: f.MoveVariance, observations: obs}
	}

	// ---- 全部校验通过：提交内存状态并落盘；落盘失败精确回滚 ----
	trajLenBefore := len(st.trajectory)
	srcLenBefore := len(st.sources)
	type undoLandmark struct {
		lm *landmarkState
		ok bool
	}
	undo := make(map[string]undoLandmark, len(staged))
	for id, nl := range staged {
		old, ok := st.landmarks[id]
		undo[id] = undoLandmark{old, ok}
		st.landmarks[id] = nl
	}
	st.trajectory = append(st.trajectory, newPoses...)
	st.sources = append(st.sources, newSources...)
	st.segments[seg.ID] = &segmentRecord{hash: canonicalHash(seg.Frames), result: cloneResult(res)}

	if err := m.saveReplace(); err != nil {
		st.trajectory = st.trajectory[:trajLenBefore]
		st.sources = st.sources[:srcLenBefore]
		for id, u := range undo {
			if u.ok {
				st.landmarks[id] = u.lm
			} else {
				delete(st.landmarks, id)
			}
		}
		delete(st.segments, seg.ID)
		return ImportResult{}, err
	}
	return res, nil
}

func cloneResult(r ImportResult) ImportResult {
	ids := make([]string, len(r.LandmarkIDs))
	copy(ids, r.LandmarkIDs)
	r.LandmarkIDs = ids
	return r
}

// CurrentPose 返回当前位姿及其位置方差；尚无帧导入时返回初始位姿。
func (m *Map) CurrentPose() (Pose, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return Pose{}, ErrClosed
	}
	return m.state.trajectory[len(m.state.trajectory)-1], nil
}

// PoseAt 返回不晚于 t 的最后一份历史位姿及其方差。t 早于初始时间时
// 返回 ErrNotFound。
func (m *Map) PoseAt(t int64) (Pose, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return Pose{}, ErrClosed
	}
	tr := m.state.trajectory
	// j 为第一个时间 > t 的位置。
	j := sort.Search(len(tr), func(j int) bool { return tr[j].Time > t })
	if j == 0 {
		return Pose{}, ErrNotFound
	}
	return tr[j-1], nil
}

// Rect 描述一个包含边界的轴对齐矩形查询区域，单位米。
type Rect struct {
	MinX, MinY, MaxX, MaxY float64
}

// LandmarksInRect 返回矩形（含边界）内的路标及观测次数，按标识排序；
// 区域为空时返回非 nil 的空切片。上下界颠倒或含非有限值时报 ErrInvalidRect。
func (m *Map) LandmarksInRect(r Rect) ([]Landmark, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return nil, ErrClosed
	}
	if !isFinite(r.MinX) || !isFinite(r.MinY) || !isFinite(r.MaxX) || !isFinite(r.MaxY) ||
		r.MinX > r.MaxX || r.MinY > r.MaxY {
		return nil, fmt.Errorf("%w: min bounds must not exceed max bounds and all values must be finite", ErrInvalidRect)
	}
	var out []Landmark
	for id, lm := range m.state.landmarks {
		if lm.x >= r.MinX && lm.x <= r.MaxX && lm.y >= r.MinY && lm.y <= r.MaxY {
			out = append(out, Landmark{ID: id, X: lm.x, Y: lm.y, Count: lm.count})
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	if out == nil {
		out = []Landmark{}
	}
	return out, nil
}

// AsRejectError 返回错误对应的 *RejectError；不是段拒绝错误时返回 nil、false。
func AsRejectError(err error) (*RejectError, bool) {
	var r *RejectError
	if errors.As(err, &r) {
		return r, true
	}
	return nil, false
}
