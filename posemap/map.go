package posemap

import (
	"errors"
	"fmt"
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
	config       Config
	trajectory   []Pose // 按时间严格递增；trajectory[0] 为初始位姿
	landmarks    map[string]*landmarkState
	segments     map[string]*segmentRecord
	sources      []*frameSource // sources[k] 对应 trajectory[k+1]；长度恒为 len(trajectory)-1
	corrections  []*CorrectionRecord
	corrIndex    map[string]string              // 校正标识 -> 首次内容指纹
	invalRecords []*invalidationRecord          // 按提交次序的成功失效操作
	invalIndex   map[string]*invalidationRecord // 失效标识 -> 首次记录
}

// frameSource 是生成某一帧的原始输入，供回环校正重放路标观测与累计
// 原运动方差。该切片中的 nil 元素表示对应帧来自不携带逐帧依据的旧
// 文件，涉及它的历史范围不能校正。
type frameSource struct {
	moveVariance float64
	observations []Observation
}

// landmarkState 是同一标识历次出现的聚合状态。appearances 按出现编号
// 递增保存（编号即下标+1）；最后一次出现可能仍为当前有效记录，也可能
// 已被失效。失效后的旧出现不参与新观测的合并；下一次出现从编号 1 起的
// 全新记录开始，观测计数从 1 计起。
//
// legacy* 只用于打开不携带逐帧观测来源的旧文件：旧文件中每一帧的观测
// 无法重放，因此旧文件第 1 次出现的全部观测贡献视为固定的 legacyCount
// 次平均，不随后续校正改变；旧文件之后新导入且依据完整的观测再做增量
// 平均。由本版本创建/导入的出现 legacyCount 为 0，全部观测均可重放。
type landmarkState struct {
	appearances []*occurrenceState
}

// occurrenceState 是某一标识某一次出现的状态。
type occurrenceState struct {
	x             float64
	y             float64
	count         int
	legacyCount   int
	legacyMX      float64
	legacyMY      float64
	firstSeenTime int64 // 首次观测所在帧时间
	hasFirstSeen  bool  // 旧文件缺少来源时为假
	active        bool  // 是否为当前有效记录
	invalidTime   int64 // 失效时间（提交时当前位姿时间）
	invalidReason string
	invalidOpID   string
}

// activeOccurrence 返回当前有效记录；不存在（从未出现或最近一次已失效）
// 时返回 nil。
func (lm *landmarkState) activeOccurrence() *occurrenceState {
	if len(lm.appearances) == 0 {
		return nil
	}
	last := lm.appearances[len(lm.appearances)-1]
	if !last.active {
		return nil
	}
	return last
}

// invalidationRecord 记录一次成功失效操作的首次内容与结果，供相同操作
// 标识以相同原因、相同路标集合重复提交时直接返回。
type invalidationRecord struct {
	id     string
	reason string
	hash   string // 路标集合（排序后）的指纹
	result InvalidationResult
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
		config:       cfg,
		trajectory:   []Pose{{Time: cfg.InitialTime, X: cfg.InitialX, Y: cfg.InitialY, Heading: cfg.InitialHeading, Variance: cfg.InitialVariance}},
		landmarks:    make(map[string]*landmarkState),
		segments:     make(map[string]*segmentRecord),
		corrIndex:    make(map[string]string),
		invalRecords: nil,
		invalIndex:   make(map[string]*invalidationRecord),
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
// 此前已提交的位姿、路标、观测次数不受影响。输入本身全为有限数值并不保证
// 推演结果有限：运动完成后的位置、朝向、累计位置方差，或某条观测转换到
// 地图坐标后的位置若出现 NaN/无穷值，仍以 RejectNonFinite 拒绝产生异常
// 的帧（从零开始的帧序号；路标坐标异常时携带路标标识），不会留到保存时
// 才以普通编码错误失败。已成功保存的段标识再次以
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
	// staged 记录本帧触及路标的暂存聚合：agg 按统一规则（见 aggregate.go）
	// 维护本段内逐条接纳观测的等权平均位置与计数；created 为真表示这是本段
	// 首条观测产生的新出现（此前不存在当前有效记录），firstSeen 为该出现首
	// 条观测所在帧时间。
	type stagedLM struct {
		agg       observationAgg
		created   bool
		firstSeen int64
	}
	staged := make(map[string]stagedLM) // 本段触及路标的暂存聚合
	touched := make(map[string]struct{})

	reject := func(kind string, frame int) (ImportResult, error) {
		return ImportResult{}, &RejectError{Kind: kind, Frame: frame, HasFrame: true}
	}

	for i, f := range seg.Frames {
		// 时间：严格递增；首帧必须晚于初始时间或上一段末帧。
		if f.Time <= prevTime {
			return reject(RejectTimeOrder, i)
		}
		// 相邻位姿间隔上限。时间以有符号 int64 毫秒表示且允许负值，
		// f.Time-prevTime 在 prevTime 接近 math.MinInt64 而 f.Time 为正时
		// 会溢出回绕成负数，把实际超限的大缺口放过去。此处已保证
		// f.Time > prevTime，真实差值必落在 [1, 2^64-1]，用无符号减法
		// 得到精确的数学差值再与上限比较（MaxInterval 已校验为正）。
		if uint64(f.Time)-uint64(prevTime) > uint64(m.state.config.MaxInterval) {
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
		// 输入本身有限并不保证结果有限：位置叠加、朝向累计或方差累计
		// 仍可能溢出为无穷（例如 1e308 再前进 1e308）。按数值异常拒绝
		// 产生异常的这一帧，而不是等到保存时得到普通编码错误。
		if !isFinite(cur.X) || !isFinite(cur.Y) || !isFinite(cur.Heading) || !isFinite(cur.Variance) {
			return reject(RejectNonFinite, i)
		}
		newPoses = append(newPoses, cur)

		// 观测按运动完成后的位姿转换到地图坐标，同帧按输入次序逐条处理；
		// 接纳与合并规则与回环校正重放共享同一实现（见 aggregate.go）。
		for _, ob := range f.Observations {
			s, seen := staged[ob.ID]
			if !seen {
				if lm := st.landmarks[ob.ID]; lm != nil {
					if active := lm.activeOccurrence(); active != nil {
						// 以当前有效出现的已提交观测为固定起点，本段观测在
						// 其当前均值上增量合并；暂存不修改真实状态。旧出现
						// （已失效）永不参与新观测的合并。
						s.agg.seed(active.count, active.x, active.y)
					}
				}
				if s.agg.count == 0 {
					// 从未出现或最近一次已失效：本次为新的一次出现，编号在
					// 提交时确定，首条观测计数从 1 开始。
					s.created = true
					s.firstSeen = f.Time
				}
			}
			// 转换、非有限优先、首条建点、距离判定（恰含上限）、增量平均
			// 与计数全部走统一规则。
			switch s.agg.admit(cur, ob, st.config.MergeDistance) {
			case obsNonFinite:
				// 转换结果可能因位姿/观测坐标过大而溢出：无论该路标是首次
				// 出现、失效后再次出现，还是与有效记录合并，数值异常都优先
				// 于距离冲突，按同一原因拒绝并携带路标标识。
				return ImportResult{}, &RejectError{Kind: RejectNonFinite, Frame: i, HasFrame: true, Landmark: ob.ID, HasLandmark: true}
			case obsConflict:
				return ImportResult{}, &RejectError{Kind: RejectLandmarkConflict, Frame: i, HasFrame: true, Landmark: ob.ID, HasLandmark: true}
			}
			staged[ob.ID] = s
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
		lm       *landmarkState
		ok       bool
		created  bool
		replaced *occurrenceState // 非新建时被暂存副本替换掉的原当前出现
	}
	undo := make(map[string]undoLandmark, len(staged))
	for id, s := range staged {
		old, ok := st.landmarks[id]
		if s.created {
			// 新的一次出现：可能是该标识的首次出现，也可能接在已失效的
			// 历次出现之后。编号即追加后的切片长度；位置/计数取自暂存聚合。
			occ := &occurrenceState{
				x: s.agg.x, y: s.agg.y, count: s.agg.count,
				firstSeenTime: s.firstSeen, hasFirstSeen: true,
				active: true,
			}
			lm := old
			if lm == nil {
				lm = &landmarkState{}
			}
			lm.appearances = append(lm.appearances, occ)
			st.landmarks[id] = lm
			undo[id] = undoLandmark{lm: old, ok: ok, created: true}
		} else {
			// 更新既有当前有效出现：复制原出现（保留固定旧贡献等元数据），
			// 仅把位置与计数换成暂存聚合结果，再以副本替换原指针，旧出现
			// 原样保留。
			src := old.appearances[len(old.appearances)-1]
			occ := *src
			occ.x, occ.y, occ.count = s.agg.x, s.agg.y, s.agg.count
			replaced := src
			old.appearances[len(old.appearances)-1] = &occ
			undo[id] = undoLandmark{lm: old, ok: ok, created: false, replaced: replaced}
		}
	}
	st.trajectory = append(st.trajectory, newPoses...)
	st.sources = append(st.sources, newSources...)
	st.segments[seg.ID] = &segmentRecord{hash: canonicalHash(seg.Frames), result: cloneResult(res)}

	if err := m.saveReplace(); err != nil {
		st.trajectory = st.trajectory[:trajLenBefore]
		st.sources = st.sources[:srcLenBefore]
		for id, u := range undo {
			switch {
			case !u.ok:
				delete(st.landmarks, id)
			case u.created:
				u.lm.appearances = u.lm.appearances[:len(u.lm.appearances)-1]
			default:
				u.lm.appearances[len(u.lm.appearances)-1] = u.replaced
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

// LandmarksInRect 返回矩形（含边界）内当前仍有效路标的最新一次出现及
// 观测次数，按标识排序；已失效（且尚未再次出现）的路标立即排除，区域为
// 空时返回非 nil 的空切片。上下界颠倒或含非有限值时报 ErrInvalidRect。
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
		occ := lm.activeOccurrence()
		if occ == nil {
			continue // 最近一次出现已失效：立即排除。
		}
		if occ.x >= r.MinX && occ.x <= r.MaxX && occ.y >= r.MinY && occ.y <= r.MaxY {
			out = append(out, Landmark{ID: id, X: occ.x, Y: occ.y, Count: occ.count})
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
