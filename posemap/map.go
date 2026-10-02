package posemap

import (
	"crypto/sha256"
	"encoding/binary"
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
	config        Config
	trajectory    []Pose // 按时间严格递增；trajectory[0] 为初始位姿
	landmarks     map[string]*landmarkState
	segments      map[string]*segmentRecord
	sources       []*frameSource // sources[k] 对应 trajectory[k+1]；长度恒为 len(trajectory)-1
	corrections   []*CorrectionRecord
	corrIndex     map[string]string              // 校正标识 -> 首次内容指纹
	invalidations map[string]*invalidationRecord // 失效操作标识 -> 首次记录
}

// frameSource 是生成某一帧的原始输入，供回环校正重放路标观测与累计
// 原运动方差。该切片中的 nil 元素表示对应帧来自不携带逐帧依据的旧
// 文件，涉及它的历史范围不能校正。
type frameSource struct {
	moveVariance float64
	observations []Observation
}

// landmarkState 是路标在地图坐标下的聚合状态，按出现次数组织。
//
// 同一标识首次出现编号为 1，每次失效后在新的成功导入轨迹中再次出现时
// 编号递增。各次出现独立聚合：旧记录不参与新记录的合并，校正时各次出现
// 分别重放观测并遵守合并距离限制。
//
// legacy* 只用于打开不携带逐帧观测来源的旧文件，存于首次出现记录：旧文件
// 中该路标的全部观测贡献视为固定的 legacyCount 次平均，不随后续校正改变；
// 旧文件之后新导入且依据完整的观测再做增量平均。
type landmarkState struct {
	appearances []*landmarkAppearance
}

// landmarkAppearance 是路标的一次出现记录。
type landmarkAppearance struct {
	number        int
	x, y          float64
	count         int
	valid         bool
	firstTime     int64 // 首次观测时间；旧文件缺少来源时为 -1
	invalidTime   int64
	invalidReason string
	legacyCount   int
	legacyMX      float64
	legacyMY      float64
}

// currentValid 返回当前有效的出现记录；没有则返回 nil。
func (lm *landmarkState) currentValid() *landmarkAppearance {
	if lm == nil || len(lm.appearances) == 0 {
		return nil
	}
	last := lm.appearances[len(lm.appearances)-1]
	if last.valid {
		return last
	}
	return nil
}

// appearanceByNumber 返回指定编号的出现记录；不存在返回 nil。
func (lm *landmarkState) appearanceByNumber(n int) *landmarkAppearance {
	if lm == nil {
		return nil
	}
	for _, a := range lm.appearances {
		if a.number == n {
			return a
		}
	}
	return nil
}

// appearanceAt 返回帧时间 t 所属的出现记录索引；不属任何出现返回 -1。
// 旧文件首次记录 firstTime=-1，覆盖下一次出现之前的全部帧。
func (lm *landmarkState) appearanceAt(t int64) int {
	if lm == nil {
		return -1
	}
	idx := -1
	for i, a := range lm.appearances {
		if a.firstTime <= t {
			idx = i
		} else {
			break
		}
	}
	return idx
}

// invalidationRecord 记录一次已成功保存的失效操作：原因、路标集合与
// 首次结果，供相同内容重复提交时直接返回。
type invalidationRecord struct {
	reason      string
	landmarkIDs []string // 已排序，便于比较集合
	result      InvalidateResult
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
		config:        cfg,
		trajectory:    []Pose{{Time: cfg.InitialTime, X: cfg.InitialX, Y: cfg.InitialY, Heading: cfg.InitialHeading, Variance: cfg.InitialVariance}},
		landmarks:     make(map[string]*landmarkState),
		segments:      make(map[string]*segmentRecord),
		corrIndex:     make(map[string]string),
		invalidations: make(map[string]*invalidationRecord),
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
					// 复制已提交路标（含各次出现），避免提前修改真实状态。
					cp := &landmarkState{}
					cp.appearances = make([]*landmarkAppearance, len(lm.appearances))
					for k, a := range lm.appearances {
						acp := *a
						cp.appearances[k] = &acp
					}
					lm = cp
				}
			}
			if a := lm.currentValid(); a != nil {
				// 已有当前有效记录：沿用合并距离限制并增量平均。
				if math.Hypot(mx-a.x, my-a.y) > m.state.config.MergeDistance {
					return ImportResult{}, &RejectError{Kind: RejectLandmarkConflict, Frame: i, HasFrame: true, Landmark: ob.ID, HasLandmark: true}
				}
				a.x = (a.x*float64(a.count) + mx) / float64(a.count+1)
				a.y = (a.y*float64(a.count) + my) / float64(a.count+1)
				a.count++
			} else {
				// 无当前有效记录（首次出现或失效后再次出现）：新建一次出现，
				// 位置来自首次新观测，观测计数从 1 开始，旧记录不参加合并。
				num := 1
				if lm != nil && len(lm.appearances) > 0 {
					num = lm.appearances[len(lm.appearances)-1].number + 1
				} else {
					lm = &landmarkState{}
				}
				lm.appearances = append(lm.appearances, &landmarkAppearance{
					number:    num,
					x:         mx,
					y:         my,
					count:     1,
					valid:     true,
					firstTime: f.Time,
				})
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
		a := lm.currentValid()
		if a == nil {
			continue // 无当前有效记录：区域查询不展示
		}
		if a.x >= r.MinX && a.x <= r.MaxX && a.y >= r.MinY && a.y <= r.MaxY {
			out = append(out, Landmark{ID: id, X: a.x, Y: a.y, Count: a.count})
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	if out == nil {
		out = []Landmark{}
	}
	return out, nil
}

// LandmarkHistory 返回指定标识的全部出现记录，按编号递增排列。未知标识
// 返回 ErrNotFound。未失效的出现记录中 InvalidTime 与 InvalidReason 为
// 零值，FirstTime 为 -1 表示首次观测时间未知（旧文件缺少来源）。
func (m *Map) LandmarkHistory(id string) (LandmarkRecord, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return LandmarkRecord{}, ErrClosed
	}
	lm := m.state.landmarks[id]
	if lm == nil {
		return LandmarkRecord{}, ErrNotFound
	}
	out := LandmarkRecord{ID: id, Appearances: make([]LandmarkAppearance, 0, len(lm.appearances))}
	for _, a := range lm.appearances {
		out.Appearances = append(out.Appearances, LandmarkAppearance{
			Number:        a.number,
			X:             a.x,
			Y:             a.y,
			Count:         a.count,
			Valid:         a.valid,
			FirstTime:     a.firstTime,
			InvalidTime:   a.invalidTime,
			InvalidReason: a.invalidReason,
		})
	}
	return out, nil
}

// Invalidate 提交一次路标失效请求。
//
// 操作标识与原因必须非空，路标列表必须非空且不含空标识或重复标识；路标
// 必须存在且当前有效。全部校验通过后，对这些路标的当前有效记录同时生效，
// 失效时间为提交时的当前位姿时间，返回失效时间和各路标的出现编号。区域
// 查询立即排除这些路标，位姿与已接受的观测不被删除。
//
// 同一失效操作标识以相同原因、相同路标集合重复提交返回首次结果，即使
// 路标后来再次出现也不再撤下；同一标识不同内容明确拒绝。任何拒绝或保存
// 失败都不留下部分变化。
func (m *Map) Invalidate(req InvalidateRequest) (InvalidateResult, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return InvalidateResult{}, ErrClosed
	}
	st := m.state

	// 已保存失效操作：相同内容直接返回首次结果；不同内容明确拒绝。
	if rec, ok := st.invalidations[req.ID]; ok {
		if canonicalInvalidationHash(req.Reason, req.LandmarkIDs) == canonicalInvalidationHash(rec.reason, rec.landmarkIDs) {
			return cloneInvalidateResult(rec.result), nil
		}
		return InvalidateResult{}, &RejectError{Kind: RejectInvalidationMismatch, Landmark: req.ID, HasLandmark: true}
	}

	if req.ID == "" {
		return InvalidateResult{}, &RejectError{Kind: RejectEmptyID}
	}
	if req.Reason == "" {
		return InvalidateResult{}, &RejectError{Kind: RejectEmptyReason}
	}
	if len(req.LandmarkIDs) == 0 {
		return InvalidateResult{}, &RejectError{Kind: RejectEmptyLandmarkList}
	}

	// 校验列表：无空标识、无重复。
	seen := make(map[string]struct{}, len(req.LandmarkIDs))
	dups := make([]string, 0)
	for _, id := range req.LandmarkIDs {
		if id == "" {
			return InvalidateResult{}, &RejectError{Kind: RejectEmptyID}
		}
		if _, ok := seen[id]; ok {
			dups = append(dups, id)
		}
		seen[id] = struct{}{}
	}
	if len(dups) > 0 {
		return InvalidateResult{}, &RejectError{Kind: RejectDuplicateLandmark, Landmarks: dups}
	}

	// 校验路标存在且当前有效。
	notFound := make([]string, 0)
	alreadyInvalid := make([]string, 0)
	for _, id := range req.LandmarkIDs {
		lm := st.landmarks[id]
		if lm == nil {
			notFound = append(notFound, id)
			continue
		}
		if lm.currentValid() == nil {
			alreadyInvalid = append(alreadyInvalid, id)
		}
	}
	if len(notFound) > 0 {
		return InvalidateResult{}, &RejectError{Kind: RejectLandmarkNotFound, Landmarks: notFound}
	}
	if len(alreadyInvalid) > 0 {
		return InvalidateResult{}, &RejectError{Kind: RejectLandmarkAlreadyInvalid, Landmarks: alreadyInvalid}
	}

	// ---- 全部校验通过：暂存失效并落盘；落盘失败精确回滚 ----
	invalidTime := st.trajectory[len(st.trajectory)-1].Time
	result := InvalidateResult{InvalidTime: invalidTime, Appearances: make(map[string]int, len(req.LandmarkIDs))}
	type undoEntry struct {
		lm         *landmarkState
		appearance *landmarkAppearance
		wasValid   bool
		oldTime    int64
		oldReason  string
	}
	undo := make([]undoEntry, 0, len(req.LandmarkIDs))
	for _, id := range req.LandmarkIDs {
		lm := st.landmarks[id]
		a := lm.currentValid()
		result.Appearances[id] = a.number
		undo = append(undo, undoEntry{lm: lm, appearance: a, wasValid: a.valid, oldTime: a.invalidTime, oldReason: a.invalidReason})
		a.valid = false
		a.invalidTime = invalidTime
		a.invalidReason = req.Reason
	}

	sortedIDs := append([]string(nil), req.LandmarkIDs...)
	sort.Strings(sortedIDs)
	st.invalidations[req.ID] = &invalidationRecord{
		reason:      req.Reason,
		landmarkIDs: sortedIDs,
		result:      cloneInvalidateResult(result),
	}

	if err := m.saveReplace(); err != nil {
		for _, u := range undo {
			u.appearance.valid = u.wasValid
			u.appearance.invalidTime = u.oldTime
			u.appearance.invalidReason = u.oldReason
		}
		delete(st.invalidations, req.ID)
		return InvalidateResult{}, err
	}
	return result, nil
}

func cloneInvalidateResult(r InvalidateResult) InvalidateResult {
	cp := InvalidateResult{InvalidTime: r.InvalidTime, Appearances: make(map[string]int, len(r.Appearances))}
	for k, v := range r.Appearances {
		cp.Appearances[k] = v
	}
	return cp
}

// canonicalInvalidationHash 计算一次失效请求的确定性指纹，用于同标识重复
// 提交判定。原因与路标集合（排序后）参与哈希。
func canonicalInvalidationHash(reason string, ids []string) string {
	h := sha256.New()
	var lenBuf [4]byte
	putU32 := func(v uint32) {
		binary.BigEndian.PutUint32(lenBuf[:], v)
		h.Write(lenBuf[:])
	}
	putStr := func(s string) {
		putU32(uint32(len(s)))
		h.Write([]byte(s))
	}
	putStr(reason)
	sorted := append([]string(nil), ids...)
	sort.Strings(sorted)
	putU32(uint32(len(sorted)))
	for _, id := range sorted {
		putStr(id)
	}
	return fmt.Sprintf("%x", h.Sum(nil))
}

// AsRejectError 返回错误对应的 *RejectError；不是段拒绝错误时返回 nil、false。
func AsRejectError(err error) (*RejectError, bool) {
	var r *RejectError
	if errors.As(err, &r) {
		return r, true
	}
	return nil, false
}
