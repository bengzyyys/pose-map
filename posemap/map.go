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
	config       Config
	trajectory   []Pose // 按时间严格递增；trajectory[0] 为初始位姿
	landmarks    map[string]*landmarkState
	segments     map[string]*segmentRecord
	observations [][]Observation // 与 trajectory 平行；observations[i] 为第 i 帧的逐帧观测来源
	// （nil 条目表示该帧来源未知——旧版文件导入的帧；非 nil 空切片表示该帧无观测且来源完整）
	corrections []CorrectionRecord // 按提交次序留档的成功校正记录
}

// landmarkState 是路标在地图坐标下的聚合状态。
type landmarkState struct {
	x     float64
	y     float64
	count int
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
		observations: [][]Observation{{}}, // 初始位姿无观测，但来源完整（已知为零观测）
		corrections:  nil,
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

	// ---- 全部校验通过：提交内存状态并落盘；落盘失败精确回滚 ----
	trajLenBefore := len(st.trajectory)
	obsLenBefore := len(st.observations)
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
	// 逐帧观测来源与轨迹平行保存，供日后回环校正重放；零观测帧存非 nil 空切片。
	newObs := make([][]Observation, 0, len(seg.Frames))
	for _, f := range seg.Frames {
		obs := f.Observations
		if obs == nil {
			obs = []Observation{}
		}
		newObs = append(newObs, obs)
	}
	st.observations = append(st.observations, newObs...)
	st.segments[seg.ID] = &segmentRecord{hash: canonicalHash(seg.Frames), result: cloneResult(res)}

	if err := m.saveReplace(); err != nil {
		st.trajectory = st.trajectory[:trajLenBefore]
		st.observations = st.observations[:obsLenBefore]
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

// CorrectLoop 应用一次“已确认回环”校正：锚点帧（必须精确命中已导入帧，
// 且不得为初始位姿）采用提交的目标位姿；从锚点到提交时末帧的轨迹整体
// 平移加旋转，保持这些帧在校正前彼此的相对位置与朝向，时间不变，朝向
// 沿用归一范围；锚点之后每帧的方差等于目标方差加锚点之后至该帧的原
// 运动方差累计值。受影响帧的路标观测随位姿改变地图位置，更早观测的
// 地图位置不变；同一路标仍按全部已接受观测取平均，标识与观测次数不
// 变。校正后的观测仍按原时间与同帧输入次序遵守合并距离限制，冲突则
// 整次拒绝。
//
// 整次校正要么全部生效（含落盘），要么因可区分的 *RejectError 被整次
// 拒绝，此前已提交的位姿、路标、校正记录不受影响，查询看不到部分更
// 新。同一校正标识再次以相同内容提交时直接返回首次结果且不改变数据；
// 同一标识对应不同内容则以 RejectCorrectionDuplicateMismatch 拒绝。
func (m *Map) CorrectLoop(lc LoopCorrection) (CorrectionRecord, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return CorrectionRecord{}, ErrClosed
	}
	st := m.state

	// 校正标识非空。
	if lc.ID == "" {
		return CorrectionRecord{}, &RejectError{Kind: RejectEmptyCorrectionID}
	}

	// 已保存校正：相同内容直接返回首次结果；不同内容明确拒绝。
	if dup := findCorrection(st.corrections, lc.ID); dup != nil {
		if sameCorrectionContent(dup, lc) {
			return cloneCorrectionRecord(*dup), nil
		}
		return CorrectionRecord{}, &RejectError{Kind: RejectCorrectionDuplicateMismatch, Landmark: lc.ID, HasLandmark: true}
	}

	// 目标数值有限、方差非负。
	if !isFinite(lc.X) || !isFinite(lc.Y) || !isFinite(lc.Heading) || !isFinite(lc.Variance) {
		return CorrectionRecord{}, &RejectError{Kind: RejectCorrectionNonFinite}
	}
	if lc.Variance < 0 {
		return CorrectionRecord{}, &RejectError{Kind: RejectNegativeTargetVariance}
	}

	// 锚点：必须精确命中已导入帧，且不得为初始位姿。
	anchorIdx := -1
	for i := range st.trajectory {
		if st.trajectory[i].Time == lc.AnchorTime {
			anchorIdx = i
			break
		}
	}
	if anchorIdx < 1 { // 未命中或命中初始位姿（trajectory[0]）
		return CorrectionRecord{}, &RejectError{Kind: RejectAnchorNotFound}
	}
	endIdx := len(st.trajectory) - 1

	// 受影响范围 [anchorIdx, endIdx] 必须具备完整逐帧观测依据。
	if len(st.observations) != len(st.trajectory) {
		return CorrectionRecord{}, &RejectError{Kind: RejectIncompleteBasis}
	}
	for i := anchorIdx; i <= endIdx; i++ {
		if st.observations[i] == nil {
			return CorrectionRecord{}, &RejectError{Kind: RejectIncompleteBasis}
		}
	}

	// ---- 计算校正后位姿（暂存在 newPoses，真实状态此时不变）----
	oldAnchor := st.trajectory[anchorIdx]
	targetHeading := normalizeAngle(lc.Heading)
	theta := normalizeAngle(targetHeading - oldAnchor.Heading)
	cosT, sinT := math.Cos(theta), math.Sin(theta)
	// 旋转 R(theta) 加平移 t，使旧锚点映射到目标位姿。
	tx := lc.X - (cosT*oldAnchor.X - sinT*oldAnchor.Y)
	ty := lc.Y - (sinT*oldAnchor.X + cosT*oldAnchor.Y)

	newPoses := make([]Pose, len(st.trajectory))
	copy(newPoses, st.trajectory)
	newPoses[anchorIdx] = Pose{Time: oldAnchor.Time, X: lc.X, Y: lc.Y, Heading: targetHeading, Variance: lc.Variance}
	for i := anchorIdx + 1; i <= endIdx; i++ {
		p := st.trajectory[i]
		nx := cosT*p.X - sinT*p.Y + tx
		ny := sinT*p.X + cosT*p.Y + ty
		nh := normalizeAngle(p.Heading + theta)
		// 锚点之后每帧方差 = 目标方差 + 锚点至该帧的原运动方差累计。
		nv := lc.Variance + (p.Variance - oldAnchor.Variance)
		if !isFinite(nx) || !isFinite(ny) || !isFinite(nh) || !isFinite(nv) {
			return CorrectionRecord{}, &RejectError{Kind: RejectCorrectionNonFinite}
		}
		newPoses[i] = Pose{Time: p.Time, X: nx, Y: ny, Heading: nh, Variance: nv}
	}

	// ---- 重放受影响路标：先按旧位姿扣减受影响观测，再按校正后位姿按原次序重放 ----
	affectedIDs := make(map[string]struct{})
	for i := anchorIdx; i <= endIdx; i++ {
		for _, ob := range st.observations[i] {
			affectedIDs[ob.ID] = struct{}{}
		}
	}
	newLandmarks := make(map[string]*landmarkState, len(st.landmarks))
	for id, lm := range st.landmarks {
		cp := *lm
		newLandmarks[id] = &cp
	}

	affectedList := make([]string, 0, len(affectedIDs))
	for id := range affectedIDs {
		affectedList = append(affectedList, id)
	}
	sort.Strings(affectedList)

	for _, id := range affectedList {
		cur := st.landmarks[id]
		// 收集 [anchorIdx, endIdx] 内该路标的观测，按帧次序、同帧输入次序。
		var postObs []frameObs
		for i := anchorIdx; i <= endIdx; i++ {
			for _, ob := range st.observations[i] {
				if ob.ID == id {
					postObs = append(postObs, frameObs{frameIdx: i, ob: ob})
				}
			}
		}
		// 用旧位姿计算这些观测的地图坐标和，从当前聚合中扣减，得到锚点前聚合。
		var sumPostX, sumPostY float64
		for _, po := range postObs {
			p := st.trajectory[po.frameIdx]
			mx, my := localToMap(p.X, p.Y, p.Heading, po.ob.X, po.ob.Y)
			sumPostX += mx
			sumPostY += my
		}
		preCount := cur.count - len(postObs)
		preSumX := cur.x*float64(cur.count) - sumPostX
		preSumY := cur.y*float64(cur.count) - sumPostY

		var aggX, aggY float64
		var aggCount int
		if preCount > 0 {
			aggX = preSumX / float64(preCount)
			aggY = preSumY / float64(preCount)
			aggCount = preCount
		}

		// 按校正后位姿重放，仍按原时间与同帧输入次序遵守合并距离限制。
		for _, po := range postObs {
			p := newPoses[po.frameIdx]
			mx, my := localToMap(p.X, p.Y, p.Heading, po.ob.X, po.ob.Y)
			if !isFinite(mx) || !isFinite(my) {
				return CorrectionRecord{}, &RejectError{Kind: RejectCorrectionNonFinite, Landmark: id, HasLandmark: true}
			}
			if aggCount > 0 {
				if math.Hypot(mx-aggX, my-aggY) > st.config.MergeDistance {
					return CorrectionRecord{}, &RejectError{
						Kind: RejectLandmarkConflict, Frame: po.frameIdx, HasFrame: true,
						FrameTime: p.Time, HasFrameTime: true, Landmark: id, HasLandmark: true,
					}
				}
				aggX = (aggX*float64(aggCount) + mx) / float64(aggCount+1)
				aggY = (aggY*float64(aggCount) + my) / float64(aggCount+1)
				aggCount++
			} else {
				aggX, aggY, aggCount = mx, my, 1
			}
		}
		newLandmarks[id] = &landmarkState{x: aggX, y: aggY, count: aggCount}
	}

	// ---- 组装留档（含受影响位姿与路标的校正前后值）----
	rec := CorrectionRecord{
		ID:         lc.ID,
		AnchorTime: lc.AnchorTime,
		Target:     Pose{Time: oldAnchor.Time, X: lc.X, Y: lc.Y, Heading: targetHeading, Variance: lc.Variance},
		EndTime:    st.trajectory[endIdx].Time,
	}
	for i := anchorIdx; i <= endIdx; i++ {
		rec.Poses = append(rec.Poses, CorrectedPose{
			Time:   st.trajectory[i].Time,
			Before: st.trajectory[i],
			After:  newPoses[i],
		})
	}
	for _, id := range affectedList {
		before := st.landmarks[id]
		after := newLandmarks[id]
		rec.Landmarks = append(rec.Landmarks, CorrectedLandmark{
			ID:     id,
			Before: Landmark{ID: id, X: before.x, Y: before.y, Count: before.count},
			After:  Landmark{ID: id, X: after.x, Y: after.y, Count: after.count},
		})
	}

	// ---- 提交内存状态并落盘；落盘失败精确回滚 ----
	oldTraj := st.trajectory
	oldLMs := st.landmarks
	corrLenBefore := len(st.corrections)
	st.trajectory = newPoses
	st.landmarks = newLandmarks
	st.corrections = append(st.corrections, rec)

	if err := m.saveReplace(); err != nil {
		st.trajectory = oldTraj
		st.landmarks = oldLMs
		st.corrections = st.corrections[:corrLenBefore]
		return CorrectionRecord{}, err
	}
	return cloneCorrectionRecord(rec), nil
}

// Corrections 返回按提交次序留档的成功校正记录。
func (m *Map) Corrections() ([]CorrectionRecord, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return nil, ErrClosed
	}
	out := make([]CorrectionRecord, len(m.state.corrections))
	for i, rec := range m.state.corrections {
		out[i] = cloneCorrectionRecord(rec)
	}
	return out, nil
}

// frameObs 是重放路标时的一条观测：所在轨迹帧序号与观测内容。
type frameObs struct {
	frameIdx int
	ob       Observation
}

// findCorrection 返回已保存的同标识校正记录，不存在返回 nil。
func findCorrection(recs []CorrectionRecord, id string) *CorrectionRecord {
	for i := range recs {
		if recs[i].ID == id {
			return &recs[i]
		}
	}
	return nil
}

// sameCorrectionContent 判断两次校正提交的内容（锚点时间与目标位姿）是否相同。
func sameCorrectionContent(rec *CorrectionRecord, lc LoopCorrection) bool {
	return rec.AnchorTime == lc.AnchorTime &&
		rec.Target.X == lc.X && rec.Target.Y == lc.Y &&
		rec.Target.Heading == lc.Heading && rec.Target.Variance == lc.Variance
}

// cloneCorrectionRecord 深拷贝校正记录的切片字段。
func cloneCorrectionRecord(rec CorrectionRecord) CorrectionRecord {
	if rec.Poses != nil {
		cp := make([]CorrectedPose, len(rec.Poses))
		copy(cp, rec.Poses)
		rec.Poses = cp
	}
	if rec.Landmarks != nil {
		cp := make([]CorrectedLandmark, len(rec.Landmarks))
		copy(cp, rec.Landmarks)
		rec.Landmarks = cp
	}
	return rec
}
