package posemap

import (
	"math"
	"sort"
)

// poseShift 描述一次回环校正对轨迹施加的同一次刚体重定位：锚点落到目标
// 位置、朝向归一到 [-π,π)；从锚点到末帧的其余帧与锚点一起平移旋转，保留
// 各帧相对锚点的位置关系与朝向差。提交校正（Correct）与打开地图时的校正
// 记录几何核对（validateLoaded，见 persistence.go）共用这同一份规则与
// 运算次序，两处对“校正后位置与朝向”的理解因此始终一致。
type poseShift struct {
	x, y          float64 // 目标位置，即锚点校正后位置
	heading       float64 // 目标朝向归一到 [-π,π)，即锚点校正后朝向
	anchorX       float64 // 锚点校正前位置
	anchorY       float64
	anchorHeading float64 // 锚点校正前朝向
	dHeading      float64 // 归一后的最短朝向差，即各帧叠加的旋转角
	cosD, sinD    float64 // 旋转角的余弦与正弦
	identity      bool    // 只调方差的恒等校正：位置与朝向保持原值
}

// newPoseShift 按提交目标与锚点校正前位姿构造重定位。目标朝向先归一，
// 旋转角取归一后的最短朝向差。目标位置与锚点原位置完全相同、目标朝向归
// 一后与原朝向等价（旋转角为 0）时是恒等校正：几何上没有任何平移或旋
// 转，校正前后位姿必须完全相同。恒等校正不能套用旋转平移公式——公式会
// 先求相对锚点的偏移，轨迹横跨 ±1e308 这类大坐标时该差值先溢出为
// ±Inf、再参与浮点运算退化为 NaN，使本应成功的校正被误判为数值异常；
// 恒等校正直接保留原位姿。
func newPoseShift(tgt CorrectionTarget, anchorBefore Pose) poseShift {
	heading := normalizeAngle(tgt.Heading)
	dHeading := normalizeAngle(heading - anchorBefore.Heading)
	return poseShift{
		x:             tgt.X,
		y:             tgt.Y,
		heading:       heading,
		anchorX:       anchorBefore.X,
		anchorY:       anchorBefore.Y,
		anchorHeading: anchorBefore.Heading,
		dHeading:      dHeading,
		cosD:          math.Cos(dHeading),
		sinD:          math.Sin(dHeading),
		identity:      dHeading == 0 && tgt.X == anchorBefore.X && tgt.Y == anchorBefore.Y,
	}
}

// apply 计算一帧校正后的位置与朝向：恒等校正原样保留校正前值；否则把校
// 正前相对锚点的偏移随锚点一起旋转平移。帧时间与方差不属于几何规则，由
// 调用方处理。
//
// 校正后朝向按“目标朝向 + 该校正前相对锚点的朝向差”计算，而不是把旋转
// 角加到校正前朝向上：旋转角 dHeading 本身是目标朝向减去锚点朝向的舍入
// 结果，锚点朝向为 1 弧度而目标朝向为 1e-20 这类可表示的小角度时，差值
// 会舍入成 -1，再加回锚点朝向就把目标朝向丢成了零。先求相对朝向差（锚
// 点自身与原本朝向相同的帧差值恰为 0）再叠加到目标朝向上，锚点帧精确落
// 在归一后的目标朝向，原本与锚点朝向完全相同的帧也得到同一目标朝向，
// 其余帧保留与锚点的相对方向；结果仍归一到 [-π,π)。
func (s poseShift) apply(before Pose) (x, y, heading float64) {
	if s.identity {
		return before.X, before.Y, before.Heading
	}
	dx := before.X - s.anchorX
	dy := before.Y - s.anchorY
	return s.x + s.cosD*dx - s.sinD*dy,
		s.y + s.sinD*dx + s.cosD*dy,
		normalizeAngle(s.heading + normalizeAngle(before.Heading-s.anchorHeading))
}

// Correct 提交一次已确认回环校正。
//
// req.ID 必须非空；req.Anchor 必须准确命中一个已导入帧的时间（不能是
// 初始位姿，也不借用历史查询得到的最近一帧）；目标位姿数值必须有限、
// 目标方差必须非负。命中后，锚点帧采用提交的位姿；从锚点到提交时末帧
// 的轨迹整体平移并旋转（保持这些帧在校正前彼此的相对位置与朝向，时间
// 不变，朝向归一到既有范围）；锚点之后每帧的方差等于目标方差加上锚点
// 之后至该帧的原运动方差累计值。更早的位姿不变。
//
// 受影响帧中的路标观测随新位姿改变地图位置；更早观测的地图位置不变。
// 同一路标仍按全部已接受观测、依原时间与同帧输入次序做增量平均并遵守
// 合并距离限制；观测次数不变。若校正后出现路标冲突，整次校正拒绝并说
// 明冲突帧时间与路标。输入或校正结果含非有限数值同样拒绝；观测转换到
// 地图坐标后溢出为 NaN/无穷时按数值异常拒绝，说明该观测所在帧的实际
// 时间、路标标识与出现编号，而不是误报为距离冲突。
//
// 同一标识以相同内容重复提交返回首次结果且不再修改数据；同一标识不同
// 内容明确拒绝。任何拒绝或保存失败都不改变轨迹、路标与既有校正记录。
// 旧文件缺少逐帧观测来源时，覆盖其历史范围的校正返回 RejectNoBasis。
func (m *Map) Correct(req Correction) (CorrectionRecord, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return CorrectionRecord{}, ErrClosed
	}
	st := m.state

	// 已保存校正：相同内容直接返回首次记录；不同内容明确拒绝。
	// 不做任何校验或状态变更，即使其后又提交过其他校正。
	if hash, ok := st.corrIndex[req.ID]; ok {
		if hash == canonicalCorrectionHash(req.Anchor, req.Target) {
			return cloneRecord(st.correctionRecordByID(req.ID)), nil
		}
		return CorrectionRecord{}, &RejectError{Kind: RejectCorrectionMismatch, Landmark: req.ID, HasLandmark: true}
	}

	if req.ID == "" {
		return CorrectionRecord{}, &RejectError{Kind: RejectEmptyID}
	}
	tgt := req.Target
	if !isFinite(tgt.X) || !isFinite(tgt.Y) || !isFinite(tgt.Heading) || !isFinite(tgt.Variance) {
		return CorrectionRecord{}, &RejectError{Kind: RejectNonFinite, Time: req.Anchor, HasTime: true}
	}
	if tgt.Variance < 0 {
		return CorrectionRecord{}, &RejectError{Kind: RejectNegativeVariance, Time: req.Anchor, HasTime: true}
	}

	// 锚点必须准确命中已导入帧：trajectory[0] 是初始位姿，不能校正。
	anchorIdx := -1
	for j := 1; j < len(st.trajectory); j++ {
		if st.trajectory[j].Time == req.Anchor {
			anchorIdx = j
			break
		}
	}
	if anchorIdx < 0 {
		return CorrectionRecord{}, &RejectError{Kind: RejectAnchorNotFound, Time: req.Anchor, HasTime: true}
	}
	end := len(st.trajectory) - 1

	// 受影响范围内每一帧都必须有逐帧依据（旧文件没有，不能靠推测校正）。
	for k := anchorIdx - 1; k < len(st.sources); k++ {
		if st.sources[k] == nil {
			return CorrectionRecord{}, &RejectError{Kind: RejectNoBasis, Time: req.Anchor, HasTime: true}
		}
	}

	oldAnchor := st.trajectory[anchorIdx]
	// 锚点到末帧的整体平移旋转规则（含只调方差的恒等情形）与打开地图时
	// 的记录几何核对共享同一实现，见上方 poseShift。
	shift := newPoseShift(tgt, oldAnchor)

	// ---- 在副本上推演新位姿，真实状态此时保持不变 ----
	next := append([]Pose(nil), st.trajectory...)
	accumVar := tgt.Variance
	for j := anchorIdx; j <= end; j++ {
		old := st.trajectory[j]
		if j > anchorIdx {
			accumVar += st.sources[j-1].moveVariance
		}
		x, y, heading := shift.apply(old)
		np := Pose{
			Time:     old.Time,
			X:        x,
			Y:        y,
			Heading:  heading,
			Variance: accumVar,
		}
		if !isFinite(np.X) || !isFinite(np.Y) || !isFinite(np.Heading) || !isFinite(np.Variance) {
			return CorrectionRecord{}, &RejectError{Kind: RejectNonFinite, Time: req.Anchor, HasTime: true}
		}
		next[j] = np
	}

	// 重放器：观测按帧时间归入当时那次出现、旧观测固定贡献作为起点、
	// 逐条重建位置的规则与打开地图时的核对共享同一实现（见 replay.go）。
	replay := newObservationReplay(st.config.MergeDistance)
	for id, lm := range st.landmarks {
		replay.track(id, lm.occurrenceWindows())
	}
	// 重放帧序列：受影响帧（锚点至末帧）用校正后位姿，更早帧用原位姿；
	// 旧文件帧没有逐帧依据，其观测已计入对应出现的固定旧贡献，这里按
	// 空观测帧处理。
	frames := make([]replayFrame, 0, end)
	for j := 1; j <= end; j++ {
		var obs []Observation
		if src := st.sources[j-1]; src != nil {
			obs = src.observations
		}
		frames = append(frames, replayFrame{time: st.trajectory[j].Time, pose: next[j], observations: obs})
	}

	// 受影响的路标出现：在受影响帧中被观测到的全部（标识，出现编号）。
	// 回环校正跨越同一路标的多次出现时，各次出现分别聚合、分别遵守合并
	// 距离限制；已接受观测必然属于某次出现。
	affected := replay.observed(frames[anchorIdx-1:])
	affectedKeys := make([]occurrenceKey, 0, len(affected))
	for k := range affected {
		affectedKeys = append(affectedKeys, k)
	}
	sort.Slice(affectedKeys, func(i, j int) bool {
		if affectedKeys[i].id != affectedKeys[j].id {
			return affectedKeys[i].id < affectedKeys[j].id
		}
		return affectedKeys[i].num < affectedKeys[j].num
	})

	// 逐“出现”登记固定旧贡献：旧文件第 1 次出现的既有观测无法重放，
	// 其贡献固定为文件中的聚合值（位置不随校正改变）；其余观测随帧序列
	// 重放。同一路标的不同出现互不参与彼此的合并。
	for _, k := range affectedKeys {
		occ := st.landmarks[k.id].appearances[k.num-1]
		replay.seed(k.id, k.num, occ.legacyCount, occ.legacyMX, occ.legacyMY)
	}
	// 目标、原始观测与校正后位姿都有限，转换结果仍可能因数值过大
	// 溢出为 NaN/无穷。与导入一致：数值异常优先于距离冲突，按
	// non_finite 拒绝并指出该观测所在帧的实际时间、路标标识与
	// 其所属的出现编号（不是锚点时间），无论该次出现此前是否已有
	// 观测、记录仍有效还是已失效。
	if rej := replay.replay(frames); rej != nil {
		switch rej.reason {
		case replayNonFinite:
			return CorrectionRecord{}, &RejectError{
				Kind:          RejectNonFinite,
				Time:          rej.time,
				HasTime:       true,
				Landmark:      rej.id,
				HasLandmark:   true,
				Occurrence:    rej.num,
				HasOccurrence: true,
			}
		case replayConflict:
			return CorrectionRecord{}, &RejectError{
				Kind:          RejectLandmarkConflict,
				Time:          rej.time,
				HasTime:       true,
				Landmark:      rej.id,
				HasLandmark:   true,
				Occurrence:    rej.num,
				HasOccurrence: true,
			}
		default:
			// 已接受观测必然属于某个已登记出现；归属失败说明内存不变量
			// 被破坏，不可能由本包产生，按损坏处理而非半提交。
			return CorrectionRecord{}, ErrCorrupt
		}
	}
	for _, k := range affectedKeys {
		a := replay.agg(k.id, k.num)
		occ := st.landmarks[k.id].appearances[k.num-1]
		if !isFinite(a.x) || !isFinite(a.y) {
			return CorrectionRecord{}, &RejectError{Kind: RejectNonFinite, Time: req.Anchor, HasTime: true}
		}
		if a.count != occ.count {
			// 内存不变量被破坏不可能由本包写出，按损坏处理而非半提交。
			return CorrectionRecord{}, ErrCorrupt
		}
	}

	// ---- 组装校正前后快照（记录不可变） ----
	rec := &CorrectionRecord{
		ID:        req.ID,
		Anchor:    req.Anchor,
		Target:    tgt,
		EndTime:   st.trajectory[end].Time,
		Poses:     make([]PoseChange, 0, end-anchorIdx+1),
		Landmarks: make([]LandmarkChange, 0, len(affectedKeys)),
	}
	for j := anchorIdx; j <= end; j++ {
		rec.Poses = append(rec.Poses, PoseChange{Before: st.trajectory[j], After: next[j]})
	}
	for _, k := range affectedKeys {
		occ := st.landmarks[k.id].appearances[k.num-1]
		a := replay.agg(k.id, k.num)
		rec.Landmarks = append(rec.Landmarks, LandmarkChange{
			ID:         k.id,
			Occurrence: k.num,
			Before:     Landmark{ID: k.id, X: occ.x, Y: occ.y, Count: occ.count},
			After:      Landmark{ID: k.id, X: a.x, Y: a.y, Count: a.count},
		})
	}

	// ---- 全部校验通过：暂存提交并落盘；落盘失败精确回滚 ----
	oldTraj := append([]Pose(nil), st.trajectory...)
	type occSnapshot struct {
		key  occurrenceKey
		occ  *occurrenceState
		x, y float64
	}
	oldOcc := make([]occSnapshot, 0, len(affectedKeys))
	for _, k := range affectedKeys {
		occ := st.landmarks[k.id].appearances[k.num-1]
		oldOcc = append(oldOcc, occSnapshot{key: k, occ: occ, x: occ.x, y: occ.y})
	}
	oldCorrLen := len(st.corrections)

	st.trajectory = next
	for _, s := range oldOcc {
		a := replay.agg(s.key.id, s.key.num)
		s.occ.x, s.occ.y = a.x, a.y
	}
	st.corrections = append(st.corrections, rec)
	st.corrIndex[req.ID] = canonicalCorrectionHash(req.Anchor, tgt)

	if err := m.saveReplace(); err != nil {
		st.trajectory = oldTraj
		for _, s := range oldOcc {
			s.occ.x, s.occ.y = s.x, s.y
		}
		st.corrections = st.corrections[:oldCorrLen]
		delete(st.corrIndex, req.ID)
		return CorrectionRecord{}, err
	}
	return cloneRecord(rec), nil
}

// Corrections 返回按提交次序排列的成功校正记录副本。记录一经生成永不
// 被后续校正改写；后续校正只是以当时最新状态为起点生成新记录。
func (m *Map) Corrections() ([]CorrectionRecord, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return nil, ErrClosed
	}
	out := make([]CorrectionRecord, 0, len(m.state.corrections))
	for _, rec := range m.state.corrections {
		out = append(out, cloneRecord(rec))
	}
	return out, nil
}

func (st *mapState) correctionRecordByID(id string) *CorrectionRecord {
	for _, rec := range st.corrections {
		if rec.ID == id {
			return rec
		}
	}
	return nil
}

func cloneRecord(rec *CorrectionRecord) CorrectionRecord {
	if rec == nil {
		return CorrectionRecord{}
	}
	cp := *rec
	cp.Poses = append([]PoseChange(nil), rec.Poses...)
	cp.Landmarks = append([]LandmarkChange(nil), rec.Landmarks...)
	return cp
}
