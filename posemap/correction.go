package posemap

import (
	"math"
	"sort"
)

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
// 明冲突帧时间与路标。输入或校正结果含非有限数值同样拒绝。
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
	newHeading := normalizeAngle(tgt.Heading)
	// 相对位姿整体旋转 δ；平移锚点到目标位置。
	dHeading := normalizeAngle(newHeading - oldAnchor.Heading)
	cosD, sinD := math.Cos(dHeading), math.Sin(dHeading)

	// ---- 在副本上推演新位姿，真实状态此时保持不变 ----
	next := append([]Pose(nil), st.trajectory...)
	accumVar := tgt.Variance
	for j := anchorIdx; j <= end; j++ {
		old := st.trajectory[j]
		if j > anchorIdx {
			accumVar += st.sources[j-1].moveVariance
		}
		dx := old.X - oldAnchor.X
		dy := old.Y - oldAnchor.Y
		np := Pose{
			Time:     old.Time,
			X:        tgt.X + cosD*dx - sinD*dy,
			Y:        tgt.Y + sinD*dx + cosD*dy,
			Heading:  normalizeAngle(old.Heading + dHeading),
			Variance: accumVar,
		}
		if !isFinite(np.X) || !isFinite(np.Y) || !isFinite(np.Heading) || !isFinite(np.Variance) {
			return CorrectionRecord{}, &RejectError{Kind: RejectNonFinite, Time: req.Anchor, HasTime: true}
		}
		next[j] = np
	}

	// occKey 标识某一路标的某一次出现。回环校正跨越同一路标的多次出现
	// 时，各次出现分别聚合、分别遵守合并距离限制。
	type occKey struct {
		id  string
		num int
	}

	// occAt 把时间 t 的一次观测归入当时的当前出现：各次出现的时间区间
	// （首次观测时间 .. 失效时间，两端都包含）互不重叠。旧文件的第 1 次
	// 出现缺少逐帧来源（hasFirstSeen 为假），起始端不约束。
	occAt := func(id string, t int64) int {
		lm := st.landmarks[id]
		i := lm.occurrenceIndexAt(t)
		return i + 1 // 无归属时为 0；Open 已保证每条来源都有归属
	}

	// 受影响的路标出现：在受影响帧中被观测到的全部（标识，出现编号）。
	affected := make(map[occKey]struct{})
	for j := anchorIdx; j <= end; j++ {
		t := st.trajectory[j].Time
		for _, ob := range st.sources[j-1].observations {
			affected[occKey{id: ob.ID, num: occAt(ob.ID, t)}] = struct{}{}
		}
	}
	affectedKeys := make([]occKey, 0, len(affected))
	for k := range affected {
		affectedKeys = append(affectedKeys, k)
	}
	sort.Slice(affectedKeys, func(i, j int) bool {
		if affectedKeys[i].id != affectedKeys[j].id {
			return affectedKeys[i].id < affectedKeys[j].id
		}
		return affectedKeys[i].num < affectedKeys[j].num
	})

	// 逐“出现”运行聚合：旧文件第 1 次出现的既有观测作为固定“旧贡献”
	// 起点（位置不随校正改变），其余观测按帧时间与同帧输入次序、以当前/
	// 校正后位姿重放，完整复刻导入时的增量平均与合并距离判定。同一路标的
	// 不同出现互不参与彼此的合并。
	type agg struct {
		count int
		x, y  float64
	}
	aggs := make(map[occKey]*agg, len(affectedKeys))
	for _, k := range affectedKeys {
		occ := st.landmarks[k.id].appearances[k.num-1]
		a := &agg{}
		if occ.legacyCount > 0 {
			a.count = occ.legacyCount
			a.x = occ.legacyMX
			a.y = occ.legacyMY
		}
		aggs[k] = a
	}
	for j := 1; j <= end; j++ {
		src := st.sources[j-1]
		if src == nil {
			continue // 旧文件帧：观测已计入对应出现的 legacy 起点
		}
		pose := next[j]
		t := st.trajectory[j].Time
		for _, ob := range src.observations {
			k := occKey{id: ob.ID, num: occAt(ob.ID, t)}
			a, ok := aggs[k]
			if !ok {
				continue
			}
			mx, my := localToMap(pose.X, pose.Y, pose.Heading, ob.X, ob.Y)
			if a.count > 0 && math.Hypot(mx-a.x, my-a.y) > st.config.MergeDistance {
				return CorrectionRecord{}, &RejectError{
					Kind:          RejectLandmarkConflict,
					Time:          st.trajectory[j].Time,
					HasTime:       true,
					Landmark:      ob.ID,
					HasLandmark:   true,
					Occurrence:    k.num,
					HasOccurrence: true,
				}
			}
			a.x = mergeMean(a.x, mx, a.count)
			a.y = mergeMean(a.y, my, a.count)
			a.count++
		}
	}
	for _, k := range affectedKeys {
		a := aggs[k]
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
		a := aggs[k]
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
		key  occKey
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
		a := aggs[s.key]
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
