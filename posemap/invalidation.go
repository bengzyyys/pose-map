package posemap

import (
	"crypto/sha256"
	"encoding/binary"
	"fmt"
	"sort"
)

// Invalidate 提交一次路标失效操作：req.ID 与 req.Reason 必须非空；
// req.Landmarks 必须是非空、无重复的路标标识集合，且每个路标当前都必须
// 存在一条仍有效的出现记录。满足时，这些当前有效记录同时失效，失效时间
// 取提交时的当前位姿时间；位姿与已接受的观测均不删除，路标只是不再被
// 当作当前有效路标（区域查询立即排除）。
//
// 成功时返回失效时间与每个路标的标识及被失效记录的出现编号（按标识
// 排序）。操作标识/原因为空、列表为空、路标标识为空或重复、路标不存在
// 或当前已失效时，以可区分的 *RejectError 整次拒绝，不留下任何部分
// 变化。
//
// 同一操作标识以相同原因与相同路标集合重复提交时，直接返回首次结果，
// 即使这些路标后来又再次出现也不会再次撤下它们；同一标识对应不同内容
// （原因或集合不同）则明确拒绝。
func (m *Map) Invalidate(req Invalidation) (InvalidationResult, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return InvalidationResult{}, ErrClosed
	}
	st := m.state

	// 已保存的失效操作：相同内容直接返回首次结果；不同内容明确拒绝。
	// 不做任何校验或状态变更，即使其后路标又再次出现。
	setHash := landmarkSetHash(req.Landmarks)
	if rec, ok := st.invalIndex[req.ID]; ok {
		if rec.reason == req.Reason && rec.hash == setHash {
			return cloneInvalidationResult(rec.result), nil
		}
		return InvalidationResult{}, &RejectError{Kind: RejectInvalidationMismatch, Landmark: req.ID, HasLandmark: true}
	}

	if req.ID == "" {
		return InvalidationResult{}, &RejectError{Kind: RejectEmptyID}
	}
	if req.Reason == "" {
		return InvalidationResult{}, &RejectError{Kind: RejectEmptyReason}
	}
	if len(req.Landmarks) == 0 {
		return InvalidationResult{}, &RejectError{Kind: RejectEmptyLandmarkList}
	}

	// 先校验集合本身：标识非空、无重复；再校验每个路标当前存在有效记录。
	seen := make(map[string]struct{}, len(req.Landmarks))
	for _, id := range req.Landmarks {
		if id == "" {
			return InvalidationResult{}, &RejectError{Kind: RejectEmptyID}
		}
		if _, dup := seen[id]; dup {
			return InvalidationResult{}, &RejectError{Kind: RejectDuplicateLandmarkEntry, Landmark: id, HasLandmark: true}
		}
		seen[id] = struct{}{}
	}
	for _, id := range req.Landmarks {
		lm, ok := st.landmarks[id]
		if !ok {
			return InvalidationResult{}, &RejectError{Kind: RejectLandmarkNotFound, Landmark: id, HasLandmark: true}
		}
		if lm.activeOccurrence() == nil {
			return InvalidationResult{}, &RejectError{Kind: RejectLandmarkInactive, Landmark: id, HasLandmark: true}
		}
	}

	// ---- 全部校验通过：对当前有效记录同时标记失效并落盘 ----
	invalidTime := st.trajectory[len(st.trajectory)-1].Time
	ids := append([]string(nil), req.Landmarks...)
	sort.Strings(ids)
	res := InvalidationResult{
		Time:      invalidTime,
		Landmarks: make([]InvalidatedLandmark, 0, len(ids)),
	}
	invalidated := make([]*occurrenceState, 0, len(ids))
	for _, id := range ids {
		lm := st.landmarks[id]
		occ := lm.activeOccurrence()
		invalidated = append(invalidated, occ)
		res.Landmarks = append(res.Landmarks, InvalidatedLandmark{ID: id, Occurrence: len(lm.appearances)})
		occ.active = false
		occ.invalidTime = invalidTime
		occ.invalidReason = req.Reason
		occ.invalidOpID = req.ID
	}
	rec := &invalidationRecord{
		id:     req.ID,
		reason: req.Reason,
		hash:   setHash,
		result: cloneInvalidationResult(res),
	}
	invalLenBefore := len(st.invalRecords)
	st.invalRecords = append(st.invalRecords, rec)
	st.invalIndex[req.ID] = rec

	if err := m.saveReplace(); err != nil {
		for _, occ := range invalidated {
			occ.active = true
			occ.invalidTime = 0
			occ.invalidReason = ""
			occ.invalidOpID = ""
		}
		st.invalRecords = st.invalRecords[:invalLenBefore]
		delete(st.invalIndex, req.ID)
		return InvalidationResult{}, err
	}
	return res, nil
}

// LandmarkAppearances 按出现编号递增返回同一标识的历次出现记录：每次的
// 位置、观测计数、是否仍为当前有效记录、首次观测时间，以及失效时间与
// 原因。仍有效的出现明确不携带失效信息（Appearance.Active 为真）；旧
// 文件中的第 1 次出现缺少逐帧来源，HasFirstSeen 为假，首次观测时间
// 未知。标识从未出现时返回可 errors.Is(ErrLandmarkNotFound) 区分的
// 未找到结果。
func (m *Map) LandmarkAppearances(id string) (LandmarkHistory, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return LandmarkHistory{}, ErrClosed
	}
	lm, ok := m.state.landmarks[id]
	if !ok {
		return LandmarkHistory{}, fmt.Errorf("%w: %s", ErrLandmarkNotFound, id)
	}
	h := LandmarkHistory{ID: id, Appearances: make([]Appearance, 0, len(lm.appearances))}
	for i, occ := range lm.appearances {
		ap := Appearance{
			Number:        i + 1,
			Landmark:      Landmark{ID: id, X: occ.x, Y: occ.y, Count: occ.count},
			Active:        occ.active,
			FirstSeenTime: occ.firstSeenTime,
			HasFirstSeen:  occ.hasFirstSeen,
		}
		if !occ.active {
			ap.InvalidTime = occ.invalidTime
			ap.InvalidReason = occ.invalidReason
			ap.InvalidOpID = occ.invalidOpID
		}
		h.Appearances = append(h.Appearances, ap)
	}
	return h, nil
}

func cloneInvalidationResult(r InvalidationResult) InvalidationResult {
	lms := make([]InvalidatedLandmark, len(r.Landmarks))
	copy(lms, r.Landmarks)
	r.Landmarks = lms
	return r
}

// landmarkSetHash 计算一个路标标识集合的确定性指纹，用于同一失效操作
// 标识的重复内容判定，与输入次序无关（集合语义）。
func landmarkSetHash(ids []string) string {
	sorted := append([]string(nil), ids...)
	sort.Strings(sorted)
	h := sha256.New()
	var lenBuf [4]byte
	binary.BigEndian.PutUint32(lenBuf[:], uint32(len(sorted)))
	h.Write(lenBuf[:])
	for _, s := range sorted {
		binary.BigEndian.PutUint32(lenBuf[:], uint32(len(s)))
		h.Write(lenBuf[:])
		h.Write([]byte(s))
	}
	return fmt.Sprintf("%x", h.Sum(nil))
}
