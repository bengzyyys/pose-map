package posemap

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"hash/crc32"
	"math"
	"os"
	"path/filepath"
)

// 文件布局（均为大端）：
//
//	magic     8 字节  "POSEMAP1"
//	version   uint16
//	length    uint32  payload 字节数
//	payload   length 字节（JSON，见 fileData）
//	crc       uint32  对 crc 之前全部字节的 CRC-32/IEEE
//
// 保存采用“同目录临时文件写完并 fsync 后 rename”的方式，成功导入对
// 调用方可见时文件必为完整的新版本；无法识别或校验失败的文件绝不被
// 覆盖（Create 用 O_EXCL，Open 只读取证）。
// 文件格式在版本 1 内以“追加 JSON 字段”的方式演进：旧文件不含
// sources/corrections 字段（解码后为 nil/空），据此识别缺少逐帧依据的
// 旧地图；二进制版本号保持 1，不支持的版本号仍按既有约定拒绝。
const (
	fileMagic   = "POSEMAP1"
	fileVersion = uint16(1)
	headerLen   = 8 + 2 + 4
)

type fileData struct {
	Version       int             `json:"version"`
	Config        Config          `json:"config"`
	Trajectory    []Pose          `json:"trajectory"`
	Landmarks     []landmarkJSON  `json:"landmarks"`
	Segments      []segmentJSON   `json:"segments"`
	Sources       []*frameSrcJSON `json:"sources,omitempty"`       // 新格式：逐帧依据，旧帧为 null；旧文件整列缺省
	Corrections   []corrRecJSON   `json:"corrections,omitempty"`   // 新格式：按提交次序的校正记录
	Invalidations []invalJSON     `json:"invalidations,omitempty"` // 新格式：按提交次序的失效操作
}

type landmarkJSON struct {
	ID          string           `json:"id"`
	Occurrences []occurrenceJSON `json:"occurrences,omitempty"` // 新格式：历次出现，编号递增
	// 以下平铺字段仅用于读取不携带 occurrences 的旧文件。
	X           float64 `json:"x,omitempty"`
	Y           float64 `json:"y,omitempty"`
	Count       int     `json:"count,omitempty"`
	LegacyCount int     `json:"legacy_count,omitempty"`
	LegacyMX    float64 `json:"legacy_mx,omitempty"`
	LegacyMY    float64 `json:"legacy_my,omitempty"`
}

type occurrenceJSON struct {
	Number        int     `json:"number"` // 出现编号，从 1 开始连续
	X             float64 `json:"x"`
	Y             float64 `json:"y"`
	Count         int     `json:"count"`
	LegacyCount   int     `json:"legacy_count,omitempty"`
	LegacyMX      float64 `json:"legacy_mx,omitempty"`
	LegacyMY      float64 `json:"legacy_my,omitempty"`
	FirstSeenTime int64   `json:"first_seen_time,omitempty"`
	HasFirstSeen  bool    `json:"has_first_seen"`
	Active        bool    `json:"active"`
	InvalidTime   int64   `json:"invalid_time,omitempty"`
	InvalidReason string  `json:"invalid_reason,omitempty"`
	InvalidOpID   string  `json:"invalid_op_id,omitempty"`
}

type invalJSON struct {
	ID        string        `json:"id"`
	Reason    string        `json:"reason"`
	Hash      string        `json:"hash"`
	Time      int64         `json:"time"`
	Landmarks []invalLMJSON `json:"landmarks"`
}

type invalLMJSON struct {
	ID         string `json:"id"`
	Occurrence int    `json:"occurrence"`
}

type segmentJSON struct {
	ID     string       `json:"id"`
	Hash   string       `json:"hash"`
	Result ImportResult `json:"result"`
}

type frameSrcJSON struct {
	MoveVariance float64       `json:"move_variance"`
	Observations []Observation `json:"observations"`
}

type poseChangeJSON struct {
	Before Pose `json:"before"`
	After  Pose `json:"after"`
}

type landmarkChangeJSON struct {
	ID         string   `json:"id"`
	Occurrence int      `json:"occurrence"` // 受影响的出现编号（1 起）；旧校正记录缺省为 1
	Before     Landmark `json:"before"`
	After      Landmark `json:"after"`
}

type corrRecJSON struct {
	ID        string               `json:"id"`
	Anchor    int64                `json:"anchor"`
	Target    CorrectionTarget     `json:"target"`
	EndTime   int64                `json:"end_time"`
	Poses     []poseChangeJSON     `json:"poses"`
	Landmarks []landmarkChangeJSON `json:"landmarks"`
}

// encodeFile 把内存状态编码为完整文件字节。
func encodeFile(st *mapState) ([]byte, error) {
	fd := fileData{
		Version:       int(fileVersion),
		Config:        st.config,
		Trajectory:    st.trajectory,
		Landmarks:     make([]landmarkJSON, 0, len(st.landmarks)),
		Segments:      make([]segmentJSON, 0, len(st.segments)),
		Sources:       make([]*frameSrcJSON, len(st.sources)),
		Corrections:   make([]corrRecJSON, 0, len(st.corrections)),
		Invalidations: make([]invalJSON, 0, len(st.invalRecords)),
	}
	for id, lm := range st.landmarks {
		lj := landmarkJSON{ID: id, Occurrences: make([]occurrenceJSON, 0, len(lm.appearances))}
		for i, occ := range lm.appearances {
			oj := occurrenceJSON{
				Number:        i + 1,
				X:             occ.x,
				Y:             occ.y,
				Count:         occ.count,
				LegacyCount:   occ.legacyCount,
				LegacyMX:      occ.legacyMX,
				LegacyMY:      occ.legacyMY,
				FirstSeenTime: occ.firstSeenTime,
				HasFirstSeen:  occ.hasFirstSeen,
				Active:        occ.active,
			}
			if !occ.active {
				oj.InvalidTime = occ.invalidTime
				oj.InvalidReason = occ.invalidReason
				oj.InvalidOpID = occ.invalidOpID
			}
			lj.Occurrences = append(lj.Occurrences, oj)
		}
		fd.Landmarks = append(fd.Landmarks, lj)
	}
	for id, rec := range st.segments {
		fd.Segments = append(fd.Segments, segmentJSON{ID: id, Hash: rec.hash, Result: rec.result})
	}
	for k, src := range st.sources {
		if src == nil {
			continue // 旧文件帧在新文件中保留为 null，表示仍无逐帧依据
		}
		obs := append([]Observation(nil), src.observations...)
		fd.Sources[k] = &frameSrcJSON{MoveVariance: src.moveVariance, Observations: obs}
	}
	for _, rec := range st.corrections {
		cj := corrRecJSON{
			ID:        rec.ID,
			Anchor:    rec.Anchor,
			Target:    rec.Target,
			EndTime:   rec.EndTime,
			Poses:     make([]poseChangeJSON, len(rec.Poses)),
			Landmarks: make([]landmarkChangeJSON, len(rec.Landmarks)),
		}
		for i, pc := range rec.Poses {
			cj.Poses[i] = poseChangeJSON{Before: pc.Before, After: pc.After}
		}
		for i, lc := range rec.Landmarks {
			cj.Landmarks[i] = landmarkChangeJSON{ID: lc.ID, Occurrence: lc.Occurrence, Before: lc.Before, After: lc.After}
		}
		fd.Corrections = append(fd.Corrections, cj)
	}
	for _, rec := range st.invalRecords {
		ij := invalJSON{
			ID:        rec.id,
			Reason:    rec.reason,
			Hash:      rec.hash,
			Time:      rec.result.Time,
			Landmarks: make([]invalLMJSON, len(rec.result.Landmarks)),
		}
		for i, l := range rec.result.Landmarks {
			ij.Landmarks[i] = invalLMJSON{ID: l.ID, Occurrence: l.Occurrence}
		}
		fd.Invalidations = append(fd.Invalidations, ij)
	}
	payload, err := json.Marshal(&fd)
	if err != nil {
		return nil, fmt.Errorf("posemap: encode state: %w", err)
	}

	out := make([]byte, headerLen+len(payload)+4)
	copy(out[0:8], fileMagic)
	binary.BigEndian.PutUint16(out[8:10], fileVersion)
	binary.BigEndian.PutUint32(out[10:14], uint32(len(payload)))
	copy(out[headerLen:], payload)
	sum := crc32.ChecksumIEEE(out[:headerLen+len(payload)])
	binary.BigEndian.PutUint32(out[headerLen+len(payload):], sum)
	return out, nil
}

// loadFile 只读取证地解析地图文件。
func loadFile(path string) (*mapState, error) {
	raw, err := os.ReadFile(path)
	if err != nil {
		return nil, err // 含 fs.ErrNotExist，交由调用方/上层用 errors.Is 判断
	}
	if len(raw) < headerLen+4 || string(raw[:8]) != fileMagic {
		return nil, fmt.Errorf("%w: unrecognized header", ErrCorrupt)
	}
	version := binary.BigEndian.Uint16(raw[8:10])
	if version != fileVersion {
		return nil, fmt.Errorf("%w: version %d", ErrUnsupportedVersion, version)
	}
	length := binary.BigEndian.Uint32(raw[10:14])
	if int(length) != len(raw)-headerLen-4 {
		return nil, fmt.Errorf("%w: truncated or length mismatch", ErrCorrupt)
	}
	want := binary.BigEndian.Uint32(raw[len(raw)-4:])
	if crc32.ChecksumIEEE(raw[:len(raw)-4]) != want {
		return nil, fmt.Errorf("%w: checksum mismatch", ErrCorrupt)
	}

	var fd fileData
	if err := json.Unmarshal(raw[headerLen:headerLen+int(length)], &fd); err != nil {
		return nil, fmt.Errorf("%w: %v", ErrCorrupt, err)
	}
	if err := validateLoaded(&fd); err != nil {
		return nil, err
	}

	st := &mapState{
		config:       fd.Config,
		trajectory:   fd.Trajectory,
		landmarks:    make(map[string]*landmarkState, len(fd.Landmarks)),
		segments:     make(map[string]*segmentRecord, len(fd.Segments)),
		sources:      make([]*frameSource, len(fd.Trajectory)-1),
		corrections:  make([]*CorrectionRecord, 0, len(fd.Corrections)),
		corrIndex:    make(map[string]string, len(fd.Corrections)),
		invalRecords: make([]*invalidationRecord, 0, len(fd.Invalidations)),
		invalIndex:   make(map[string]*invalidationRecord, len(fd.Invalidations)),
	}
	// 旧文件不携带逐帧依据：有帧而 sources 缺省即旧地图。无帧时二者
	// 无法区分也无需区分（尚无范围可校正）。
	oldFormat := len(fd.Sources) == 0 && len(fd.Trajectory) > 1
	for _, lmj := range fd.Landmarks {
		lm := &landmarkState{}
		if len(lmj.Occurrences) > 0 {
			// 新格式：历次出现按编号递增保存。
			for _, oj := range lmj.Occurrences {
				lm.appearances = append(lm.appearances, &occurrenceState{
					x:             oj.X,
					y:             oj.Y,
					count:         oj.Count,
					legacyCount:   oj.LegacyCount,
					legacyMX:      oj.LegacyMX,
					legacyMY:      oj.LegacyMY,
					firstSeenTime: oj.FirstSeenTime,
					hasFirstSeen:  oj.HasFirstSeen,
					active:        oj.Active,
					invalidTime:   oj.InvalidTime,
					invalidReason: oj.InvalidReason,
					invalidOpID:   oj.InvalidOpID,
				})
			}
		} else {
			// 兼容旧布局：一个平铺路标即第 1 次有效出现。
			occ := &occurrenceState{
				x:      lmj.X,
				y:      lmj.Y,
				count:  lmj.Count,
				active: true,
			}
			if oldFormat {
				// 旧文件中该路标的全部既有观测都是固定旧贡献，位置取
				// 文件中的聚合值；缺少逐帧来源，首次观测时间未知。
				occ.legacyCount = lmj.Count
				occ.legacyMX, occ.legacyMY = lmj.X, lmj.Y
				occ.hasFirstSeen = false
			} else {
				occ.legacyCount = lmj.LegacyCount
				occ.legacyMX, occ.legacyMY = lmj.LegacyMX, lmj.LegacyMY
				// 含固定旧贡献时最早观测缺少来源，首次观测时间未知；
				// 否则从逐帧依据中恢复该出现第一次被观测到的帧时间。
				occ.hasFirstSeen = lmj.LegacyCount == 0
				if occ.hasFirstSeen {
					for k, src := range fd.Sources {
						if src == nil {
							continue
						}
						found := false
						for _, ob := range src.Observations {
							if ob.ID == lmj.ID {
								found = true
								break
							}
						}
						if found {
							occ.firstSeenTime = fd.Trajectory[k+1].Time
							break
						}
					}
				}
			}
			lm.appearances = append(lm.appearances, occ)
		}
		st.landmarks[lmj.ID] = lm
	}
	for _, sg := range fd.Segments {
		ids := sg.Result.LandmarkIDs
		if ids == nil {
			ids = []string{}
		}
		st.segments[sg.ID] = &segmentRecord{
			hash:   sg.Hash,
			result: ImportResult{EndPose: sg.Result.EndPose, LandmarkIDs: ids},
		}
	}
	if !oldFormat {
		for k, src := range fd.Sources {
			if src == nil {
				continue // 来自旧文件的帧：保持依据缺失
			}
			obs := append([]Observation(nil), src.Observations...)
			st.sources[k] = &frameSource{moveVariance: src.MoveVariance, observations: obs}
		}
	}
	for _, cj := range fd.Corrections {
		rec := &CorrectionRecord{
			ID:        cj.ID,
			Anchor:    cj.Anchor,
			Target:    cj.Target,
			EndTime:   cj.EndTime,
			Poses:     make([]PoseChange, len(cj.Poses)),
			Landmarks: make([]LandmarkChange, len(cj.Landmarks)),
		}
		for i, pc := range cj.Poses {
			rec.Poses[i] = PoseChange{Before: pc.Before, After: pc.After}
		}
		for i, lc := range cj.Landmarks {
			occNum := lc.Occurrence
			if occNum == 0 {
				occNum = 1 // 更早版本写出的校正记录不带编号，只有第 1 次出现
			}
			rec.Landmarks[i] = LandmarkChange{ID: lc.ID, Occurrence: occNum, Before: lc.Before, After: lc.After}
		}
		st.corrections = append(st.corrections, rec)
		st.corrIndex[cj.ID] = canonicalCorrectionHash(cj.Anchor, cj.Target)
	}
	for _, ij := range fd.Invalidations {
		lms := make([]InvalidatedLandmark, len(ij.Landmarks))
		for i, l := range ij.Landmarks {
			lms[i] = InvalidatedLandmark{ID: l.ID, Occurrence: l.Occurrence}
		}
		rec := &invalidationRecord{
			id:     ij.ID,
			reason: ij.Reason,
			hash:   ij.Hash,
			result: InvalidationResult{Time: ij.Time, Landmarks: lms},
		}
		st.invalRecords = append(st.invalRecords, rec)
		st.invalIndex[ij.ID] = rec
	}
	return st, nil
}

// validateLoaded 对解码后的状态做基本不变量检查，违背即视为损坏。
func validateLoaded(fd *fileData) error {
	switch {
	case fd.Version != int(fileVersion):
		return fmt.Errorf("%w: version %d in payload", ErrUnsupportedVersion, fd.Version)
	case len(fd.Trajectory) == 0:
		return fmt.Errorf("%w: empty trajectory", ErrCorrupt)
	}
	if err := validateConfig(fd.Config); err != nil {
		// 配置不合法不可能由本版本写出。
		return fmt.Errorf("%w: %v", ErrCorrupt, err)
	}
	// 保存的整条轨迹必须满足与导入时相同的时间规则：从初始位姿起
	// 时间严格递增，且相邻位姿的真实间隔不超过 MaxInterval（恰好
	// 等于上限合法）。任何一处相等、倒退或超限都使按时间查询失去
	// 可靠的先后依据，整份文件按损坏拒绝。间隔按真实差值判断：前
	// 一时间接近 math.MinInt64 而后一时间为非负时，有符号减法会
	// 溢出回绕成负数，把超限的巨大缺口放过去；此处已保证递增，真
	// 实差值落在 [1, 2^64-1]，用无符号减法取精确的数学差值再与上
	// 限比较（MaxInterval 已校验为正）。
	for i := 1; i < len(fd.Trajectory); i++ {
		prev, cur := fd.Trajectory[i-1].Time, fd.Trajectory[i].Time
		if cur <= prev {
			return fmt.Errorf("%w: trajectory time at pose %d is not strictly increasing", ErrCorrupt, i)
		}
		if uint64(cur)-uint64(prev) > uint64(fd.Config.MaxInterval) {
			return fmt.Errorf("%w: trajectory interval between poses %d and %d exceeds max interval", ErrCorrupt, i-1, i)
		}
	}
	// 新格式的逐帧依据必须与轨迹（除初始位姿外）一一对应；旧文件缺省
	// 该字段（解码为 nil），数量为 0。
	if len(fd.Sources) != 0 && len(fd.Sources) != len(fd.Trajectory)-1 {
		return fmt.Errorf("%w: frame sources length %d for %d frames", ErrCorrupt, len(fd.Sources), len(fd.Trajectory)-1)
	}
	seenIDs := make(map[string]struct{}, len(fd.Landmarks))
	for _, lm := range fd.Landmarks {
		if lm.ID == "" {
			return fmt.Errorf("%w: invalid landmark record", ErrCorrupt)
		}
		if _, dup := seenIDs[lm.ID]; dup {
			return fmt.Errorf("%w: duplicate landmark %s", ErrCorrupt, lm.ID)
		}
		seenIDs[lm.ID] = struct{}{}
		if len(lm.Occurrences) == 0 {
			// 兼容旧布局：平铺字段表示第 1 次出现。
			if lm.Count < 1 || !isFinite(lm.X) || !isFinite(lm.Y) {
				return fmt.Errorf("%w: invalid landmark record", ErrCorrupt)
			}
			if lm.LegacyCount < 0 || lm.LegacyCount > lm.Count {
				return fmt.Errorf("%w: invalid legacy count for landmark %s", ErrCorrupt, lm.ID)
			}
			if lm.LegacyCount > 0 && (!isFinite(lm.LegacyMX) || !isFinite(lm.LegacyMY)) {
				return fmt.Errorf("%w: invalid legacy mean for landmark %s", ErrCorrupt, lm.ID)
			}
			continue
		}
		activeCount := 0
		for i, oj := range lm.Occurrences {
			if oj.Number != i+1 || oj.Count < 1 || !isFinite(oj.X) || !isFinite(oj.Y) {
				return fmt.Errorf("%w: invalid occurrence for landmark %s", ErrCorrupt, lm.ID)
			}
			if oj.LegacyCount < 0 || oj.LegacyCount > oj.Count {
				return fmt.Errorf("%w: invalid legacy count for landmark %s occurrence %d", ErrCorrupt, lm.ID, oj.Number)
			}
			if oj.LegacyCount > 0 && (!isFinite(oj.LegacyMX) || !isFinite(oj.LegacyMY)) {
				return fmt.Errorf("%w: invalid legacy mean for landmark %s occurrence %d", ErrCorrupt, lm.ID, oj.Number)
			}
			if oj.Active {
				activeCount++
				if i != len(lm.Occurrences)-1 {
					return fmt.Errorf("%w: active occurrence %d of landmark %s is not the latest", ErrCorrupt, oj.Number, lm.ID)
				}
			} else if oj.InvalidReason == "" {
				return fmt.Errorf("%w: invalidated occurrence %d of landmark %s lacks reason", ErrCorrupt, oj.Number, lm.ID)
			}
			// 出现编号即先后次序，各次出现接纳观测的时间范围必须与之
			// 相容，否则同一观测可能同时属于两次出现（归属校验只确认能
			// 归入某一次，不排除同时归入两次）。已失效且首次观测时间
			// 已知的出现，首次观测不能晚于失效；下一次出现的首次观测
			// 必须严格晚于上一次的失效时间——首次观测与失效两个端点都
			// 接纳所在时刻的观测，相等即重叠。第 2 次及以后的出现必定
			// 在上一次失效之后由本包创建，首次观测时间必须已知；只有
			// 旧文件的第 1 次出现允许未知（无下界，不把零当作起点）。
			// 这些检查针对保存的出现记录本身，与重叠范围内当前是否有
			// 观测无关。
			if !oj.Active && oj.HasFirstSeen && oj.FirstSeenTime > oj.InvalidTime {
				return fmt.Errorf("%w: occurrence %d of landmark %s first seen at %d after invalidation at %d", ErrCorrupt, oj.Number, lm.ID, oj.FirstSeenTime, oj.InvalidTime)
			}
			if i > 0 {
				if !oj.HasFirstSeen {
					return fmt.Errorf("%w: occurrence %d of landmark %s lacks first seen time", ErrCorrupt, oj.Number, lm.ID)
				}
				// 上一次出现必已失效（仍有效且非最后一次已在上面拒绝）。
				if prev := lm.Occurrences[i-1]; oj.FirstSeenTime <= prev.InvalidTime {
					return fmt.Errorf("%w: occurrence %d of landmark %s starts at %d before previous invalidation at %d", ErrCorrupt, oj.Number, lm.ID, oj.FirstSeenTime, prev.InvalidTime)
				}
			}
		}
		if activeCount > 1 {
			return fmt.Errorf("%w: multiple active occurrences for landmark %s", ErrCorrupt, lm.ID)
		}
	}
	// 校验每条带逐帧依据的观测来源：观测路标必须存在，且所在帧时间必须
	// 落入该路标某一次出现的时间边界（共享规则见 occurrence.go）。旧出现
	// 已失效不代表其历史观测无效；同一标识两次出现之间（如 200 失效、
	// 300 再现时的 250 帧）的观测没有归属，按损坏拒绝。null 帧来自缺少
	// 逐帧依据的旧地图，其观测不在这里（也无法）核验；旧版平铺路标视为
	// 一次无边界的有效出现，因此旧地图后来追加的来源帧仍按此规则检查。
	if len(fd.Sources) != 0 {
		bounds := make(map[string][]occurrenceWindow, len(fd.Landmarks))
		for _, lm := range fd.Landmarks {
			bounds[lm.ID] = occurrenceWindowsOf(lm)
		}
		// 归属校验的同时按（路标，出现编号）统计来源观测条数：每条观测
		// 各算一次，同帧重复观测同一标识不去重，空观测帧不计。null 帧
		// 缺少逐帧依据，其贡献已固定在出现的旧观测计数中，不在此统计，
		// 也不能按帧数猜测。
		tallies := make(map[string]map[int]int, len(fd.Landmarks))
		for k, src := range fd.Sources {
			if src == nil {
				continue // 旧地图帧：无逐帧依据，不要求补齐归属
			}
			t := fd.Trajectory[k+1].Time
			for _, ob := range src.Observations {
				wins, ok := bounds[ob.ID]
				if !ok {
					return fmt.Errorf("%w: observation of unknown landmark %s at frame time %d", ErrCorrupt, ob.ID, t)
				}
				num := occurrenceNumberAt(wins, t)
				if num == 0 {
					return fmt.Errorf("%w: observation of landmark %s at frame time %d belongs to no occurrence", ErrCorrupt, ob.ID, t)
				}
				c := tallies[ob.ID]
				if c == nil {
					c = make(map[int]int, 1)
					tallies[ob.ID] = c
				}
				c[num]++
			}
		}
		// 每次出现记录的观测次数必须等于归属该次出现的来源观测条数加上
		// 该次出现已保存的旧观测贡献：偏多或偏少都是文件损坏。各次出现
		// 分别核对，不能用另一次出现的观测补足；已失效的旧出现同样核对，
		// 不因它不再参与区域查询而跳过。回环校正只改位置不改已接受观测
		// 次数，正常校正后保存的文件仍满足此不变量。
		for _, lm := range fd.Landmarks {
			if len(lm.Occurrences) == 0 {
				// 旧版平铺路标即第 1 次出现。
				if got := tallies[lm.ID][1] + lm.LegacyCount; got != lm.Count {
					return fmt.Errorf("%w: landmark %s records %d observations but sources and legacy contributions account for %d", ErrCorrupt, lm.ID, lm.Count, got)
				}
				continue
			}
			for _, oj := range lm.Occurrences {
				if got := tallies[lm.ID][oj.Number] + oj.LegacyCount; got != oj.Count {
					return fmt.Errorf("%w: landmark %s occurrence %d records %d observations but sources and legacy contributions account for %d", ErrCorrupt, lm.ID, oj.Number, oj.Count, got)
				}
			}
		}

		// 位置核对：次数相符只说明观测“条数”对得上，还必须核对保存的路标
		// 位置确实是这些观测依据当前保存位姿重放后的等权平均。每条来源观测
		// 按所在帧当前保存的位姿转换到地图坐标，按帧时间升序、同帧按输入
		// 次序，归入各自出现分别重放；同帧对同一标识的多条观测各自参与，
		// 空观测帧不增加贡献。转换、接纳、增量平均与计数全部复用导入/校正
		// 的同一规则（见 aggregate.go），旧文件固定旧贡献（LegacyCount 次
		// 平均）作为聚合起点——不能当成零，也不能用缺少依据的帧数代替。
		// 回环校正后保存的文件按当前轨迹重放即得到当前位置，不要求等于校正
		// 记录里的历史快照；各次出现分别核对，已失效的出现同样核对。合并
		// 距离只是导入/校正时的接纳门槛，不是保存位置偏差的容许范围：重放
		// 中出现非有限结果或距离冲突，或最终平均位置与保存位置不完全一致，
		// 都按损坏拒绝。纯旧文件没有 sources、null 旧帧没有依据，均不重放。
		type posKey struct {
			id  string
			num int
		}
		posAggs := make(map[posKey]*observationAgg, len(fd.Landmarks))
		seedPosAgg := func(id string, num, legacyCount int, lx, ly float64) {
			a := &observationAgg{}
			if legacyCount > 0 {
				a.seed(legacyCount, lx, ly)
			}
			posAggs[posKey{id: id, num: num}] = a
		}
		for _, lm := range fd.Landmarks {
			if len(lm.Occurrences) == 0 {
				// 旧版平铺路标即第 1 次出现。
				seedPosAgg(lm.ID, 1, lm.LegacyCount, lm.LegacyMX, lm.LegacyMY)
				continue
			}
			for _, oj := range lm.Occurrences {
				seedPosAgg(lm.ID, oj.Number, oj.LegacyCount, oj.LegacyMX, oj.LegacyMY)
			}
		}
		for k, src := range fd.Sources {
			if src == nil {
				continue // 旧地图帧：无逐帧依据，其贡献固定在旧贡献起点中
			}
			pose := fd.Trajectory[k+1]
			for _, ob := range src.Observations {
				// 归属（存在性与时间边界）已在上面校验，num 必非 0。
				num := occurrenceNumberAt(bounds[ob.ID], pose.Time)
				a := posAggs[posKey{id: ob.ID, num: num}]
				switch a.admit(pose, ob, fd.Config.MergeDistance) {
				case obsNonFinite:
					return fmt.Errorf("%w: replayed observation of landmark %s occurrence %d at frame time %d is non-finite", ErrCorrupt, ob.ID, num, pose.Time)
				case obsConflict:
					return fmt.Errorf("%w: replayed observation of landmark %s occurrence %d at frame time %d violates merge distance", ErrCorrupt, ob.ID, num, pose.Time)
				}
			}
		}
		checkPosition := func(id string, num int, x, y float64) error {
			a := posAggs[posKey{id: id, num: num}]
			// 次数核对已保证 a.count 与记录次数一致（含旧贡献起点），这里只
			// 核对位置：保存位置必须与按依据重放出的等权平均完全一致。
			if a.x != x || a.y != y {
				return fmt.Errorf("%w: landmark %s occurrence %d is saved at (%v,%v) but its observations replay to (%v,%v)", ErrCorrupt, id, num, x, y, a.x, a.y)
			}
			return nil
		}
		for _, lm := range fd.Landmarks {
			if len(lm.Occurrences) == 0 {
				if err := checkPosition(lm.ID, 1, lm.X, lm.Y); err != nil {
					return err
				}
				continue
			}
			for _, oj := range lm.Occurrences {
				if err := checkPosition(lm.ID, oj.Number, oj.X, oj.Y); err != nil {
					return err
				}
			}
		}
	}
	segIDs := make(map[string]struct{}, len(fd.Segments))
	for _, sg := range fd.Segments {
		if sg.ID == "" || sg.Hash == "" {
			return fmt.Errorf("%w: invalid segment record", ErrCorrupt)
		}
		if _, dup := segIDs[sg.ID]; dup {
			return fmt.Errorf("%w: duplicate segment %s", ErrCorrupt, sg.ID)
		}
		segIDs[sg.ID] = struct{}{}
	}
	corrIDs := make(map[string]struct{}, len(fd.Corrections))
	for _, cr := range fd.Corrections {
		if cr.ID == "" || cr.Anchor > cr.EndTime {
			return fmt.Errorf("%w: invalid correction record", ErrCorrupt)
		}
		// 锚点与结束时间必须准确命中已导入帧：trajectory[0] 是初始位姿，
		// 不是可校正的帧，也不能用相邻帧的时间代替。轨迹时间已在前面
		// 校验为严格递增，命中即唯一。
		anchorIdx, endIdx := -1, -1
		for j := 1; j < len(fd.Trajectory); j++ {
			if fd.Trajectory[j].Time == cr.Anchor {
				anchorIdx = j
			}
			if fd.Trajectory[j].Time == cr.EndTime {
				endIdx = j
			}
		}
		if anchorIdx < 0 || endIdx < 0 {
			return fmt.Errorf("%w: correction %s range does not hit imported frames", ErrCorrupt, cr.ID)
		}
		// 记录必须恰好覆盖 [anchor, end] 内的每一帧：位姿条目数量等于
		// 范围内帧数，按轨迹时间次序一一对应，且同一份前后位姿的时间
		// 相同并就是对应帧的时间。缺失、重复、错序、额外加入范围外的
		// 帧，或把前后时间写成两个不同帧，都使记录不能完整描述声明的
		// 范围，按损坏拒绝。这里只核对覆盖的帧及时间：快照中的位置、
		// 朝向和方差是提交时的历史值，不要求等于当前轨迹（后来对相同
		// 范围再校正、或之后导入新帧，都不影响既有记录的合法性）。
		if len(cr.Poses) != endIdx-anchorIdx+1 {
			return fmt.Errorf("%w: correction %s does not cover every frame in its range", ErrCorrupt, cr.ID)
		}
		for i, pc := range cr.Poses {
			ft := fd.Trajectory[anchorIdx+i].Time
			if pc.Before.Time != ft || pc.After.Time != ft {
				return fmt.Errorf("%w: correction %s pose entry %d does not match frame time %d", ErrCorrupt, cr.ID, i, ft)
			}
			if !finitePose(pc.Before) || !finitePose(pc.After) {
				return fmt.Errorf("%w: correction %s has non-finite pose", ErrCorrupt, cr.ID)
			}
		}
		for _, lc := range cr.Landmarks {
			if lc.ID == "" {
				return fmt.Errorf("%w: correction %s has empty landmark id", ErrCorrupt, cr.ID)
			}
			if lc.Occurrence < 0 || !isFinite(lc.Before.X) || !isFinite(lc.Before.Y) ||
				!isFinite(lc.After.X) || !isFinite(lc.After.Y) ||
				lc.Before.Count < 1 || lc.After.Count < 1 {
				return fmt.Errorf("%w: correction %s has invalid landmark change", ErrCorrupt, cr.ID)
			}
		}
		if _, dup := corrIDs[cr.ID]; dup {
			return fmt.Errorf("%w: duplicate correction %s", ErrCorrupt, cr.ID)
		}
		corrIDs[cr.ID] = struct{}{}
	}
	// 路标记录按标识索引，供失效记录与出现历史的交叉核对使用。
	lmByID := make(map[string]*landmarkJSON, len(fd.Landmarks))
	for i := range fd.Landmarks {
		lmByID[fd.Landmarks[i].ID] = &fd.Landmarks[i]
	}
	invalIDs := make(map[string]struct{}, len(fd.Invalidations))
	// invalTriples 记录每条失效操作实际引用的（操作标识，路标，出现编号）
	// 三元组，供反向核对：每个已失效出现都必须是某条操作记录里的实际条目。
	type invalTriple struct {
		op  string
		lm  string
		num int
	}
	invalTriples := make(map[invalTriple]struct{}, len(fd.Invalidations))
	for _, iv := range fd.Invalidations {
		if iv.ID == "" || iv.Reason == "" || iv.Hash == "" || len(iv.Landmarks) == 0 {
			return fmt.Errorf("%w: invalid invalidation record", ErrCorrupt)
		}
		if _, dup := invalIDs[iv.ID]; dup {
			return fmt.Errorf("%w: duplicate invalidation %s", ErrCorrupt, iv.ID)
		}
		invalIDs[iv.ID] = struct{}{}
		lmSeen := make(map[string]struct{}, len(iv.Landmarks))
		for _, l := range iv.Landmarks {
			if l.ID == "" || l.Occurrence < 1 {
				return fmt.Errorf("%w: invalid invalidation landmark entry", ErrCorrupt)
			}
			if _, dup := lmSeen[l.ID]; dup {
				return fmt.Errorf("%w: invalidation %s lists %s twice", ErrCorrupt, iv.ID, l.ID)
			}
			lmSeen[l.ID] = struct{}{}
		}
		// 交叉核对：成功失效结果里的每个（路标标识，出现编号）必须准确
		// 指向该路标历史中同编号的那次出现，不能改指同一路标的其他出现，
		// 也不能因为路标后来再次出现就把旧结果指向最新记录。被指出现必须
		// 已经失效，且其上记录的失效操作标识、失效时间、失效原因都与这条
		// 操作记录一致——时间与原因各自再合理也不够，实际由另一条操作撤下
		// 时不能仅凭相同时间/原因认可。旧版平铺路标视为第 1 次有效出现，
		// 不可能被失效记录引用（引用即损坏）。一条操作同时撤下多个路标时，
		// 任意一个条目不满足对应关系都拒绝打开整个文件。
		for _, l := range iv.Landmarks {
			lm := lmByID[l.ID]
			if lm == nil {
				return fmt.Errorf("%w: invalidation %s result points to unknown landmark %s", ErrCorrupt, iv.ID, l.ID)
			}
			if len(lm.Occurrences) == 0 {
				return fmt.Errorf("%w: invalidation %s result points to flat legacy landmark %s which has no invalidated occurrence", ErrCorrupt, iv.ID, l.ID)
			}
			if l.Occurrence > len(lm.Occurrences) {
				return fmt.Errorf("%w: invalidation %s result points to landmark %s occurrence %d which does not exist", ErrCorrupt, iv.ID, l.ID, l.Occurrence)
			}
			// 出现编号连续性已在上方路标校验中强制，该下标处编号即 l.Occurrence。
			oj := lm.Occurrences[l.Occurrence-1]
			if oj.Active {
				return fmt.Errorf("%w: invalidation %s result points to landmark %s occurrence %d which is still active", ErrCorrupt, iv.ID, l.ID, l.Occurrence)
			}
			if oj.InvalidOpID != iv.ID || oj.InvalidTime != iv.Time || oj.InvalidReason != iv.Reason {
				return fmt.Errorf("%w: invalidation %s result for landmark %s occurrence %d does not match its saved invalidation (op %q, time %d, reason %q)", ErrCorrupt, iv.ID, l.ID, l.Occurrence, oj.InvalidOpID, oj.InvalidTime, oj.InvalidReason)
			}
			invalTriples[invalTriple{op: iv.ID, lm: l.ID, num: l.Occurrence}] = struct{}{}
		}
	}
	// 反向核对：每个已失效出现都必须能在失效操作序列中找到引用同一（操作，
	// 路标，出现编号）三元组的条目。正向核对已保证被引用出现的失效时间与
	// 原因和引用方一致；此处堵住另一方向的矛盾：出现凭空携带失效信息、
	// 声称撤下它的操作记录实际指向另一次出现，或两次出现声称被同一操作
	// 撤下（一次操作对同一路标只会撤下当时唯一的有效出现）。旧地图没有
	// 失效操作时，平铺路标视为有效、不携带失效信息，不会触发；旧地图后来
	// 产生的失效操作同样在上述核对之列，不要求补齐原本缺失的逐帧观测。
	for _, lm := range fd.Landmarks {
		for _, oj := range lm.Occurrences {
			if oj.Active {
				continue
			}
			if _, ok := invalTriples[invalTriple{op: oj.InvalidOpID, lm: lm.ID, num: oj.Number}]; !ok {
				return fmt.Errorf("%w: landmark %s occurrence %d is marked invalidated by operation %q which does not record that invalidation", ErrCorrupt, lm.ID, oj.Number, oj.InvalidOpID)
			}
		}
	}
	return nil
}

func finitePose(p Pose) bool {
	return isFinite(p.X) && isFinite(p.Y) && isFinite(p.Heading) && isFinite(p.Variance)
}

// canonicalCorrectionHash 计算一次校正请求（锚点时间 + 目标位姿）的
// 确定性指纹，用于同标识重复提交判定。直接按 IEEE-754 位编码。
func canonicalCorrectionHash(anchor int64, tgt CorrectionTarget) string {
	h := sha256.New()
	var b [8]byte
	binary.BigEndian.PutUint64(b[:], uint64(anchor))
	h.Write(b[:])
	for _, v := range []float64{tgt.X, tgt.Y, tgt.Heading, tgt.Variance} {
		binary.BigEndian.PutUint64(b[:], math.Float64bits(v))
		h.Write(b[:])
	}
	return fmt.Sprintf("%x", h.Sum(nil))
}

// saveExclusive 创建新文件，目标已存在即失败（O_EXCL）。
func (m *Map) saveExclusive() error {
	data, err := encodeFile(m.state)
	if err != nil {
		return err
	}
	f, err := os.OpenFile(m.path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0o644)
	if err != nil {
		return err
	}
	return writeAndClose(f, data, m.path)
}

// saveReplace 用同目录临时文件 + rename 原子替换现有文件。
func (m *Map) saveReplace() (err error) {
	data, err := encodeFile(m.state)
	if err != nil {
		return err
	}
	dir, base := filepath.Split(m.path)
	if dir == "" {
		dir = "."
	}
	f, err := os.CreateTemp(dir, base+".tmp-*")
	if err != nil {
		return err
	}
	tmpName := f.Name()
	// 任何失败路径都尽力清理临时文件。
	defer func() {
		if err != nil {
			_ = os.Remove(tmpName)
		}
	}()
	if err = writeAndClose(f, data, tmpName); err != nil {
		return err
	}
	if err = os.Rename(tmpName, m.path); err != nil {
		return err
	}
	return nil
}

func writeAndClose(f *os.File, data []byte, name string) (err error) {
	defer func() {
		if cerr := f.Close(); err == nil {
			err = cerr
		}
	}()
	if _, err = f.Write(data); err != nil {
		return err
	}
	if err = f.Sync(); err != nil {
		return err
	}
	return nil
}

// canonicalHash 计算一批帧的确定性指纹，用于重复导入判定。直接按
// IEEE-754 位编码，NaN/Inf 也有稳定表示，不依赖 JSON 对其不支持的问题。
func canonicalHash(frames []Frame) string {
	h := sha256.New()
	var lenBuf [4]byte
	var numBuf [8]byte
	putU32 := func(v uint32) {
		binary.BigEndian.PutUint32(lenBuf[:], v)
		h.Write(lenBuf[:])
	}
	putU64 := func(v uint64) {
		binary.BigEndian.PutUint64(numBuf[:], v)
		h.Write(numBuf[:])
	}
	putStr := func(s string) {
		putU32(uint32(len(s)))
		h.Write([]byte(s))
	}

	putU32(uint32(len(frames)))
	for _, fr := range frames {
		putU64(uint64(fr.Time))
		putU64(math.Float64bits(fr.DX))
		putU64(math.Float64bits(fr.DY))
		putU64(math.Float64bits(fr.DHeading))
		putU64(math.Float64bits(fr.MoveVariance))
		putU32(uint32(len(fr.Observations)))
		for _, ob := range fr.Observations {
			putStr(ob.ID)
			putU64(math.Float64bits(ob.X))
			putU64(math.Float64bits(ob.Y))
		}
	}
	return fmt.Sprintf("%x", h.Sum(nil))
}
