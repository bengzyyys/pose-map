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
	Version       int                `json:"version"`
	Config        Config             `json:"config"`
	Trajectory    []Pose             `json:"trajectory"`
	Landmarks     []landmarkJSON     `json:"landmarks"`
	Segments      []segmentJSON      `json:"segments"`
	Sources       []*frameSrcJSON    `json:"sources,omitempty"`       // 新格式：逐帧依据，旧帧为 null；旧文件整列缺省
	Corrections   []corrRecJSON      `json:"corrections,omitempty"`   // 新格式：按提交次序的校正记录
	Invalidations []invalidationJSON `json:"invalidations,omitempty"` // 新格式：失效操作记录
}

type landmarkJSON struct {
	ID          string           `json:"id"`
	X           float64          `json:"x,omitempty"`            // 旧格式：聚合位置
	Y           float64          `json:"y,omitempty"`            // 旧格式
	Count       int              `json:"count,omitempty"`        // 旧格式
	LegacyCount int              `json:"legacy_count,omitempty"` // 旧格式
	LegacyMX    float64          `json:"legacy_mx,omitempty"`    // 旧格式
	LegacyMY    float64          `json:"legacy_my,omitempty"`    // 旧格式
	Appearances []appearanceJSON `json:"appearances,omitempty"`  // 新格式：各次出现
}

type appearanceJSON struct {
	Number        int     `json:"number"`
	X             float64 `json:"x"`
	Y             float64 `json:"y"`
	Count         int     `json:"count"`
	Valid         bool    `json:"valid"`
	FirstTime     int64   `json:"first_time"`
	InvalidTime   int64   `json:"invalid_time,omitempty"`
	InvalidReason string  `json:"invalid_reason,omitempty"`
	LegacyCount   int     `json:"legacy_count,omitempty"`
	LegacyMX      float64 `json:"legacy_mx,omitempty"`
	LegacyMY      float64 `json:"legacy_my,omitempty"`
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
	ID          string                 `json:"id"`
	Before      Landmark               `json:"before"`
	After       Landmark               `json:"after"`
	Appearances []appearanceChangeJSON `json:"appearances,omitempty"`
}

type appearanceChangeJSON struct {
	Number int      `json:"number"`
	Before Landmark `json:"before"`
	After  Landmark `json:"after"`
}

type corrRecJSON struct {
	ID        string               `json:"id"`
	Anchor    int64                `json:"anchor"`
	Target    CorrectionTarget     `json:"target"`
	EndTime   int64                `json:"end_time"`
	Poses     []poseChangeJSON     `json:"poses"`
	Landmarks []landmarkChangeJSON `json:"landmarks"`
}

type invalidationJSON struct {
	ID          string         `json:"id"`
	Reason      string         `json:"reason"`
	LandmarkIDs []string       `json:"landmark_ids"`
	InvalidTime int64          `json:"invalid_time"`
	Appearances map[string]int `json:"appearances"`
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
		Invalidations: make([]invalidationJSON, 0, len(st.invalidations)),
	}
	for id, lm := range st.landmarks {
		lj := landmarkJSON{ID: id, Appearances: make([]appearanceJSON, 0, len(lm.appearances))}
		for _, a := range lm.appearances {
			lj.Appearances = append(lj.Appearances, appearanceJSON{
				Number:        a.number,
				X:             a.x,
				Y:             a.y,
				Count:         a.count,
				Valid:         a.valid,
				FirstTime:     a.firstTime,
				InvalidTime:   a.invalidTime,
				InvalidReason: a.invalidReason,
				LegacyCount:   a.legacyCount,
				LegacyMX:      a.legacyMX,
				LegacyMY:      a.legacyMY,
			})
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
			acj := landmarkChangeJSON{ID: lc.ID, Before: lc.Before, After: lc.After}
			if len(lc.Appearances) > 0 {
				acj.Appearances = make([]appearanceChangeJSON, len(lc.Appearances))
				for j, ac := range lc.Appearances {
					acj.Appearances[j] = appearanceChangeJSON{Number: ac.Number, Before: ac.Before, After: ac.After}
				}
			}
			cj.Landmarks[i] = acj
		}
		fd.Corrections = append(fd.Corrections, cj)
	}
	for id, rec := range st.invalidations {
		apps := make(map[string]int, len(rec.result.Appearances))
		for k, v := range rec.result.Appearances {
			apps[k] = v
		}
		fd.Invalidations = append(fd.Invalidations, invalidationJSON{
			ID:          id,
			Reason:      rec.reason,
			LandmarkIDs: append([]string(nil), rec.landmarkIDs...),
			InvalidTime: rec.result.InvalidTime,
			Appearances: apps,
		})
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
		config:        fd.Config,
		trajectory:    fd.Trajectory,
		landmarks:     make(map[string]*landmarkState, len(fd.Landmarks)),
		segments:      make(map[string]*segmentRecord, len(fd.Segments)),
		sources:       make([]*frameSource, len(fd.Trajectory)-1),
		corrections:   make([]*CorrectionRecord, 0, len(fd.Corrections)),
		corrIndex:     make(map[string]string, len(fd.Corrections)),
		invalidations: make(map[string]*invalidationRecord, len(fd.Invalidations)),
	}
	// 旧文件不携带逐帧依据：有帧而 sources 缺省即旧地图。无帧时二者
	// 无法区分也无需区分（尚无范围可校正）。
	oldFormat := len(fd.Sources) == 0 && len(fd.Trajectory) > 1
	for _, lm := range fd.Landmarks {
		ls := &landmarkState{}
		if len(lm.Appearances) > 0 {
			// 新格式：直接加载各次出现。
			for _, aj := range lm.Appearances {
				ls.appearances = append(ls.appearances, &landmarkAppearance{
					number:        aj.Number,
					x:             aj.X,
					y:             aj.Y,
					count:         aj.Count,
					valid:         aj.Valid,
					firstTime:     aj.FirstTime,
					invalidTime:   aj.InvalidTime,
					invalidReason: aj.InvalidReason,
					legacyCount:   aj.LegacyCount,
					legacyMX:      aj.LegacyMX,
					legacyMY:      aj.LegacyMY,
				})
			}
			// 旧文件（无逐帧依据）：首次出现的全部既有观测视为固定旧贡献。
			if oldFormat && len(ls.appearances) > 0 {
				first := ls.appearances[0]
				first.legacyCount = first.count
				first.legacyMX, first.legacyMY = first.x, first.y
				first.firstTime = -1
			}
		} else {
			// 旧格式：全部既有观测视为首次出现的固定旧贡献，首次观测时间未知。
			legacyCount := lm.LegacyCount
			legacyMX, legacyMY := lm.LegacyMX, lm.LegacyMY
			if oldFormat {
				legacyCount = lm.Count
				legacyMX, legacyMY = lm.X, lm.Y
			}
			ls.appearances = append(ls.appearances, &landmarkAppearance{
				number:      1,
				x:           lm.X,
				y:           lm.Y,
				count:       lm.Count,
				valid:       true,
				firstTime:   -1,
				legacyCount: legacyCount,
				legacyMX:    legacyMX,
				legacyMY:    legacyMY,
			})
		}
		st.landmarks[lm.ID] = ls
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
			change := LandmarkChange{ID: lc.ID, Before: lc.Before, After: lc.After}
			for _, ac := range lc.Appearances {
				change.Appearances = append(change.Appearances, AppearanceChange{
					Number: ac.Number, Before: ac.Before, After: ac.After,
				})
			}
			rec.Landmarks[i] = change
		}
		st.corrections = append(st.corrections, rec)
		st.corrIndex[cj.ID] = canonicalCorrectionHash(cj.Anchor, cj.Target)
	}
	for _, ij := range fd.Invalidations {
		apps := make(map[string]int, len(ij.Appearances))
		for k, v := range ij.Appearances {
			apps[k] = v
		}
		st.invalidations[ij.ID] = &invalidationRecord{
			reason:      ij.Reason,
			landmarkIDs: append([]string(nil), ij.LandmarkIDs...),
			result:      InvalidateResult{InvalidTime: ij.InvalidTime, Appearances: apps},
		}
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
	// 新格式的逐帧依据必须与轨迹（除初始位姿外）一一对应；旧文件缺省
	// 该字段（解码为 nil），数量为 0。
	if len(fd.Sources) != 0 && len(fd.Sources) != len(fd.Trajectory)-1 {
		return fmt.Errorf("%w: frame sources length %d for %d frames", ErrCorrupt, len(fd.Sources), len(fd.Trajectory)-1)
	}
	timeAt := func(t int64) bool {
		for _, p := range fd.Trajectory {
			if p.Time == t {
				return true
			}
		}
		return false
	}
	seenIDs := make(map[string]struct{}, len(fd.Landmarks))
	for _, lm := range fd.Landmarks {
		if lm.ID == "" {
			return fmt.Errorf("%w: invalid landmark record", ErrCorrupt)
		}
		if len(lm.Appearances) > 0 {
			// 新格式：校验各次出现。
			seenNums := make(map[int]struct{}, len(lm.Appearances))
			for _, aj := range lm.Appearances {
				if aj.Number < 1 || aj.Count < 1 || !isFinite(aj.X) || !isFinite(aj.Y) {
					return fmt.Errorf("%w: invalid appearance for landmark %s", ErrCorrupt, lm.ID)
				}
				if aj.LegacyCount < 0 || aj.LegacyCount > aj.Count {
					return fmt.Errorf("%w: invalid legacy count for landmark %s", ErrCorrupt, lm.ID)
				}
				if aj.LegacyCount > 0 && (!isFinite(aj.LegacyMX) || !isFinite(aj.LegacyMY)) {
					return fmt.Errorf("%w: invalid legacy mean for landmark %s", ErrCorrupt, lm.ID)
				}
				if _, dup := seenNums[aj.Number]; dup {
					return fmt.Errorf("%w: duplicate appearance %d for landmark %s", ErrCorrupt, aj.Number, lm.ID)
				}
				seenNums[aj.Number] = struct{}{}
			}
		} else {
			// 旧格式：校验聚合字段。
			if lm.Count < 1 || !isFinite(lm.X) || !isFinite(lm.Y) {
				return fmt.Errorf("%w: invalid landmark record", ErrCorrupt)
			}
			if lm.LegacyCount < 0 || lm.LegacyCount > lm.Count {
				return fmt.Errorf("%w: invalid legacy count for landmark %s", ErrCorrupt, lm.ID)
			}
			if lm.LegacyCount > 0 && (!isFinite(lm.LegacyMX) || !isFinite(lm.LegacyMY)) {
				return fmt.Errorf("%w: invalid legacy mean for landmark %s", ErrCorrupt, lm.ID)
			}
		}
		if _, dup := seenIDs[lm.ID]; dup {
			return fmt.Errorf("%w: duplicate landmark %s", ErrCorrupt, lm.ID)
		}
		seenIDs[lm.ID] = struct{}{}
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
		if cr.ID == "" || !timeAt(cr.Anchor) || !timeAt(cr.EndTime) || cr.Anchor > cr.EndTime {
			return fmt.Errorf("%w: invalid correction record", ErrCorrupt)
		}
		if len(cr.Poses) == 0 {
			return fmt.Errorf("%w: correction %s has no affected poses", ErrCorrupt, cr.ID)
		}
		for _, pc := range cr.Poses {
			if !finitePose(pc.Before) || !finitePose(pc.After) {
				return fmt.Errorf("%w: correction %s has non-finite pose", ErrCorrupt, cr.ID)
			}
		}
		if _, dup := corrIDs[cr.ID]; dup {
			return fmt.Errorf("%w: duplicate correction %s", ErrCorrupt, cr.ID)
		}
		corrIDs[cr.ID] = struct{}{}
	}
	invIDs := make(map[string]struct{}, len(fd.Invalidations))
	for _, inv := range fd.Invalidations {
		if inv.ID == "" || inv.Reason == "" || len(inv.LandmarkIDs) == 0 {
			return fmt.Errorf("%w: invalid invalidation record", ErrCorrupt)
		}
		if _, dup := invIDs[inv.ID]; dup {
			return fmt.Errorf("%w: duplicate invalidation %s", ErrCorrupt, inv.ID)
		}
		invIDs[inv.ID] = struct{}{}
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
