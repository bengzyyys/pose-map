// Package posemap 是本地定位与地图的本地基线。
//
// 调用方通过 Create 或 Open 打开一个本地数据文件，随后可以逐段导入
// （Import）有序帧：每帧给出时间、机器人自身坐标下的平移与朝向增量、
// 运动方差，以及零个或多个路标观测。包内按航迹推算维护地图坐标下的
// 轨迹与路标，并支持按时间回溯位姿、按矩形查询路标。
//
// 每段导入都是事务性的：整段校验通过并成功落盘后才会改变内存中的
// 轨迹、地图与重复导入信息；任何一帧失败都会拒绝整段，不留下任何
// 副作用。
package posemap

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"sync"
)

// Config 描述一份地图的初始状态与阈值。
//
// 位置单位为米，时间单位为整数毫秒，朝向单位为弧度。
// InitialVariance 必须非负；MaxInterval 与 MergeDistance 必须为正；
// 所有数值都必须是有限数。
type Config struct {
	// InitialTime 是初始时间（整数毫秒）。
	InitialTime int64
	// InitialX、InitialY 是初始位置（米）。
	InitialX, InitialY float64
	// InitialTheta 是初始朝向（弧度）。
	InitialTheta float64
	// InitialVariance 是初始位置方差，非负。
	InitialVariance float64
	// MaxInterval 是相邻位姿之间允许的最大时间间隔（整数毫秒），为正。
	MaxInterval int64
	// MergeDistance 是同标识路标观测允许的最大合并距离（米），为正。
	MergeDistance float64
}

// Pose 是某一时刻的位姿与位置方差。
type Pose struct {
	// Time 是该位姿的时间（整数毫秒）。
	Time int64
	// X、Y 是地图坐标下的位置（米）。
	X, Y float64
	// Theta 是地图坐标下的朝向（弧度），范围为 [-π, π)。
	Theta float64
	// Variance 是累计位置方差。
	Variance float64
}

// Observation 是一帧内对一个路标的观测。
//
// 位置为机器人自身坐标：x 轴朝前，y 轴朝左。
type Observation struct {
	// ID 是非空路标标识。
	ID string
	// X、Y 是机器人自身坐标下的路标位置（米）。
	X, Y float64
}

// Frame 是一段轨迹中的一帧。
type Frame struct {
	// Time 是该帧时间（整数毫秒），段内严格递增。
	Time int64
	// DX、DY 是按运动前朝向转换到地图坐标的机器人自身平移（米）。
	DX, DY float64
	// DTheta 是朝向增量（弧度），更新后归一到 [-π, π)。
	DTheta float64
	// Variance 是该帧的运动方差，非负，逐帧累加。
	Variance float64
	// Observations 是零个或多个路标观测，按输入次序处理。
	Observations []Observation
}

// Landmark 是地图上的一个路标及其被接受观测的统计。
type Landmark struct {
	// ID 是非空路标标识。
	ID string
	// X、Y 是该标识全部已接受观测位置的算术平均（米）。
	X, Y float64
	// Count 是已接受观测次数。
	Count int
}

// Kind 描述错误的类别。
type Kind int

const (
	// KindEmptyBatch：导入的段没有帧。
	KindEmptyBatch Kind = iota + 1
	// KindEmptySegmentID：段标识为空。
	KindEmptySegmentID
	// KindEmptyLandmarkID：路标观测标识为空。
	KindEmptyLandmarkID
	// KindNegativeVariance：运动方差为负。
	KindNegativeVariance
	// KindNonFinite：数值不是有限数（NaN 或 Inf）。
	KindNonFinite
	// KindTimeNotIncreasing：帧时间未严格递增，或首帧未晚于上一段末帧。
	KindTimeNotIncreasing
	// KindIntervalTooLarge：相邻位姿时间间隔超过上限。
	KindIntervalTooLarge
	// KindLandmarkConflict：同标识路标观测与当前地图位置距离超过合并距离。
	KindLandmarkConflict
	// KindSegmentConflict：同一段标识以不同内容重复导入。
	KindSegmentConflict
	// KindReversedRect：矩形查询的上下界颠倒。
	KindReversedRect
	// KindCorruptFile：文件损坏或格式非法。
	KindCorruptFile
	// KindUnsupportedVersion：文件格式版本不受支持。
	KindUnsupportedVersion
	// KindFileExists：创建时文件已存在。
	KindFileExists
	// KindClosed：地图已关闭。
	KindClosed
	// KindInvalidConfig：配置非法。
	KindInvalidConfig
)

// Error 是 posemap 包的结构化错误。
//
// 通过 errors.Is 与下列哨兵错误比较即可区分类别；涉及具体帧时 Frame
// 为从零开始的帧序号（不涉及帧时为 -1）；涉及路标或段标识时 ID 为
// 该标识。
type Error struct {
	Kind  Kind
	Frame int
	ID    string
}

func (e *Error) Error() string {
	switch e.Kind {
	case KindEmptyBatch:
		return "空批次：段至少需要一帧"
	case KindEmptySegmentID:
		return "段标识为空"
	case KindEmptyLandmarkID:
		return fmt.Sprintf("帧 %d：路标标识为空", e.Frame)
	case KindNegativeVariance:
		return fmt.Sprintf("帧 %d：运动方差为负", e.Frame)
	case KindNonFinite:
		return fmt.Sprintf("帧 %d：包含非有限数值", e.Frame)
	case KindTimeNotIncreasing:
		return fmt.Sprintf("帧 %d：时间未严格递增（首帧须晚于上一段末帧）", e.Frame)
	case KindIntervalTooLarge:
		return fmt.Sprintf("帧 %d：相邻位姿时间间隔超过上限", e.Frame)
	case KindLandmarkConflict:
		return fmt.Sprintf("帧 %d：路标 %q 与当前地图位置距离超过合并距离", e.Frame, e.ID)
	case KindSegmentConflict:
		return fmt.Sprintf("段 %q：标识已存在但内容与首次导入不同", e.ID)
	case KindReversedRect:
		return "矩形上下界颠倒"
	case KindCorruptFile:
		return "文件损坏或格式非法"
	case KindUnsupportedVersion:
		return "不支持的文件格式版本"
	case KindFileExists:
		return fmt.Sprintf("文件已存在：%s", e.ID)
	case KindClosed:
		return "地图已关闭"
	case KindInvalidConfig:
		return "配置非法：初始方差非负、两个阈值为正，且所有数值有限"
	default:
		return "未知错误"
	}
}

// Is 让同类别的 Error 可以通过 errors.Is 比较。
func (e *Error) Is(target error) bool {
	t, ok := target.(*Error)
	return ok && e.Kind == t.Kind
}

// 哨兵错误，供 errors.Is 区分拒绝原因。
var (
	ErrEmptyBatch         = &Error{Kind: KindEmptyBatch, Frame: -1}
	ErrEmptySegmentID     = &Error{Kind: KindEmptySegmentID, Frame: -1}
	ErrEmptyLandmarkID    = &Error{Kind: KindEmptyLandmarkID, Frame: -1}
	ErrNegativeVariance   = &Error{Kind: KindNegativeVariance, Frame: -1}
	ErrNonFinite          = &Error{Kind: KindNonFinite, Frame: -1}
	ErrTimeNotIncreasing  = &Error{Kind: KindTimeNotIncreasing, Frame: -1}
	ErrIntervalTooLarge   = &Error{Kind: KindIntervalTooLarge, Frame: -1}
	ErrLandmarkConflict   = &Error{Kind: KindLandmarkConflict, Frame: -1}
	ErrSegmentConflict    = &Error{Kind: KindSegmentConflict, Frame: -1}
	ErrReversedRect       = &Error{Kind: KindReversedRect, Frame: -1}
	ErrCorruptFile        = &Error{Kind: KindCorruptFile, Frame: -1}
	ErrUnsupportedVersion = &Error{Kind: KindUnsupportedVersion, Frame: -1}
	ErrFileExists         = &Error{Kind: KindFileExists, Frame: -1}
	ErrClosed             = &Error{Kind: KindClosed, Frame: -1}
	ErrInvalidConfig      = &Error{Kind: KindInvalidConfig, Frame: -1}
)

// 文件格式：magic(8) + version(4, 小端) + JSON 载荷 + sha256(载荷)(32)。
var fileMagic = [8]byte{'P', 'O', 'S', 'E', 'M', 'A', 'P', 0x01}

const (
	fileHeaderSize    = 12
	fileChecksumSize  = sha256.Size
	fileFormatVersion = uint32(1)
)

// Map 是一份打开的本地地图。
//
// Map 的方法可以并发调用；每段导入在内部以事务方式执行。
type Map struct {
	mu        sync.Mutex
	path      string
	cfg       Config
	poses     []Pose
	landmarks map[string]*landmarkState
	segments  map[string]*segmentRecord
	closed    bool
}

// landmarkState 是路标的内部状态。坐标为全部已接受观测位置之和，
// 地图位置为 SumX/Count、SumY/Count。
type landmarkState struct {
	SumX  float64 `json:"sum_x"`
	SumY  float64 `json:"sum_y"`
	Count int     `json:"count"`
}

// segmentRecord 是一段已成功导入内容的重复导入信息。
type segmentRecord struct {
	// Hash 是段内容（帧序列）的规范化哈希。
	Hash string `json:"hash"`
	// End 是首次导入返回的本段末位姿及方差。
	End Pose `json:"end"`
	// IDs 是本段涉及的路标标识（去重并排序）。
	IDs []string `json:"ids"`
}

// filePayload 是落盘的完整状态。
type filePayload struct {
	Config    Config                    `json:"config"`
	Poses     []Pose                    `json:"poses"`
	Landmarks map[string]*landmarkState `json:"landmarks"`
	Segments  map[string]*segmentRecord `json:"segments"`
}

// Create 在 path 创建一份新地图并写入初始状态。
//
// 若 path 已存在则返回 KindFileExists；配置非法返回 KindInvalidConfig。
// 创建后文件即包含初始位姿，随后可以直接查询与导入。
func Create(path string, cfg Config) (*Map, error) {
	if err := validateConfig(cfg); err != nil {
		return nil, err
	}
	// 以 O_EXCL 占位，避免覆盖已有文件。
	f, err := os.OpenFile(path, os.O_WRONLY|os.O_CREATE|os.O_EXCL, 0o644)
	if err != nil {
		if os.IsExist(err) {
			return nil, &Error{Kind: KindFileExists, Frame: -1, ID: path}
		}
		return nil, err
	}
	if err := f.Close(); err != nil {
		return nil, err
	}

	m := &Map{
		path: path,
		cfg:  cfg,
		poses: []Pose{{
			Time:     cfg.InitialTime,
			X:        cfg.InitialX,
			Y:        cfg.InitialY,
			Theta:    cfg.InitialTheta,
			Variance: cfg.InitialVariance,
		}},
		landmarks: make(map[string]*landmarkState),
		segments:  make(map[string]*segmentRecord),
	}
	if err := m.persist(m.poses, m.landmarks, nil); err != nil {
		os.Remove(path)
		return nil, err
	}
	return m, nil
}

// Open 打开 path 上已有的地图。
//
// 文件损坏或格式不受支持时返回错误，不会把它当作新地图覆盖。
func Open(path string) (*Map, error) {
	data, err := os.ReadFile(path)
	if err != nil {
		return nil, err
	}
	if len(data) < fileHeaderSize+fileChecksumSize {
		return nil, &Error{Kind: KindCorruptFile, Frame: -1}
	}
	if !bytesEqual(data[:8], fileMagic[:]) {
		return nil, &Error{Kind: KindCorruptFile, Frame: -1}
	}
	version := binary.LittleEndian.Uint32(data[8:12])
	if version != fileFormatVersion {
		return nil, &Error{Kind: KindUnsupportedVersion, Frame: -1}
	}

	payloadBytes := data[fileHeaderSize : len(data)-fileChecksumSize]
	got := sha256.Sum256(payloadBytes)
	if !bytesEqual(got[:], data[len(data)-fileChecksumSize:]) {
		return nil, &Error{Kind: KindCorruptFile, Frame: -1}
	}

	var payload filePayload
	if err := json.Unmarshal(payloadBytes, &payload); err != nil {
		return nil, &Error{Kind: KindCorruptFile, Frame: -1}
	}
	if err := validatePayload(&payload); err != nil {
		return nil, err
	}

	m := &Map{
		path:      path,
		cfg:       payload.Config,
		poses:     payload.Poses,
		landmarks: payload.Landmarks,
		segments:  payload.Segments,
	}
	if m.landmarks == nil {
		m.landmarks = make(map[string]*landmarkState)
	}
	if m.segments == nil {
		m.segments = make(map[string]*segmentRecord)
	}
	return m, nil
}

// Close 关闭地图。关闭后再调用查询或导入返回 KindClosed。
func (m *Map) Close() error {
	m.mu.Lock()
	defer m.mu.Unlock()
	m.closed = true
	return nil
}

// Import 导入一段以 segmentID 标识的有序帧。
//
// 段内帧时间必须严格递增，且首帧晚于初始时间或上一段末帧；相邻位姿
// 间隔不得超过 MaxInterval。平移按运动前朝向转换到地图坐标，朝向增量
// 更新后归一到 [-π, π)，位置方差逐帧累加。路标观测按该帧运动完成后的
// 位姿转换到地图坐标：首次出现的标识直接新增；再次出现时，仅当观测
// 位置与路标当前地图位置距离不超过 MergeDistance 才接受，合并坐标为
// 该标识全部已接受观测位置的算术平均。
//
// 任何一帧失败都拒绝整段，且不改变已保存的位姿、路标或观测次数。
// 成功时返回本段末位姿及方差、本段涉及的路标标识（去重并排序）。
//
// 若 segmentID 已成功导入过：内容完全相同则直接返回首次导入的结果，
// 不改变当前数据；内容不同则返回 KindSegmentConflict。失败的段修正
// 后可以用同一标识再次导入。
func (m *Map) Import(segmentID string, frames []Frame) (Pose, []string, error) {
	m.mu.Lock()
	defer m.mu.Unlock()

	if m.closed {
		return Pose{}, nil, &Error{Kind: KindClosed, Frame: -1}
	}
	if segmentID == "" {
		return Pose{}, nil, &Error{Kind: KindEmptySegmentID, Frame: -1}
	}
	if len(frames) == 0 {
		return Pose{}, nil, &Error{Kind: KindEmptyBatch, Frame: -1}
	}

	// 先计算段内容哈希并检查重复导入：已成功保存的段，相同内容直接
	// 返回首次导入的结果（即使后来又导入了其他段），不同内容明确拒绝。
	hash, err := hashFrames(frames)
	if err != nil {
		return Pose{}, nil, err
	}
	if rec, ok := m.segments[segmentID]; ok {
		if rec.Hash == hash {
			return rec.End, append([]string(nil), rec.IDs...), nil
		}
		return Pose{}, nil, &Error{Kind: KindSegmentConflict, Frame: -1, ID: segmentID}
	}

	// 新段：做不依赖路标合并的整段校验；此时不触碰任何状态。
	if err := validateFrames(frames, m.poses[len(m.poses)-1].Time, m.cfg.MaxInterval); err != nil {
		return Pose{}, nil, err
	}

	// 在副本上试算，失败则整段回滚。
	poses := append([]Pose(nil), m.poses...)
	lms := cloneLandmarks(m.landmarks)
	involved := make(map[string]struct{})

	last := poses[len(poses)-1]
	for i, f := range frames {
		// 平移按运动前朝向转换到地图坐标。
		c, s := math.Cos(last.Theta), math.Sin(last.Theta)
		nx := last.X + f.DX*c - f.DY*s
		ny := last.Y + f.DX*s + f.DY*c
		ntheta := normalizeAngle(last.Theta + f.DTheta)
		nvar := last.Variance + f.Variance

		// 路标观测按该帧运动完成后的位姿转换，并按输入次序处理。
		for _, o := range f.Observations {
			oc, os := math.Cos(ntheta), math.Sin(ntheta)
			ox := nx + o.X*oc - o.Y*os
			oy := ny + o.X*os + o.Y*oc

			lm, exists := lms[o.ID]
			if !exists {
				lms[o.ID] = &landmarkState{SumX: ox, SumY: oy, Count: 1}
			} else {
				cx := lm.SumX / float64(lm.Count)
				cy := lm.SumY / float64(lm.Count)
				if math.Hypot(ox-cx, oy-cy) > m.cfg.MergeDistance {
					return Pose{}, nil, &Error{Kind: KindLandmarkConflict, Frame: i, ID: o.ID}
				}
				lm.SumX += ox
				lm.SumY += oy
				lm.Count++
			}
			involved[o.ID] = struct{}{}
		}

		poses = append(poses, Pose{Time: f.Time, X: nx, Y: ny, Theta: ntheta, Variance: nvar})
		last = poses[len(poses)-1]
	}

	ids := sortedKeys(involved)
	rec := &segmentRecord{Hash: hash, End: last, IDs: ids}

	// 先落盘，成功后才提交内存状态；保存失败本次导入不生效。
	if err := m.persist(poses, lms, map[string]*segmentRecord{segmentID: rec}); err != nil {
		return Pose{}, nil, err
	}
	m.poses = poses
	m.landmarks = lms
	m.segments[segmentID] = rec
	return last, append([]string(nil), ids...), nil
}

// CurrentPose 返回当前（最新）位姿及方差。
func (m *Map) CurrentPose() (Pose, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return Pose{}, &Error{Kind: KindClosed, Frame: -1}
	}
	return m.poses[len(m.poses)-1], nil
}

// HistoryAt 返回不晚于 t 的最后一份位姿及方差。
//
// 若 t 早于初始时间，返回 (Pose{}, false)。
func (m *Map) HistoryAt(t int64) (Pose, bool) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return Pose{}, false
	}
	// poses 按时间严格递增：找第一个时间大于 t 的位姿，其前一个即所求。
	idx := sort.Search(len(m.poses), func(i int) bool { return m.poses[i].Time > t })
	if idx == 0 {
		return Pose{}, false
	}
	return m.poses[idx-1], true
}

// QueryRect 返回矩形区域 [x0,x1] × [y0,y1] 内的路标（含边界）及观测次数，
// 结果按标识排序。区域内没有路标时返回空切片。上下界颠倒返回
// KindReversedRect。
func (m *Map) QueryRect(x0, y0, x1, y1 float64) ([]Landmark, error) {
	m.mu.Lock()
	defer m.mu.Unlock()
	if m.closed {
		return nil, &Error{Kind: KindClosed, Frame: -1}
	}
	if x0 > x1 || y0 > y1 {
		return nil, &Error{Kind: KindReversedRect, Frame: -1}
	}
	var out []Landmark
	for id, lm := range m.landmarks {
		x := lm.SumX / float64(lm.Count)
		y := lm.SumY / float64(lm.Count)
		if x >= x0 && x <= x1 && y >= y0 && y <= y1 {
			out = append(out, Landmark{ID: id, X: x, Y: y, Count: lm.Count})
		}
	}
	sort.Slice(out, func(i, j int) bool { return out[i].ID < out[j].ID })
	return out, nil
}

// persist 把完整状态原子写入数据文件：先写同目录临时文件，再 rename。
// extra 用于追加本次导入产生的段记录。
func (m *Map) persist(poses []Pose, lms map[string]*landmarkState, extra map[string]*segmentRecord) error {
	payload := filePayload{
		Config:    m.cfg,
		Poses:     poses,
		Landmarks: lms,
		Segments:  make(map[string]*segmentRecord, len(m.segments)+len(extra)),
	}
	for k, v := range m.segments {
		payload.Segments[k] = v
	}
	for k, v := range extra {
		payload.Segments[k] = v
	}

	data, err := json.Marshal(&payload)
	if err != nil {
		return &Error{Kind: KindCorruptFile, Frame: -1}
	}
	sum := sha256.Sum256(data)

	buf := make([]byte, 0, fileHeaderSize+len(data)+fileChecksumSize)
	buf = append(buf, fileMagic[:]...)
	var version [4]byte
	binary.LittleEndian.PutUint32(version[:], fileFormatVersion)
	buf = append(buf, version[:]...)
	buf = append(buf, data...)
	buf = append(buf, sum[:]...)

	dir := filepath.Dir(m.path)
	tmp, err := os.CreateTemp(dir, ".posemap-*.tmp")
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	defer os.Remove(tmpName)

	if _, err := tmp.Write(buf); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Sync(); err != nil {
		tmp.Close()
		return err
	}
	if err := tmp.Close(); err != nil {
		return err
	}
	if err := os.Rename(tmpName, m.path); err != nil {
		return err
	}
	// 同步目录项，尽量保证落盘安全。
	if d, err := os.Open(dir); err == nil {
		d.Sync()
		d.Close()
	}
	return nil
}

// validateFrames 校验段内帧的数值、时间与间隔。lastTime 是上一段末帧
// 时间（或初始时间）。
func validateFrames(frames []Frame, lastTime int64, maxInterval int64) error {
	prev := lastTime
	for i, f := range frames {
		if !finite(f.DX) || !finite(f.DY) || !finite(f.DTheta) || !finite(f.Variance) {
			return &Error{Kind: KindNonFinite, Frame: i}
		}
		if f.Variance < 0 {
			return &Error{Kind: KindNegativeVariance, Frame: i}
		}
		for _, o := range f.Observations {
			if !finite(o.X) || !finite(o.Y) {
				return &Error{Kind: KindNonFinite, Frame: i, ID: o.ID}
			}
			if o.ID == "" {
				return &Error{Kind: KindEmptyLandmarkID, Frame: i}
			}
		}
		if f.Time <= prev {
			return &Error{Kind: KindTimeNotIncreasing, Frame: i}
		}
		if f.Time-prev > maxInterval {
			return &Error{Kind: KindIntervalTooLarge, Frame: i}
		}
		prev = f.Time
	}
	return nil
}

// validateConfig 校验创建配置。
func validateConfig(cfg Config) error {
	if !finite(cfg.InitialX) || !finite(cfg.InitialY) || !finite(cfg.InitialTheta) || !finite(cfg.InitialVariance) {
		return &Error{Kind: KindInvalidConfig, Frame: -1}
	}
	if cfg.InitialVariance < 0 {
		return &Error{Kind: KindInvalidConfig, Frame: -1}
	}
	if cfg.MaxInterval <= 0 {
		return &Error{Kind: KindInvalidConfig, Frame: -1}
	}
	if cfg.MergeDistance <= 0 || !finite(cfg.MergeDistance) {
		return &Error{Kind: KindInvalidConfig, Frame: -1}
	}
	return nil
}

// validatePayload 校验从文件读入的状态是否自洽。
func validatePayload(p *filePayload) error {
	if err := validateConfig(p.Config); err != nil {
		return err
	}
	if len(p.Poses) == 0 {
		return &Error{Kind: KindCorruptFile, Frame: -1}
	}
	first := p.Poses[0]
	if !finite(first.X) || !finite(first.Y) || !finite(first.Theta) || !finite(first.Variance) {
		return &Error{Kind: KindCorruptFile, Frame: -1}
	}
	if first.Time != p.Config.InitialTime ||
		first.X != p.Config.InitialX ||
		first.Y != p.Config.InitialY ||
		first.Theta != p.Config.InitialTheta ||
		first.Variance != p.Config.InitialVariance {
		return &Error{Kind: KindCorruptFile, Frame: -1}
	}
	last := first
	for i := 1; i < len(p.Poses); i++ {
		cur := p.Poses[i]
		if !finite(cur.X) || !finite(cur.Y) || !finite(cur.Theta) || !finite(cur.Variance) {
			return &Error{Kind: KindCorruptFile, Frame: -1}
		}
		if cur.Time <= last.Time || cur.Variance < last.Variance {
			return &Error{Kind: KindCorruptFile, Frame: -1}
		}
		last = cur
	}
	for _, lm := range p.Landmarks {
		if lm == nil || lm.Count < 1 || !finite(lm.SumX) || !finite(lm.SumY) {
			return &Error{Kind: KindCorruptFile, Frame: -1}
		}
	}
	for _, rec := range p.Segments {
		if rec == nil || rec.Hash == "" {
			return &Error{Kind: KindCorruptFile, Frame: -1}
		}
		if !finite(rec.End.X) || !finite(rec.End.Y) || !finite(rec.End.Theta) || !finite(rec.End.Variance) {
			return &Error{Kind: KindCorruptFile, Frame: -1}
		}
	}
	return nil
}

// hashFrames 返回帧序列的规范化哈希，用于重复导入判定。
func hashFrames(frames []Frame) (string, error) {
	h := sha256.New()
	if err := json.NewEncoder(h).Encode(frames); err != nil {
		return "", &Error{Kind: KindNonFinite, Frame: -1}
	}
	return hex.EncodeToString(h.Sum(nil)), nil
}

// cloneLandmarks 复制路标状态，供试算使用。
func cloneLandmarks(in map[string]*landmarkState) map[string]*landmarkState {
	out := make(map[string]*landmarkState, len(in))
	for k, v := range in {
		cp := *v
		out[k] = &cp
	}
	return out
}

// sortedKeys 返回集合中字典序排序后的键。
func sortedKeys(in map[string]struct{}) []string {
	out := make([]string, 0, len(in))
	for k := range in {
		out = append(out, k)
	}
	sort.Strings(out)
	return out
}

// normalizeAngle 把弧度归一到 [-π, π)。
func normalizeAngle(a float64) float64 {
	return math.Atan2(math.Sin(a), math.Cos(a))
}

// finite 判断浮点数是否有限（非 NaN、非 Inf）。
func finite(v float64) bool {
	return !math.IsNaN(v) && !math.IsInf(v, 0)
}

// bytesEqual 比较两个字节切片是否相等。
func bytesEqual(a, b []byte) bool {
	if len(a) != len(b) {
		return false
	}
	for i := range a {
		if a[i] != b[i] {
			return false
		}
	}
	return true
}
