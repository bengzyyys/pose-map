// Package posemap 是本地定位与地图的本地基线，提供可由本地程序调用的
// 二维轨迹与路标功能：创建地图、按段导入有序帧、查询历史位姿与矩形区域
// 内的路标，并把全部数据持久化到本地文件，关闭后重新打开结果一致。
package posemap

import (
	"errors"
	"math"
	"strconv"
)

// Config 描述创建一份新地图时的初始状态与两个阈值。
//
// 位置单位为米，时间单位为整数毫秒，朝向单位为弧度。初始位置方差
// InitialVariance 必须非负；最大帧间隔 MaxInterval（毫秒）与路标合并
// 距离 MergeDistance（米）必须为正。
type Config struct {
	InitialTime     int64   // 初始时间（毫秒）
	InitialX        float64 // 初始位置 X（米，地图坐标）
	InitialY        float64 // 初始位置 Y（米，地图坐标）
	InitialHeading  float64 // 初始朝向（弧度）
	InitialVariance float64 // 初始位置方差（非负）
	MaxInterval     int64   // 相邻位姿允许的最大时间间隔（毫秒，必须为正）
	MergeDistance   float64 // 路标观测合并距离（米，必须为正）
}

// Observation 是一帧中对单个路标的一次观测，位置在机器人自身坐标下。
type Observation struct {
	ID string  // 路标标识，非空
	X  float64 // 自身坐标下的 X（米）
	Y  float64 // 自身坐标下的 Y（米）
}

// Frame 是一段有序轨迹中的一帧：时间、自身坐标下的平移、朝向增量、
// 非负运动方差以及零个或多个路标观测。
type Frame struct {
	Time         int64         // 帧时间（毫秒）
	DX           float64       // 自身坐标下平移 X（米）
	DY           float64       // 自身坐标下平移 Y（米）
	DHeading     float64       // 朝向增量（弧度）
	MoveVariance float64       // 运动方差（非负）
	Observations []Observation // 零个或多个路标观测，同帧按输入次序处理
}

// Segment 是一批带有非空段标识的有序帧。
type Segment struct {
	ID     string  // 段标识，非空
	Frames []Frame // 有序帧
}

// Pose 是一份二维位姿及其位置方差。
type Pose struct {
	Time     int64
	X        float64
	Y        float64
	Heading  float64
	Variance float64
}

// Landmark 是地图坐标下的一个路标及其被接受的观测次数。
type Landmark struct {
	ID    string
	X     float64
	Y     float64
	Count int
}

// Invalidation 是一次路标失效操作：非空操作标识、非空原因与一组非空、
// 不重复的路标标识。提交时这些路标的当前有效记录同时失效，失效时间取
// 提交时的当前位姿时间。
type Invalidation struct {
	ID        string   // 操作标识，非空
	Reason    string   // 失效原因，非空
	Landmarks []string // 路标标识，非空且不重复
}

// InvalidationResult 是一次成功失效操作的结果：失效时间（提交时的当前
// 位姿时间）以及每个被撤下路标的标识与其失效的出现编号。
type InvalidationResult struct {
	Time      int64
	Landmarks []InvalidatedLandmark
}

// InvalidatedLandmark 记录一个被撤下路标的标识与被失效记录的出现编号。
type InvalidatedLandmark struct {
	ID         string
	Occurrence int
}

// Appearance 是同一标识某一次出现的记录：出现编号（同一标识首次出现为
// 1，每次失效后再次出现递增）、位置、观测计数、是否仍为当前有效记录、
// 首次观测时间，以及失效时间与原因。仍有效（Active 为真）时失效时间与
// 原因没有意义；旧文件缺少逐帧来源时 HasFirstSeen 为假，首次观测时间
// 未知。
type Appearance struct {
	Number        int // 出现编号，从 1 开始
	Landmark      Landmark
	Active        bool  // 是否为当前有效记录
	FirstSeenTime int64 // 首次观测所在帧的时间
	HasFirstSeen  bool  // 旧文件缺少逐帧来源时为假，表示首次观测时间未知
	InvalidTime   int64 // 失效时间（提交失效时的当前位姿时间）
	InvalidReason string
	InvalidOpID   string
}

// LandmarkHistory 是同一标识按出现编号递增排列的全部出现记录。
type LandmarkHistory struct {
	ID          string
	Appearances []Appearance
}

// CorrectionTarget 是已确认回环提交的目标：某个已保存帧应有的位姿与
// 非负位置方差。位置单位为米，朝向单位为弧度。
type CorrectionTarget struct {
	X        float64
	Y        float64
	Heading  float64
	Variance float64
}

// Correction 是一次已确认回环校正请求：非空标识、必须命中已导入帧的
// 锚点时间，以及该帧应有的位姿与非负位置方差。
type Correction struct {
	ID     string           // 校正标识，非空
	Anchor int64            // 已保存帧的准确时间（毫秒）
	Target CorrectionTarget // 锚点帧应有的位姿与方差
}

// PoseChange 是一帧位姿的校正前后值。
type PoseChange struct {
	Before Pose
	After  Pose
}

// LandmarkChange 是一个路标某一次出现的校正前后值（观测次数不变）。
// Occurrence 给出受影响的出现编号；同一标识的不同出现分别记录。
type LandmarkChange struct {
	ID         string
	Occurrence int
	Before     Landmark
	After      Landmark
}

// CorrectionRecord 是一次成功校正按提交次序保留的记录。后续校正可
// 覆盖相同时间范围，但先前记录永不被改写。
type CorrectionRecord struct {
	ID        string           // 校正标识
	Anchor    int64            // 锚点帧时间
	Target    CorrectionTarget // 提交目标
	EndTime   int64            // 受影响末帧时间（提交时的末帧）
	Poses     []PoseChange     // 受影响位姿的校正前后值，按时间升序
	Landmarks []LandmarkChange // 受影响路标出现的校正前后值，按标识再按出现编号升序
}

// RejectError 表示一次段导入、路标失效或回环校正因语义不合法而被整体
// 拒绝。
//
// Kind 为可区分的拒绝原因（见 Reject* 常量）；当原因涉及具体帧时，
// Frame 给出从零开始的帧序号且 HasFrame 为真；路标冲突时 Landmark
// 给出涉及的路标标识；重复段内容冲突时 Landmark 给出冲突的段标识。
// 两种情况下 HasLandmark 均为真。校正类原因用 Time 给出提交的锚点
// 时间且 HasTime 为真；校正中路标冲突时 Time 改为给出冲突观测所在帧的
// 时间、Landmark 给出路标标识、Occurrence 给出冲突的出现编号（1 起），
// HasLandmark 与 HasOccurrence 均为真。校正中某条观测转换到地图坐标后
// 出现非有限数值时同样处理：Time 为该观测所在帧的实际时间、Landmark
// 与 Occurrence 指出路标标识及其所属的出现编号。失效操作中找不到路标
// 或路标当前已失效时，Landmark 给出路标标识。
type RejectError struct {
	Kind          string
	Frame         int
	HasFrame      bool
	Landmark      string
	HasLandmark   bool
	Time          int64
	HasTime       bool
	Occurrence    int
	HasOccurrence bool
}

func (e *RejectError) Error() string {
	switch e.Kind {
	case RejectEmptySegment:
		return "posemap: segment has no frames"
	case RejectEmptyID:
		if e.HasFrame {
			return "posemap: frame " + strconv.Itoa(e.Frame) + " has empty landmark id"
		}
		return "posemap: empty segment, correction or invalidation id"
	case RejectEmptyReason:
		return "posemap: invalidation reason is empty"
	case RejectEmptyLandmarkList:
		return "posemap: invalidation landmark list is empty"
	case RejectDuplicateLandmarkEntry:
		return "posemap: invalidation lists landmark " + e.Landmark + " more than once"
	case RejectLandmarkNotFound:
		return "posemap: landmark " + e.Landmark + " not found"
	case RejectLandmarkInactive:
		return "posemap: landmark " + e.Landmark + " has no active occurrence"
	case RejectInvalidationMismatch:
		return "posemap: invalidation id " + e.Landmark + " already saved with different content"
	case RejectNegativeVariance:
		if e.HasTime {
			return "posemap: correction target variance is negative"
		}
		return "posemap: frame " + strconv.Itoa(e.Frame) + " has negative motion variance"
	case RejectNonFinite:
		if e.HasTime {
			if e.HasLandmark {
				if e.HasOccurrence {
					return "posemap: correction makes observation of landmark " + e.Landmark + " occurrence " + strconv.Itoa(e.Occurrence) + " at frame time " + strconv.FormatInt(e.Time, 10) + " non-finite"
				}
				return "posemap: correction makes observation of landmark " + e.Landmark + " at frame time " + strconv.FormatInt(e.Time, 10) + " non-finite"
			}
			return "posemap: correction target or corrected result has non-finite value"
		}
		if e.HasLandmark {
			return "posemap: frame " + strconv.Itoa(e.Frame) + " has non-finite observation of landmark " + e.Landmark
		}
		return "posemap: frame " + strconv.Itoa(e.Frame) + " has non-finite value"
	case RejectTimeOrder:
		if e.Frame == 0 {
			return "posemap: first frame must be later than initial time or previous segment end"
		}
		return "posemap: frame " + strconv.Itoa(e.Frame) + " time is not strictly increasing"
	case RejectInterval:
		return "posemap: frame " + strconv.Itoa(e.Frame) + " exceeds the maximum time interval"
	case RejectLandmarkConflict:
		if e.HasTime {
			if e.HasOccurrence {
				return "posemap: correction conflicts landmark " + e.Landmark + " occurrence " + strconv.Itoa(e.Occurrence) + " at frame time " + strconv.FormatInt(e.Time, 10)
			}
			return "posemap: correction conflicts landmark " + e.Landmark + " at frame time " + strconv.FormatInt(e.Time, 10)
		}
		return "posemap: frame " + strconv.Itoa(e.Frame) + " observation of landmark " + e.Landmark + " is beyond merge distance"
	case RejectDuplicateMismatch:
		return "posemap: segment id " + e.Landmark + " already saved with different content"
	case RejectAnchorNotFound:
		return "posemap: anchor time " + strconv.FormatInt(e.Time, 10) + " does not match an imported frame"
	case RejectCorrectionMismatch:
		return "posemap: correction id " + e.Landmark + " already saved with different content"
	case RejectNoBasis:
		return "posemap: correction range from " + strconv.FormatInt(e.Time, 10) + " lacks per-frame observation sources"
	default:
		return "posemap: request rejected (" + e.Kind + ")"
	}
}

// 可区分的拒绝原因。
const (
	RejectEmptySegment           = "empty_segment"            // 空批次
	RejectEmptyID                = "empty_id"                 // 空段/路标/校正/失效标识
	RejectEmptyReason            = "empty_reason"             // 失效原因为空
	RejectEmptyLandmarkList      = "empty_landmark_list"      // 失效路标列表为空
	RejectDuplicateLandmarkEntry = "duplicate_landmark_entry" // 失效列表中路标重复
	RejectLandmarkNotFound       = "landmark_not_found"       // 失效的路标从未出现
	RejectLandmarkInactive       = "landmark_inactive"        // 失效的路标当前无有效记录
	RejectInvalidationMismatch   = "invalidation_mismatch"    // 同一失效标识对应不同内容
	RejectNegativeVariance       = "negative_variance"        // 负运动方差或负目标方差
	RejectNonFinite              = "non_finite"               // 非有限数值（NaN/Inf）
	RejectTimeOrder              = "time_order"               // 时间未严格递增或首帧不够晚
	RejectInterval               = "interval_exceeded"        // 相邻位姿间隔超过上限
	RejectLandmarkConflict       = "landmark_conflict"        // 同标识路标超出合并距离
	RejectDuplicateMismatch      = "duplicate_mismatch"       // 同一段标识对应不同内容
	RejectAnchorNotFound         = "anchor_not_found"         // 校正锚点未命中已导入帧
	RejectCorrectionMismatch     = "correction_mismatch"      // 同一校正标识对应不同内容
	RejectNoBasis                = "no_basis"                 // 旧地图缺少逐帧观测来源
)

// 哨兵错误，供 errors.Is 使用。
var (
	// ErrClosed 在已关闭的地图上操作时返回。
	ErrClosed = errors.New("posemap: map is closed")
	// ErrInvalidConfig 在创建地图的配置不合法时返回。
	ErrInvalidConfig = errors.New("posemap: invalid config")
	// ErrInvalidRect 在矩形查询的上下界颠倒时返回。
	ErrInvalidRect = errors.New("posemap: invalid rectangle")
	// ErrNotFound 在查询时间早于初始时间（无任何不晚于它的位姿）时返回。
	ErrNotFound = errors.New("posemap: pose not found")
	// ErrLandmarkNotFound 在按标识查询路标历次出现而该标识从未出现时返回。
	ErrLandmarkNotFound = errors.New("posemap: landmark not found")
	// ErrCorrupt 在数据文件损坏、截断或校验失败时返回。
	ErrCorrupt = errors.New("posemap: corrupt data file")
	// ErrUnsupportedVersion 在数据文件格式版本不受支持时返回。
	ErrUnsupportedVersion = errors.New("posemap: unsupported file format version")
)

// normalizeAngle 把任意角度归一到 [-π, π)。
func normalizeAngle(a float64) float64 {
	a = math.Mod(a+math.Pi, 2*math.Pi)
	if a < 0 {
		a += 2 * math.Pi
	}
	return a - math.Pi
}

// isFinite 判断数值既不是 NaN 也不是 Inf。
func isFinite(v float64) bool { return !math.IsNaN(v) && !math.IsInf(v, 0) }

// mergeMean 把一个新样本 v 并入既有等权均值（已有 count 个样本），
// 返回 count+1 个样本的均值。与 (mean*count + v)/(count+1) 数学等价，
// 但按 mean + (v-mean)/(count+1) 计算：大数值坐标下 mean*count 会溢出
// 为无穷，而这里每一步都只是有限值之间的凸组合，结果保持有限且落在
// 参与平均的坐标范围内。调用方须保证 v 与 mean 均为有限值且二者之差
// 有限（被接受的观测与当前均值相距不超过有限的合并距离，自然满足）。
func mergeMean(mean, v float64, count int) float64 {
	return mean + (v-mean)/float64(count+1)
}

// localToMap 把机器人自身坐标下的点 (lx, ly) 按给定地图坐标位姿
// 转换到地图坐标：旋转 heading 后平移。
func localToMap(px, py, heading, lx, ly float64) (float64, float64) {
	c, s := math.Cos(heading), math.Sin(heading)
	return px + c*lx - s*ly, py + s*lx + c*ly
}
