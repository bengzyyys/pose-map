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

// RejectError 表示一次段导入因语义不合法而被整段拒绝。
//
// Kind 为可区分的拒绝原因（见 Reject* 常量）；当原因涉及具体帧时，
// Frame 给出从零开始的帧序号且 HasFrame 为真；路标冲突时 Landmark
// 给出涉及的路标标识；重复段内容冲突时 Landmark 给出冲突的段标识。
// 两种情况下 HasLandmark 均为真。
type RejectError struct {
	Kind        string
	Frame       int
	HasFrame    bool
	Landmark    string
	HasLandmark bool
}

func (e *RejectError) Error() string {
	switch e.Kind {
	case RejectEmptySegment:
		return "posemap: segment has no frames"
	case RejectEmptyID:
		if e.HasFrame {
			return "posemap: frame " + strconv.Itoa(e.Frame) + " has empty landmark id"
		}
		return "posemap: empty segment id"
	case RejectNegativeVariance:
		return "posemap: frame " + strconv.Itoa(e.Frame) + " has negative motion variance"
	case RejectNonFinite:
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
		return "posemap: frame " + strconv.Itoa(e.Frame) + " observation of landmark " + e.Landmark + " is beyond merge distance"
	case RejectDuplicateMismatch:
		return "posemap: segment id " + e.Landmark + " already saved with different content"
	default:
		return "posemap: segment rejected (" + e.Kind + ")"
	}
}

// 可区分的拒绝原因。
const (
	RejectEmptySegment      = "empty_segment"      // 空批次
	RejectEmptyID           = "empty_id"           // 空段标识或空路标标识
	RejectNegativeVariance  = "negative_variance"  // 负运动方差
	RejectNonFinite         = "non_finite"         // 非有限数值（NaN/Inf）
	RejectTimeOrder         = "time_order"         // 时间未严格递增或首帧不够晚
	RejectInterval          = "interval_exceeded"  // 相邻位姿间隔超过上限
	RejectLandmarkConflict  = "landmark_conflict"  // 同标识路标超出合并距离
	RejectDuplicateMismatch = "duplicate_mismatch" // 同一段标识对应不同内容
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

// localToMap 把机器人自身坐标下的点 (lx, ly) 按给定地图坐标位姿
// 转换到地图坐标：旋转 heading 后平移。
func localToMap(px, py, heading, lx, ly float64) (float64, float64) {
	c, s := math.Cos(heading), math.Sin(heading)
	return px + c*lx - s*ly, py + s*lx + c*ly
}
