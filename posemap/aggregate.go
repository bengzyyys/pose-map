package posemap

import "math"

// aggregate.go 承载“同一路标一次出现的观测接纳与合并规则”这一公共业务，
// 供轨迹导入（ImportSegment）与回环校正重放（Correct）共同使用，保证两条
// 流程对同一种观测的转换、接纳、等权平均与计数规则始终一致：
//
//   - 观测先由机器人自身坐标按当时位姿转换到地图坐标（localToMap）；
//   - 转换结果出现非有限数值时，优先按数值异常处理，不降级为距离冲突，
//     也不留到保存时才报普通错误；
//   - 该次出现的第一条观测直接作为平均位置起点、计数 1，不做距离判断；
//   - 其后每条观测与当时的平均位置比较，距离恰好等于合并上限仍可接纳，
//     超过上限则拒绝（且整条操作随之整体拒绝）；
//   - 接纳后位置是全部已接受观测的等权平均，按 mergeMean 增量计算以避免
//     大坐标下 mean*count 溢出，计数按实际接纳次数加一；
//   - 同帧多条观测按输入次序逐条处理，前一条接纳后的平均位置会影响下一
//     条的距离判断，不能先汇总整帧再统一判断。
//
// 聚合器只持有规则本身关心的计数与位置；出现编号、有效状态、失效信息、
// 首次观测时间等仍由各自流程在聚合器之外维护，规则不改变它们。

// obsRejectReason 区分一次观测被拒绝的原因。
type obsRejectReason int

const (
	obsAccepted  obsRejectReason = iota // 接纳（含首次建点）
	obsNonFinite                        // 转换到地图坐标后为非有限数值
	obsConflict                         // 与当时平均位置的距离超过合并上限
)

// observationAgg 是同一路标某一次出现对已接纳观测的增量聚合：等权平均
// 位置 (x,y) 与已接纳条数 count。count 为 0 时该次出现尚未接纳任何观测，
// 此时 (x,y) 无意义，第一条观测只做有限性检查即直接建点。
//
// 两种流程会在重放本段观测之前，先以一组“不在本次重放范围内、但固定有效”
// 的已接受观测作为起点（seed）：
//
//   - 导入时并入一条已有有效出现：该出现此前已提交的观测是固定历史，本段
//     新观测在其当前均值上增量合并；
//   - 回环校正重放旧文件第 1 次出现：旧文件缺少逐帧来源的观测无法重放，
//     其贡献固定为文件中的聚合值（legacyCount 次平均），依据完整的观测再
//     做增量平均。
//
// 起点本身不做距离检查（这些观测此前已按同一规则接纳），只有之后纳入的
// 观测才与（含固定起点在内的）当时平均位置比较。
type observationAgg struct {
	count int
	x     float64
	y     float64
}

// seed 以一组固定的已接受观测（count 条、等权平均 (x,y)）作为聚合起点。
// 调用时聚合器必须为空（尚未接纳任何观测）。
func (a *observationAgg) seed(count int, x, y float64) {
	a.count = count
	a.x = x
	a.y = y
}

// admit 按统一规则接纳一条观测：pose 为该观测所在帧运动完成后（导入）或
// 校正后（回环重放）的地图坐标位姿，ob 为机器人自身坐标下的观测，
// mergeDistance 为合并上限。
//
// 返回 obsAccepted 表示已并入等权平均（或作为第一条观测建点）；否则返回
// obsNonFinite（转换结果非有限，优先于距离冲突）或 obsConflict（距离严格
// 超过上限；恰好等于上限仍接纳）。被拒绝时聚合状态保持不变。
func (a *observationAgg) admit(pose Pose, ob Observation, mergeDistance float64) obsRejectReason {
	mx, my := localToMap(pose.X, pose.Y, pose.Heading, ob.X, ob.Y)
	// 位姿/观测坐标过大时转换可能溢出为 NaN/无穷：无论这是该次出现的
	// 第一条观测还是与既有平均合并，数值异常都优先于距离冲突。
	if !isFinite(mx) || !isFinite(my) {
		return obsNonFinite
	}
	if a.count > 0 {
		// 与当时平均位置比较：恰好等于合并上限仍可接纳，严格超过才拒绝。
		// 被接受观测与均值相距不超过有限的合并上限，二者之差有限，Hypot 不会溢出。
		if math.Hypot(mx-a.x, my-a.y) > mergeDistance {
			return obsConflict
		}
		a.x = mergeMean(a.x, mx, a.count)
		a.y = mergeMean(a.y, my, a.count)
	} else {
		// 首次出现（或该次出现重放到的第一条观测）：从一次观测开始，
		// 平均位置即观测位置、计数 1，不做距离判断。
		a.x, a.y = mx, my
	}
	a.count++
	return obsAccepted
}
