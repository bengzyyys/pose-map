package posemap

import "math"

// observation.go 承载“一条路标观测如何被一次出现接纳并并入平均位置”这一
// 唯一业务规则，供轨迹导入（ImportSegment 的段内暂存）与回环校正
// （Correct 的观测重放）共同使用，保证两条流程对同一种观测的接纳与合并
// 判定始终一致：
//
//   - 观测先按所在帧运动完成后的位姿从机器人自身坐标转换到地图坐标；
//   - 转换结果出现 NaN/无穷时一律按数值异常处理，优先于距离冲突，即使是
//     该出现的第一条观测；
//   - 一次出现首次从一条观测开始：尚无已接受样本时直接接纳、计数从 1
//     开始，不做距离判定；
//   - 其后每条观测与当时的平均位置（含同帧已接纳观测的影响）比较，距离
//     恰好等于合并上限仍接纳，严格超过上限即拒绝；
//   - 接纳后位置为全部已接受观测的等权平均（mergeMean 增量计算，大坐标
//     下保持有限），计数按实际接纳次数加一；同帧多条观测严格按输入次序
//     逐条处理，不能先汇总整帧再统一判断。
//
// 聚合器只保存与接纳判定有关的运行状态（当前平均位置与计数）；出现编号、
// 有效状态、失效信息、首次观测时间与旧文件固定历史贡献等身份元数据仍由
// 各自流程在提交/记录时维护。
type obsAccumulator struct {
	x     float64 // 已接受观测的等权平均 X（地图坐标）
	y     float64 // 已接受观测的等权平均 Y（地图坐标）
	count int     // 已接受观测次数
}

// acceptResult 是一次观测接纳尝试的结果。
type acceptResult uint8

const (
	acceptOK        acceptResult = iota // 已接纳并计入平均
	acceptNonFinite                     // 转换到地图坐标后为 NaN/无穷
	acceptConflict                      // 与当前平均位置严格超过合并上限
)

// seedObsAccumulator 用一份已存在的固定历史贡献（平均位置与已计入次数）
// 初始化聚合器。回环校正重放旧文件第 1 次出现时，旧观测不可重放，作为
// 固定起点保留：位置不随校正改变，新观测仍对其做增量平均并接受距离判定。
func seedObsAccumulator(x, y float64, count int) obsAccumulator {
	return obsAccumulator{x: x, y: y, count: count}
}

// accept 按位姿 pose 把自身坐标下的观测 ob 转换到地图坐标并尝试接纳。
// mergeDistance 为路标合并距离上限（米，配置已保证为正有限值）。接纳时
// 就地更新平均位置与计数并返回 acceptOK；转换结果非有限返回
// acceptNonFinite；已有样本且新观测与当前平均位置的距离严格超过上限时
// 返回 acceptConflict，此时聚合状态保持不变（调用方会整段/整次拒绝，不
// 留下部分结果）。
func (a *obsAccumulator) accept(pose Pose, ob Observation, mergeDistance float64) acceptResult {
	mx, my := localToMap(pose.X, pose.Y, pose.Heading, ob.X, ob.Y)
	// 数值异常优先于距离冲突：转换结果可能因位姿/观测坐标过大而溢出，
	// 无论该出现此前是否已有观测都按同一异常处理。
	if !isFinite(mx) || !isFinite(my) {
		return acceptNonFinite
	}
	// 首次出现的第一条观测直接接纳：平均位置就是该观测本身、计数从 1
	// 开始；其后与“当时的平均位置”比较，端点恰好等于合并上限仍可接纳，
	// 同帧前一条接纳后的均值立即参与下一条判定。
	if a.count > 0 {
		if math.Hypot(mx-a.x, my-a.y) > mergeDistance {
			return acceptConflict
		}
		a.x = mergeMean(a.x, mx, a.count)
		a.y = mergeMean(a.y, my, a.count)
	} else {
		a.x, a.y = mx, my
	}
	a.count++
	return acceptOK
}
