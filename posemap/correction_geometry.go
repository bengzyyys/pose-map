package posemap

import "math"

// correctionGeometry 统一维护一次已确认回环校正在“校正前位姿 → 校正后位姿”
// 的位置与朝向之间隐含的同一次刚体重定位：锚点帧准确落到目标位置、朝向
// 归一到 [-π,π)；锚点之后每一帧保留各自校正前相对锚点的位置关系与朝向
// 差，随锚点一起平移并旋转。提交校正（correction.go）与打开地图时逐帧核对
// 校正记录（persistence.go）共用这一个实现，两处不再分别维护同一套规则。
//
// 这里只描述位置与朝向的关系：帧时间不变、锚点之前的位姿不变，校正后方差
// 另有累计规则，路标观测随新位姿另有重放规则，均不属于本类型的职责。
type correctionGeometry struct {
	target   CorrectionTarget
	anchor   Pose    // 锚点帧的校正前快照（打开核对时取记录自己的快照）
	heading  float64 // 归一到 [-π,π) 的目标朝向
	dHeading float64 // 整体旋转角，归一到 [-π,π)
	cosD     float64
	sinD     float64
	// identity 表示目标位置与锚点校正前位置完全相同、目标朝向归一后与原朝向
	// 角度等价：这是只调方差的恒等校正，几何上没有任何平移或旋转。
	identity bool
}

// newCorrectionGeometry 依据提交目标与锚点帧的校正前位姿构建几何规则。
// 打开已有地图核对旧记录时，anchorBefore 必须传该记录自己保存的锚点校正前
// 快照，而不是当前轨迹上的同时间帧：之后对重叠范围再校正或追加更晚的帧，
// 都不改写旧记录，旧记录的核对只以自己的目标与快照为准。
func newCorrectionGeometry(tgt CorrectionTarget, anchorBefore Pose) correctionGeometry {
	h := normalizeAngle(tgt.Heading)
	d := normalizeAngle(h - anchorBefore.Heading)
	return correctionGeometry{
		target:   tgt,
		anchor:   anchorBefore,
		heading:  h,
		dHeading: d,
		cosD:     math.Cos(d),
		sinD:     math.Sin(d),
		// 角度等价按归一后的最短差值判断（δ 为 0），与位置完全相同一起构成
		// 恒等校正。
		identity: d == 0 && tgt.X == anchorBefore.X && tgt.Y == anchorBefore.Y,
	}
}

// apply 返回校正前位姿 before 对应的校正后位置与朝向；时间与方差由调用方
// 另行处理。把锚点自己的校正前位姿代入即准确得到目标位置与归一后的目标
// 朝向，因此锚点与后续帧共用同一条规则。
//
// 恒等校正直接保留原位置与朝向。若此时仍套用旋转平移公式，会先求
// before-anchor 的相对偏移：当轨迹横跨 ±1e308 这类大坐标时该差值先溢出为
// ±Inf、再参与浮点运算退化为 NaN，会使本应成功的校正被误判为 non_finite，
// 或使本应合法的历史记录在打开时被误判为损坏。
func (g correctionGeometry) apply(before Pose) (x, y, heading float64) {
	if g.identity {
		return before.X, before.Y, before.Heading
	}
	dx := before.X - g.anchor.X
	dy := before.Y - g.anchor.Y
	x = g.target.X + g.cosD*dx - g.sinD*dy
	y = g.target.Y + g.sinD*dx + g.cosD*dy
	heading = normalizeAngle(before.Heading + g.dHeading)
	return x, y, heading
}
