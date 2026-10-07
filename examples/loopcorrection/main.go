// Command loopcorrection 演示 posemap 的已确认回环校正：创建地图、导入
// 一段四帧轨迹（同一个路标 door 在锚点之前与锚点之后都有观测，锚点
// 之后还保留一帧有实际位移），先演示一次锚点没有命中已导入帧的拒绝，再
// 提交一次同时改变位置与朝向的成功校正，最后提交一次会让 door 的观测
// 超过合并距离的校正并确认它只是业务拒绝、状态保持成功校正后的结果。
//
// 用法：
//
//	go run ./examples/loopcorrection <你自己的本地地图文件路径>
//
// 指定的文件必须尚不存在：posemap.Create 不会覆盖或删除已有文件，
// 创建失败时程序直接终止，本次示例不再继续。
package main

import (
	"fmt"
	"log"
	"math"
	"os"

	"github.com/bengzyyys/pose-map/posemap"
)

func main() {
	if len(os.Args) != 2 {
		fmt.Fprintf(os.Stderr, "用法: %s <地图文件路径（必须尚不存在）>\n", os.Args[0])
		os.Exit(2)
	}

	// 位置单位为米，时间为整数毫秒，朝向为弧度，位置方差为米²。
	m, err := posemap.Create(os.Args[1], posemap.Config{
		InitialTime:     2000, // 初始时间（毫秒）
		InitialX:        -2,   // 初始位置 (-2,0)（米，地图坐标）
		InitialY:        0,
		InitialHeading:  0,    // 初始朝向 0（弧度，地图 +X 方向）
		InitialVariance: 0.01, // 初始位置方差（米²）
		MaxInterval:     1000, // 相邻帧时间间隔上限（毫秒）
		MergeDistance:   2,    // 同标识路标观测合并距离上限（米）
	})
	if err != nil {
		// 路径上已有文件时会走到这里；不要为了运行示例去删除或覆盖它。
		log.Fatalf("创建地图失败，示例终止: %v", err)
	}
	defer func() {
		if err := m.Close(); err != nil {
			log.Fatalf("关闭地图失败: %v", err)
		}
	}()

	// 四帧轨迹（运动学与导入示例相同：平移按运动前朝向、观测按运动完成后
	// 位姿转换）。door 在锚点之前与锚点之后各被观测一次（锚点帧本身没有
	// 观测），两次观测在地图上相距 √2 m，落在 2m 合并上限内，属于同一次
	// 出现（出现编号 1）。
	//
	//	t=2100 位姿 (0,-2)  h=0     观测 door → 地图 (5,-6)
	//	t=2200 位姿 (0,0)   h=π/2   （无观测）                  ← 校正锚点
	//	t=2300 位姿 (4,-5)  h=-π    观测 door → 地图 (4,-5)
	//	t=2400 位姿 (6,-5)  h=-π/2  （前进 2m，无观测，有实际位移）
	//
	// 两次观测平均位置 (4.5,-5.5)，次数 2。
	_, err = m.ImportSegment(posemap.Segment{
		ID: "seg-loop",
		Frames: []posemap.Frame{
			// 第 0 帧：沿朝向 0 移动 (2,-2) 到 (0,-2)，不转向；
			// door 的自身坐标 (5,-4) 按完成后位姿转换到地图 (5,-6)。
			{
				Time: 2100, DX: 2, DY: -2, DHeading: 0, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: -4}},
			},
			// 第 1 帧：沿朝向 0 向自身 +Y 移动 2m 到 (0,0)，再逆时针
			// 转向 π/2；本帧没有路标观测。这一帧是后面的校正锚点。
			{Time: 2200, DX: 0, DY: 2, DHeading: math.Pi / 2, MoveVariance: 0.03},
			// 第 2 帧：沿朝向 π/2 的自身坐标移动 (-5,-4)（地图 +X 4m、
			// -Y 5m）到 (4,-5)，再转向 π（归一后为 -π）；door 恰好在
			// 机器人位置，自身 (0,0) → 地图 (4,-5)，与上一条观测 (5,-6)
			// 相距 √2 m，接纳。
			{
				Time: 2300, DX: -5, DY: -4, DHeading: math.Pi / 2, MoveVariance: 0.04,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 0}},
			},
			// 第 3 帧：沿朝向 π 前进 2m（地图 +X）到 (6,-5)，再转向
			// -π/2；没有路标观测，但锚点之后保留一帧有实际位移的轨迹。
			{Time: 2400, DX: -2, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.05},
		},
	})
	if err != nil {
		log.Fatalf("seg-loop 导入失败，示例终止: %v", err)
	}

	// ---- 校正前：锚点帧、后续帧与 door 的当前查询值 ----
	p2200, err := m.PoseAt(2200)
	if err != nil {
		log.Fatalf("查询锚点帧失败: %v", err)
	}
	printPose("校正前 t=2200 锚点帧", p2200)
	p2300, err := m.PoseAt(2300)
	if err != nil {
		log.Fatalf("查询 t=2300 帧失败: %v", err)
	}
	printPose("校正前 t=2300 后续帧", p2300)
	p2400, err := m.PoseAt(2400)
	if err != nil {
		log.Fatalf("查询 t=2400 末帧失败: %v", err)
	}
	printPose("校正前 t=2400 末帧", p2400)
	printDoor(m, "校正前 door（2 次观测平均，次数 2）")

	// 历史查询选在两帧之间：返回的仍是实际帧时间。2150 位于 2100 与
	// 2200 之间，PoseAt 返回不晚于它的最后一份位姿，即 2100 帧。
	pmid, err := m.PoseAt(2150)
	if err != nil {
		log.Fatalf("两帧之间历史查询失败: %v", err)
	}
	printPose("PoseAt(2150) 返回的是 2100 帧（注意 Time 是实际帧时间）", pmid)

	// ---- 第一次提交：锚点给在两帧之间，必须被拒绝 ----
	// 2250 没有对应任何已导入帧；校正不会借用“最近一帧”2200。
	_, err = m.Correct(posemap.Correction{
		ID:     "corr-miss",
		Anchor: 2250,
		Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("锚点 2250 应当被拒绝，实际返回 err=%v", err)
	}
	fmt.Printf("锚点未命中被拒绝: 原因=%s 提交锚点时间=%d hasTime=%v（%v）\n",
		rj.Kind, rj.Time, rj.HasTime, err)
	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿不变", cur)

	// ---- 第二次提交：锚点准确命中 t=2200，成功校正 ----
	// 目标让锚点从 (0,0,π/2) 落到 (10,0,0)：位置平移 10m、朝向顺时针
	// 旋转 π/2，成功后位置与朝向同时改变。锚点至末帧作为同一次刚体重
	// 定位：锚点之前 t=2100 的观测 (5,-6) 固定不动，它距这次旋转的旋转
	// 中心 (5,-5) 只有 1m；锚点之后 t=2300 的观测 (4,-5) 转到 (5,-4)，
	// 与固定观测仍相距 √2 m，重放成功，新平均为 (5,-5)。目标方差 0.02
	// （米²）从锚点接续，之后每帧累加锚点之后的原运动方差。
	rec, err := m.Correct(posemap.Correction{
		ID:     "corr-loop",
		Anchor: 2200,
		Target: posemap.CorrectionTarget{X: 10, Y: 0, Heading: 0, Variance: 0.02},
	})
	if err != nil {
		log.Fatalf("成功校正意外失败: %v", err)
	}
	fmt.Printf("校正 corr-loop 成功: 锚点=%d 受影响末帧=%d 受影响位姿 %d 份、路标出现 %d 个\n",
		rec.Anchor, rec.EndTime, len(rec.Poses), len(rec.Landmarks))

	// 锚点之前的位姿完全不变；锚点采用目标位姿；后续帧保持与锚点的相对
	// 关系整体刚体重定位。
	p2100, err := m.PoseAt(2100)
	if err != nil {
		log.Fatalf("校正后查询 2100 帧失败: %v", err)
	}
	printPose("校正后 t=2100（锚点之前，保持不变）", p2100)
	p2200, err = m.PoseAt(2200)
	if err != nil {
		log.Fatalf("校正后查询锚点帧失败: %v", err)
	}
	printPose("校正后 t=2200 锚点帧（即提交目标）", p2200)
	p2300, err = m.PoseAt(2300)
	if err != nil {
		log.Fatalf("校正后查询 2300 帧失败: %v", err)
	}
	printPose("校正后 t=2300 后续帧", p2300)
	p2400, err = m.PoseAt(2400)
	if err != nil {
		log.Fatalf("校正后查询 2400 帧失败: %v", err)
	}
	printPose("校正后 t=2400 末帧", p2400)
	// 两帧之间的查询仍返回实际帧时间，且拿到的是校正后的 2200 帧。
	pmid, err = m.PoseAt(2250)
	if err != nil {
		log.Fatalf("校正后两帧之间查询失败: %v", err)
	}
	printPose("PoseAt(2250) 返回的是校正后的 2200 帧", pmid)
	printDoor(m, "校正后 door（早期观测固定，受影响观测按新位姿重新平均）")

	// ---- 第三次提交：同样锚定 t=2200，但平移到 x=30，观测必然超距 ----
	// 锚点帧本身没有观测；重放时锚点之前 t=2100 的观测固定在 (5,-6)，
	// t=2300 的观测随这次校正变到 (25,-4)，与固定观测相距 √404 ≈
	// 20.1m，超过 2m 上限。第一条冲突观测在 t=2300（不是锚点时间），
	// 整次拒绝、不留部分更新。
	_, err = m.Correct(posemap.Correction{
		ID:     "corr-far",
		Anchor: 2200,
		Target: posemap.CorrectionTarget{X: 30, Y: 0, Heading: 0, Variance: 0.02},
	})
	rj, ok = posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("corr-far 应当被拒绝，实际返回 err=%v", err)
	}
	// 业务拒绝不是程序异常：打印原因、冲突观测所在的实际帧时间、路标
	// 标识与出现编号，然后继续用查询证明状态没有被改动。
	fmt.Printf("超距校正被拒绝: 原因=%s 冲突帧时间=%d 路标=%s 出现编号=%d hasOccurrence=%v（%v）\n",
		rj.Kind, rj.Time, rj.Landmark, rj.Occurrence, rj.HasOccurrence, err)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 corr-loop 的末帧结果）", cur)
	printDoor(m, "拒绝后 door（位置与次数不变）")

	// 已有校正记录保持成功校正后的结果：被拒绝的 corr-far 没有产生记录，
	// 记录里保存的是提交时的历史快照（校正前后值），不会随当前状态改写。
	recs, err := m.Corrections()
	if err != nil {
		log.Fatalf("查询校正记录失败: %v", err)
	}
	fmt.Printf("校正记录共 %d 条（被业务拒绝的提交不留记录）:\n", len(recs))
	for _, r := range recs {
		fmt.Printf("  记录 id=%s 锚点=%d 受影响末帧=%d 提交目标={x=%.2f y=%.2f heading=%.10f variance=%.2f}\n",
			r.ID, r.Anchor, r.EndTime, r.Target.X, r.Target.Y, r.Target.Heading, r.Target.Variance)
		for _, pc := range r.Poses {
			fmt.Printf("    位姿 time=%d 校正前 (%.2f,%.2f,h=%.10f,v=%.2f) → 快照校正后 (%.2f,%.2f,h=%.10f,v=%.2f)\n",
				pc.Before.Time, pc.Before.X, pc.Before.Y, pc.Before.Heading, pc.Before.Variance,
				pc.After.X, pc.After.Y, pc.After.Heading, pc.After.Variance)
		}
		for _, lc := range r.Landmarks {
			fmt.Printf("    路标 %s 出现 %d 校正前 (%.4f,%.4f,count=%d) → 快照校正后 (%.4f,%.4f,count=%d)\n",
				lc.ID, lc.Occurrence, lc.Before.X, lc.Before.Y, lc.Before.Count,
				lc.After.X, lc.After.Y, lc.After.Count)
		}
	}
}

// printDoor 用一个足以覆盖本示例的大矩形取出 door，打印的是当前查询值
// （地图坐标的平均位置与观测次数），区别于校正记录里保存的历史快照。
func printDoor(m *posemap.Map, label string) {
	lms, err := m.LandmarksInRect(posemap.Rect{MinX: -100, MinY: -100, MaxX: 100, MaxY: 100})
	if err != nil {
		log.Fatalf("查询 door 失败: %v", err)
	}
	for _, lm := range lms {
		if lm.ID == "door" {
			fmt.Printf("%s: x=%.4f y=%.4f count=%d\n", label, lm.X, lm.Y, lm.Count)
			return
		}
	}
	log.Fatalf("查询范围内没有 door")
}

func printPose(label string, p posemap.Pose) {
	fmt.Printf("%s: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		label, p.Time, p.X, p.Y, p.Heading, p.Variance)
}
