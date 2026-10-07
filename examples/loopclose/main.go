// Command loopclose 演示 posemap 的回环校正流程：创建地图、导入一段三帧
// 轨迹（锚点前后都观测同一个路标，锚点之后仍有实际位移），提交一次只覆盖
// 后半段轨迹的成功校正（位置与朝向同时改变），再提交一次会让路标观测超出
// 合并距离而被整体拒绝的校正，并展示拒绝后位姿、路标与已有校正记录都保持
// 成功校正后的结果。
//
// 用法：
//
//	go run ./examples/loopclose <你自己的本地地图文件路径>
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

	// 位置单位为米，时间为整数毫秒，朝向为弧度。
	m, err := posemap.Create(os.Args[1], posemap.Config{
		InitialTime:     1000, // 初始时间（毫秒）
		InitialX:        0,    // 初始位置 (0,0)（米，地图坐标）
		InitialY:        0,
		InitialHeading:  0,    // 初始朝向 0（弧度，地图 +X 方向）
		InitialVariance: 0.01, // 初始位置方差
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

	// 基础轨迹：三帧，每帧都沿自身前方移动，并各观测同一个路标 door 一次。
	// 平移一律按运动前的朝向转换到地图，观测一律按运动完成后的位姿转换。
	// 这是校正前的准备步骤，失败属于程序错误，直接终止。
	res, err := m.ImportSegment(posemap.Segment{
		ID: "seg-1",
		Frames: []posemap.Frame{
			// 第 0 帧：沿运动前朝向 0（地图 +X）前进 3m 到 (3,0)，不转向；
			// 观测按运动完成后的位姿 (3,0,0) 转换，自身坐标 (5,0) 对应
			// 地图坐标 (8,0)。路标首次出现，平均位置 (8,0)、次数 1。
			{
				Time: 1100, DX: 3, DY: 0, DHeading: 0, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: 0}},
			},
			// 第 1 帧（后来的校正锚点）：沿运动前朝向 0 前进 3m 到 (6,0)，
			// 再逆时针转向 π/2；观测按运动完成后的位姿 (6,0,π/2) 转换，
			// 自身坐标 (1,-1) 对应地图坐标 (7,1)，与平均位置 (8,0) 相距
			// √2m，在 2m 上限内；平均位置变为 (7.5,0.5)、次数 2。
			{
				Time: 1200, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.03,
				Observations: []posemap.Observation{{ID: "door", X: 1, Y: -1}},
			},
			// 第 2 帧：沿运动前朝向 π/2（地图 +Y）前进 2m 到 (6,2)，不转向；
			// 自身坐标 (-1,0) 对应地图坐标 (6,1)，与平均位置 (7.5,0.5)
			// 相距 √2.5m，仍在上限内；平均位置变为 (7,0.67)、次数 3。
			{
				Time: 1300, DX: 2, DY: 0, DHeading: 0, MoveVariance: 0.04,
				Observations: []posemap.Observation{{ID: "door", X: -1, Y: 0}},
			},
		},
	})
	if err != nil {
		log.Fatalf("seg-1 导入失败，示例终止: %v", err)
	}
	fmt.Printf("seg-1 导入结果: 末位姿 time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f, 路标 %v\n",
		res.EndPose.Time, res.EndPose.X, res.EndPose.Y, res.EndPose.Heading,
		res.EndPose.Variance, res.LandmarkIDs)

	// 校正前的完整状态：三帧位姿与路标平均位置。
	p1100, err := m.PoseAt(1100)
	if err != nil {
		log.Fatalf("查询 1100 帧失败: %v", err)
	}
	printPose("校正前 1100 帧（锚点之前，校正后保持不变）", p1100)

	p1200, err := m.PoseAt(1200)
	if err != nil {
		log.Fatalf("查询 1200 帧失败: %v", err)
	}
	printPose("校正前锚点 1200 帧", p1200)

	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("查询当前位姿失败: %v", err)
	}
	printPose("校正前末帧 1300", cur)

	// 矩形覆盖 door 校正前后的位置 (7,0.67) 与 (7.71,0.90)。
	rect := posemap.Rect{MinX: 5, MinY: -1, MaxX: 9, MaxY: 2}
	lms, err := m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("矩形查询失败: %v", err)
	}
	printLandmarks("校正前矩形 (5,-1)-(9,2) 内路标", lms)

	// 第一次校正：锚点必须准确命中已导入帧的时间（这里是 1200，不能是
	// 初始位姿 1000，也不能是两帧之间的查询时间）。目标同时改变位置与
	// 朝向：锚点从 (6,0,π/2) 重定位到 (6.5,1,π/4)，方差重置为 0.05。
	// 锚点至末帧整体平移旋转、保持相对位姿；锚点之前的 1100 帧不变。
	rec, err := m.Correct(posemap.Correction{
		ID:     "loop-1",
		Anchor: 1200,
		Target: posemap.CorrectionTarget{X: 6.5, Y: 1, Heading: math.Pi / 4, Variance: 0.05},
	})
	if err != nil {
		log.Fatalf("loop-1 校正应当成功，实际返回 err=%v", err)
	}
	fmt.Printf("校正 loop-1 已接受: 锚点=%d 目标 x=%.2f y=%.2f heading=%.10f variance=%.2f, 受影响末帧=%d\n",
		rec.Anchor, rec.Target.X, rec.Target.Y, rec.Target.Heading, rec.Target.Variance, rec.EndTime)
	for _, pc := range rec.Poses {
		fmt.Printf("  位姿 %d: 前 x=%.2f y=%.2f heading=%.10f variance=%.2f → 后 x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
			pc.Before.Time, pc.Before.X, pc.Before.Y, pc.Before.Heading, pc.Before.Variance,
			pc.After.X, pc.After.Y, pc.After.Heading, pc.After.Variance)
	}
	for _, lc := range rec.Landmarks {
		fmt.Printf("  路标 %s 第 %d 次出现: 前 x=%.2f y=%.2f → 后 x=%.2f y=%.2f, 次数 %d→%d\n",
			lc.ID, lc.Occurrence, lc.Before.X, lc.Before.Y, lc.After.X, lc.After.Y,
			lc.Before.Count, lc.After.Count)
	}

	// 校正后的当前查询值：1100 帧不变；锚点采用目标位姿；1300 帧保持与
	// 锚点的相对关系（校正前相对锚点的偏移 (0,2) 随锚点旋转 -π/4 后变为
	// (√2,√2)，朝向同为 π/4）；方差从目标方差 0.05 接续 1300 帧的运动
	// 方差 0.04，得到 0.09。
	p1100, err = m.PoseAt(1100)
	if err != nil {
		log.Fatalf("校正后查询 1100 帧失败: %v", err)
	}
	printPose("校正后 1100 帧（不变）", p1100)

	p1200, err = m.PoseAt(1200)
	if err != nil {
		log.Fatalf("校正后查询 1200 帧失败: %v", err)
	}
	printPose("校正后锚点 1200 帧（采用目标位姿）", p1200)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("校正后查询当前位姿失败: %v", err)
	}
	printPose("校正后末帧 1300（保持与锚点的相对关系）", cur)

	// 历史查询选在两帧时间 1200 与 1300 之间：返回不晚于 1250 的最后
	// 一份位姿，即校正后的 1200 帧，并携带它自己的帧时间 1200。注意
	// 1250 只是查询参数，不能作为校正锚点——锚点必须准确命中已导入帧，
	// 用 1250 提交校正会被以 anchor_not_found 拒绝。
	pmid, err := m.PoseAt(1250)
	if err != nil {
		log.Fatalf("历史位姿查询失败: %v", err)
	}
	printPose("PoseAt(1250) 返回的是 1200 帧", pmid)

	// 路标的早期观测（1100 帧，地图位置 (8,0)）保持不动，受影响帧的
	// 观测按校正后位姿重新参与平均：平均位置从 (7,0.67) 变为 (7.71,0.90)，
	// 观测次数仍为 3，不会因为重放而增加。
	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("校正后矩形查询失败: %v", err)
	}
	printLandmarks("校正后矩形 (5,-1)-(9,2) 内路标", lms)

	// 第二次校正：把锚点沿地图 +Y 平移 4m。重放后 1200 帧对 door 的
	// 观测会落在 (7.91,5)，与固定的早期观测 (8,0) 相距约 5m，超过 2m
	// 合并上限，整次校正被拒绝。这是业务拒绝，不是程序异常：用
	// AsRejectError 取出可区分的原因、冲突观测所在帧的实际时间、路标
	// 标识与出现编号，程序继续运行。
	_, err = m.Correct(posemap.Correction{
		ID:     "loop-2",
		Anchor: 1200,
		Target: posemap.CorrectionTarget{X: 6.5, Y: 5, Heading: math.Pi / 4, Variance: 0.05},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("loop-2 应当被业务拒绝，实际返回 err=%v", err)
	}
	fmt.Printf("loop-2 被拒绝: 原因=%s 冲突帧时间=%d 路标=%s 出现编号=%d（%v）\n",
		rj.Kind, rj.Time, rj.Landmark, rj.Occurrence, err)

	// 拒绝不留部分更新：位姿、路标位置与已有校正记录都保持 loop-1
	// 成功校正后的结果。
	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 loop-1 校正后的末帧）", cur)

	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("拒绝后矩形查询失败: %v", err)
	}
	printLandmarks("拒绝后矩形查询（结果不变）", lms)

	recs, err := m.Corrections()
	if err != nil {
		log.Fatalf("查询校正记录失败: %v", err)
	}
	fmt.Printf("已有校正记录: %d 条\n", len(recs))
	for _, r := range recs {
		fmt.Printf("  记录 %s: 锚点=%d 受影响末帧=%d\n", r.ID, r.Anchor, r.EndTime)
	}
	// 记录中的校正前后值是历史快照：即使之后又有别的校正或拒绝，loop-1
	// 记录里锚点的 Before 仍是最初导入时的 (6,0,π/2)，与当前查询值
	// (6.5,1,π/4) 区分开。
	first := recs[0].Poses[0]
	fmt.Printf("记录中 loop-1 锚点校正前快照（历史值，不被后续操作改写）: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		first.Before.Time, first.Before.X, first.Before.Y, first.Before.Heading, first.Before.Variance)
}

func printPose(label string, p posemap.Pose) {
	fmt.Printf("%s: time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f\n",
		label, p.Time, p.X, p.Y, p.Heading, p.Variance)
}

func printLandmarks(label string, lms []posemap.Landmark) {
	fmt.Printf("%s: %d 个\n", label, len(lms))
	for _, lm := range lms {
		fmt.Printf("  id=%s x=%.2f y=%.2f count=%d\n", lm.ID, lm.X, lm.Y, lm.Count)
	}
}
