// Command importtrack 演示 posemap 的轨迹导入流程：创建地图、导入两帧
// （同时平移、转向并观测路标）、查询历史位姿与矩形区域路标，以及一次
// 因路标观测超出合并距离而被整体拒绝的导入。
//
// 用法：
//
//	go run ./examples/importtrack <你自己的本地地图文件路径>
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

	// 第一段：两帧，每帧都沿自身前方移动并各观测同一个路标 door 一次。
	// 平移一律按运动前的朝向转换到地图，观测一律按运动完成后的位姿转换。
	res, err := m.ImportSegment(posemap.Segment{
		ID: "seg-1",
		Frames: []posemap.Frame{
			// 第 0 帧：先沿运动前朝向 0（地图 +X）前进 3m 到 (3,0)，
			// 再逆时针转向 π/2；观测按运动完成后的位姿 (3,0,π/2) 转换，
			// 自身坐标 (5,0)（正前方 5m）对应地图坐标 (3,5)。
			{
				Time: 1100, DX: 3, DY: 0, DHeading: math.Pi / 2, MoveVariance: 0.02,
				Observations: []posemap.Observation{{ID: "door", X: 5, Y: 0}},
			},
			// 第 1 帧：沿当前朝向 π/2（地图 +Y）前进 3m 到 (3,3)；
			// 自身坐标 (0,0) 转换到地图坐标 (3,3)，与此前平均位置 (3,5)
			// 的距离恰好等于合并上限 2m，仍被接纳；平均位置变为 (3,4)、
			// 观测次数变为 2。
			{
				Time: 1200, DX: 3, DY: 0, DHeading: 0, MoveVariance: 0.03,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 0}},
			},
		},
	})
	if err != nil {
		log.Fatalf("seg-1 导入失败: %v", err)
	}
	fmt.Printf("seg-1 导入结果: 末位姿 time=%d x=%.2f y=%.2f heading=%.10f variance=%.2f, 路标 %v\n",
		res.EndPose.Time, res.EndPose.X, res.EndPose.Y, res.EndPose.Heading,
		res.EndPose.Variance, res.LandmarkIDs)

	p1, err := m.PoseAt(1100)
	if err != nil {
		log.Fatalf("查询 1100 帧失败: %v", err)
	}
	printPose("第 0 帧地图位姿", p1)

	cur, err := m.CurrentPose()
	if err != nil {
		log.Fatalf("查询当前位姿失败: %v", err)
	}
	printPose("末帧地图位姿", cur)

	// 历史查询选在两帧时间 1100 与 1200 之间：返回不晚于 1150 的最后
	// 一份位姿，即 1100 帧，并携带它自己的帧时间 1100。
	pmid, err := m.PoseAt(1150)
	if err != nil {
		log.Fatalf("历史位姿查询失败: %v", err)
	}
	printPose("PoseAt(1150) 返回的是 1100 帧", pmid)

	// 矩形覆盖示例路标 (3,4)：MaxX 取 3，door 的 x 正好压在边界上，
	// 边界包含所以仍会返回；区域查询只返回当前有效路标，坐标是地图坐标。
	rect := posemap.Rect{MinX: -3, MinY: 3, MaxX: 3, MaxY: 5}
	lms, err := m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("矩形查询失败: %v", err)
	}
	printLandmarks("矩形 (-3,3)-(3,5) 内当前有效路标", lms)

	// 第二段：第 0 帧合法（继续前进 1m，无观测），第 1 帧对 door 的
	// 观测转换到地图 (-3,4)，距此前平均位置 (3,4) 为 6m，超过 2m 上限，
	// 整段被拒绝，不会有任何部分生效。
	_, err = m.ImportSegment(posemap.Segment{
		ID: "seg-2",
		Frames: []posemap.Frame{
			{Time: 1300, DX: 1, DY: 0, DHeading: 0, MoveVariance: 0.01},
			{
				Time: 1400, DX: 0, DY: 0, DHeading: 0, MoveVariance: 0.01,
				Observations: []posemap.Observation{{ID: "door", X: 0, Y: 6}},
			},
		},
	})
	rj, ok := posemap.AsRejectError(err)
	if !ok {
		log.Fatalf("seg-2 应当被拒绝，实际返回 err=%v", err)
	}
	// Frame 是从零开始的帧序号；Landmark 给出涉及的路标标识。
	fmt.Printf("seg-2 被拒绝: 原因=%s 帧序号=%d 路标=%s（%v）\n",
		rj.Kind, rj.Frame, rj.Landmark, err)

	cur, err = m.CurrentPose()
	if err != nil {
		log.Fatalf("拒绝后查询当前位姿失败: %v", err)
	}
	printPose("拒绝后当前位姿（仍是 seg-1 末帧）", cur)

	lms, err = m.LandmarksInRect(rect)
	if err != nil {
		log.Fatalf("拒绝后矩形查询失败: %v", err)
	}
	// 被拒观测本会落在 (-3,4)（矩形另一头的角上），但它没有生效，
	// 查询结果仍只有此前成功提交的 door，次数仍为 2。
	printLandmarks("拒绝后矩形查询（结果不变）", lms)
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
