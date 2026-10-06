package posemap

import (
	"errors"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

// 打开地图时核对每个已保存段首次导入结果的末位姿：
//
//   - 末位姿时间必须逐字命中某个已导入帧（不能是初始位姿，也不能借用
//     PoseAt “不晚于指定时间”的最近帧）；
//   - 依据完整时，位置、朝向与方差必须等于该段首次成功导入时的末位姿：
//     未校正过即保存的该帧位姿；校正过即第一次覆盖该帧的校正记录中的
//     Before 快照，不是校正后值也不是当前轨迹值；
//   - 矛盾按 ErrCorrupt 拒绝且原文件保持不变，错误说明指出段标识与末位
//     姿时间；
//   - 缺少逐帧依据的旧范围只跳过数值核对，时间仍须命中真实导入帧。

// segCheckConfig 给出便于手算的配置：初始位姿 t=0 位于 (0,0)、朝向 0、
// 方差 0。
func segCheckConfig() Config {
	return Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 10.0}
}

// buildSegCheckMap 构造两段轨迹：s1 含 t=100、200 两帧（各前进 1、运动
// 方差 0.25，无观测），s2 含 t=300 一帧（再前进 1、运动方差 0.25）。
// 首次导入末位姿分别为 (2,0) 方差 0.5 与 (3,0) 方差 0.75，朝向均为 0。
func buildSegCheckMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// setSegmentEndPose 按段标识改写文件中段结果的末位姿并重算校验和。
func setSegmentEndPose(t *testing.T, path, id string, pose Pose) {
	t.Helper()
	fd := readFileData(t, path)
	for i := range fd.Segments {
		if fd.Segments[i].ID == id {
			fd.Segments[i].Result.EndPose = pose
			writeFileDataRaw(t, path, &fd)
			return
		}
	}
	t.Fatalf("segment %s not found in file", id)
}

// expectCorruptAndUntouched 打开文件必须返回包装 ErrCorrupt 的错误与 nil
// 地图，且错误说明包含段标识与末位姿时间，原文件字节保持不变。
func expectCorruptAndUntouched(t *testing.T, path, segID string, endTime int64) {
	t.Helper()
	before, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("Open err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	if !strings.Contains(err.Error(), segID) || !strings.Contains(err.Error(), "time") ||
		!strings.Contains(err.Error(), strconv.FormatInt(endTime, 10)) {
		t.Fatalf("error %q must name segment %s and end pose time %d", err.Error(), segID, endTime)
	}
	after, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(before) != string(after) {
		t.Fatal("Open modified the rejected file")
	}
}

func strconvFormatInt(v int64) string {
	return strings.TrimSpace(string(strconvAppendInt(v)))
}

// strconvAppendInt 不额外引入 strconv 别名，直接手写十进制转换。
func strconvAppendInt(v int64) []byte {
	if v == 0 {
		return []byte{'0'}
	}
	var buf [20]byte
	i := len(buf)
	neg := v < 0
	u := uint64(v)
	if neg {
		u = uint64(-v)
	}
	for u > 0 {
		i--
		buf[i] = byte('0' + u%10)
		u /= 10
	}
	if neg {
		i--
		buf[i] = '-'
	}
	return buf[i:]
}

// 合法文件打开成功：重复导入原段返回首次结果（校正前末位姿），当前位姿
// 与历史位姿查询继续返回现有校正结果，两者允许不同。
func TestOpenSegmentResultAcceptedAndReplay(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}
	res, err := m.ImportSegment(seg)
	if err != nil {
		t.Fatal(err)
	}
	if res.EndPose.X != 2 || res.EndPose.Variance != 0.5 {
		t.Fatalf("first import = %+v", res.EndPose)
	}
	// 校正锚点 t=100：两帧整体平移到 (10,10) 附近，目标方差 3。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 10, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	// 校正后继续导入新段，不改写 s1 的首次结果。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legal file: %v", err)
	}
	defer m2.Close()
	// 重复导入原段：仍是首次结果 (2,0)、方差 0.5。
	again, err := m2.ImportSegment(seg)
	if err != nil {
		t.Fatalf("duplicate import: %v", err)
	}
	if again.EndPose.X != 2 || again.EndPose.Y != 0 || again.EndPose.Heading != 0 || again.EndPose.Variance != 0.5 {
		t.Fatalf("duplicate import result = %+v, want first import (2,0) variance 0.5", again.EndPose)
	}
	// 当前位姿与历史查询返回校正后结果，与段首次结果不同。
	cur, _ := m2.CurrentPose()
	if cur.Time != 300 || cur.X != 12 || cur.Y != 10 {
		t.Fatalf("current pose = %+v, want corrected (12,10)", cur)
	}
	p200, _ := m2.PoseAt(200)
	if p200.X != 11 || p200.Y != 10 || p200.Variance != 3.25 {
		t.Fatalf("PoseAt(200) = %+v, want corrected (11,10) variance 3.25", p200)
	}
}

// 末位姿时间指向初始位姿：损坏，不能把初始位姿当作段结果。
func TestOpenSegmentEndTimeInitialPose(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegCheckMap(t, path)
	savedPose := func() Pose {
		fd := readFileData(t, path)
		for _, sg := range fd.Segments {
			if sg.ID == "s1" {
				return sg.Result.EndPose
			}
		}
		t.Fatal("s1 missing")
		return Pose{}
	}()
	savedPose.Time = 0
	setSegmentEndPose(t, path, "s1", savedPose)
	expectCorruptAndUntouched(t, path, "s1", 0)
}

// 末位姿时间不存在（既不是初始位姿也不是任何导入帧，例如落在两帧之间或
// 晚于末帧）：不能借用“不晚于该时间”的最近帧，按损坏拒绝。
func TestOpenSegmentEndTimeMissing(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegCheckMap(t, path)
	cases := []struct {
		name string
		time int64
	}{
		{"between frames", 150}, // PoseAt(150) 会返回 t=100，但不能借用
		{"after last frame", 400},
		{"just before first frame", 99},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			p := filepath.Join(t.TempDir(), "m.pose")
			buildSegCheckMap(t, p)
			fd := readFileData(t, p)
			for i := range fd.Segments {
				if fd.Segments[i].ID == "s1" {
					fd.Segments[i].Result.EndPose.Time = tc.time
				}
			}
			writeFileDataRaw(t, p, &fd)
			expectCorruptAndUntouched(t, p, "s1", tc.time)
		})
	}
}

// 未校正段的末位姿数值被篡改：位置、朝向、方差任一不符都按损坏拒绝。
// 位置容差沿用地形几何核对（coordClose），路标合并距离不能放大它。
func TestOpenSegmentEndValueMismatchNoCorrection(t *testing.T) {
	good := func() Pose { return Pose{Time: 200, X: 2, Y: 0, Heading: 0, Variance: 0.5} }
	cases := []struct {
		name string
		mut  func(Pose) Pose
	}{
		{"x", func(p Pose) Pose { p.X = 2.5; return p }},
		{"y", func(p Pose) Pose { p.Y = 0.5; return p }},
		{"heading", func(p Pose) Pose { p.Heading = 0.1; return p }},
		{"variance", func(p Pose) Pose { p.Variance = 0.75; return p }},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildSegCheckMap(t, path)
			setSegmentEndPose(t, path, "s1", tc.mut(good()))
			expectCorruptAndUntouched(t, path, "s1", 200)
		})
	}

	// 合并距离（10 米）远大于偏差也不能容忍：把 x 改成 5（距真实值 3，
	// 远小于合并距离、远大于几何容差）仍拒绝。
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegCheckMap(t, path)
	p := good()
	p.X = 5
	setSegmentEndPose(t, path, "s1", p)
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 朝向按实际方向比较：与首次朝向相差 2π 的写法物理方向相同，应当接受；
// 相差 π 则是相反方向，即使数值偏差为“整数圈”也拒绝。
func TestOpenSegmentEndHeadingDirection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegCheckMap(t, path)

	// 等价方向 2π：合法。
	fd := readFileData(t, path)
	for i := range fd.Segments {
		if fd.Segments[i].ID == "s1" {
			fd.Segments[i].Result.EndPose.Heading = 2 * math.Pi
		}
	}
	writeFileDataRaw(t, path, &fd)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("heading 2π must be accepted as same direction: %v", err)
	}
	m.Close()

	// 相反方向 π：损坏。
	fd = readFileData(t, path)
	for i := range fd.Segments {
		if fd.Segments[i].ID == "s1" {
			fd.Segments[i].Result.EndPose.Heading = math.Pi
		}
	}
	writeFileDataRaw(t, path, &fd)
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 方差必须精确一致：与首次方差只相差一个极小量也不能按几何容差放过。
func TestOpenSegmentEndVarianceExact(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildSegCheckMap(t, path)
	fd := readFileData(t, path)
	for i := range fd.Segments {
		if fd.Segments[i].ID == "s1" {
			fd.Segments[i].Result.EndPose.Variance = 0.5 + 1e-12
		}
	}
	writeFileDataRaw(t, path, &fd)
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 校正之后：段结果必须仍是首次导入末位姿（第一条覆盖记录的 Before），
// 保存成校正后值（After）或当前轨迹值都按损坏拒绝。
func TestOpenSegmentResultKeepsPreCorrectionValue(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}
	if _, err := m.ImportSegment(seg); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 10, Variance: 3}})
	if err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 校正后 t=200 位姿：(11,10)、朝向 0、方差 3.25；首次导入值 (2,0)、0.5。
	after200 := rec.Poses[1].After

	// 合法文件先能打开（保存的段结果仍是首次值）。
	ok, err := Open(path)
	if err != nil {
		t.Fatalf("legal corrected file: %v", err)
	}
	again, err := ok.ImportSegment(seg)
	if err != nil || again.EndPose.X != 2 || again.EndPose.Variance != 0.5 {
		t.Fatalf("duplicate import = %+v err=%v, want first result (2,0) v0.5", again.EndPose, err)
	}
	ok.Close()

	// 把段结果改成校正后值：不能用当前位姿替换原结果，拒绝。
	setSegmentEndPose(t, path, "s1", after200)
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 只校正方差（恒等几何）：位置朝向不变也必须区分原始方差与目标方差，
// 段结果保存成校正后方差即损坏。
func TestOpenSegmentResultVarianceOnlyCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	seg := Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}
	if _, err := m.ImportSegment(seg); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 1, Y: 0, Heading: 0, Variance: 9}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 段结果方差被改成校正后的 9.25（位置朝向不变）：仍按损坏拒绝。
	setSegmentEndPose(t, path, "s1", Pose{Time: 200, X: 2, Y: 0, Heading: 0, Variance: 9.25})
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 同一帧被多次校正：旧段结果以“第一次”覆盖该帧的记录 Before 为准，第
// 二次校正的 Before（即第一次校正后值）不合法。
func TestOpenSegmentResultMultipleCorrectionsSameFrame(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	first, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 10, Variance: 3}})
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c2", Anchor: 200,
		Target: CorrectionTarget{X: 50, Y: 60, Variance: 7}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 第二次校正前的 t=200 位姿即第一次校正后的 After，不能作为段结果。
	wrong := first.Poses[1].After // (11,10) 方差 3.25
	setSegmentEndPose(t, path, "s1", wrong)
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 校正范围只覆盖旧段的一部分：锚点落在段中间（t=200），t=100 不移动，
// 末帧 t=300 随锚点一起移动。旧段的首次结果不变——期望值取第一条覆盖
// 记录 c1 中 t=300 的 Before（即首次导入值），不是校正后的 After。
func TestOpenSegmentResultPartialCorrectionRange(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
		{Time: 300, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	// 锚点 t=200：记录只覆盖 200..300，t=100 不在校正范围内、保持 (1,0)。
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 200,
		Target: CorrectionTarget{X: 10, Y: 0, Variance: 3}})
	if err != nil {
		t.Fatal(err)
	}
	if len(rec.Poses) != 2 {
		t.Fatalf("record poses = %d, want 2 (200..300)", len(rec.Poses))
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 合法文件：s1 末帧 300 的首次结果是 (3,0) 方差 0.75；t=100 未移动，
	// t=300 当前为校正后的 (11,0) 方差 3.25。
	ok, err := Open(path)
	if err != nil {
		t.Fatalf("legal partial-correction file: %v", err)
	}
	if p, _ := ok.PoseAt(100); p.X != 1 || p.Variance != 0.25 {
		t.Fatalf("PoseAt(100) = %+v, want untouched (1,0) v0.25", p)
	}
	if p, _ := ok.PoseAt(300); p.X != 11 || p.Variance != 3.25 {
		t.Fatalf("PoseAt(300) = %+v, want corrected (11,0) v3.25", p)
	}
	ok.Close()

	// 把段结果改成校正后的 t=300 值：必须按损坏拒绝，不能用当前值替换。
	setSegmentEndPose(t, path, "s1", Pose{Time: 300, X: 11, Y: 0, Variance: 3.25})
	expectCorruptAndUntouched(t, path, "s1", 300)
}

// 校正之后继续导入新段：旧段结果不被新段或校正改写；新段的结果则必须
// 符合它自己的首次导入。
func TestOpenSegmentResultNewSegmentAfterCorrection(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 10, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	// 新段在校正后的轨迹上继续：t=300 自 (11,10) 再前进 1 → (12,10)，
	// 方差 3.25+0.25 = 3.5。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	// 合法文件可以打开。
	ok, err := Open(path)
	if err != nil {
		t.Fatalf("legal file: %v", err)
	}
	ok.Close()

	// s1 的结果改成当前 t=200 值（11,10,3.25）：损坏（必须保持 (2,0,0.5)）。
	setSegmentEndPose(t, path, "s1", Pose{Time: 200, X: 11, Y: 10, Variance: 3.25})
	expectCorruptAndUntouched(t, path, "s1", 200)
}

// 校正范围没有包含后来新段的末帧时，不改变该段结果对应的期望值：校正
// 提交时末帧只有 t=100（记录只覆盖 t=100），其后导入的 s2 末帧 t=200
// 不属于该记录，段结果就是在新轨迹上的首次导入值 (11,0) 方差 3.25。
func TestOpenSegmentResultCorrectionNotCoveringEnd(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	// 提交时末帧即 100，c1 只覆盖 t=100。
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100,
		Target: CorrectionTarget{X: 10, Y: 0, Variance: 3}}); err != nil {
		t.Fatal(err)
	}
	// 校正后追加 s2：t=200 自 (10,0) 前进 1 → (11,0)，方差 3.25。
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("legal: correction not covering later segment end: %v", err)
	}
	m2.Close()
	// s2 的末帧 200 不在 c1 范围内，期望值是首次导入值 (11,0) v3.25；
	// 篡改成 c1 锚点校正后坐标 (10,0) v3 必须拒绝。
	setSegmentEndPose(t, path, "s2", Pose{Time: 200, X: 10, Y: 0, Variance: 3})
	expectCorruptAndUntouched(t, path, "s2", 200)
}

// 纯旧文件（无逐帧依据）：时间规则仍适用——末位姿时间不命中真实导入帧
// 时拒绝；时间合法时维持原有打开方式，不核对数值。
func TestOpenSegmentResultLegacyFile(t *testing.T) {
	cfg := segCheckConfig()

	// 时间不成立：即使是旧文件也拒绝。
	bad := filepath.Join(t.TempDir(), "bad.pose")
	m, err := Create(bad, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, bad, m.state)
	m.Close()
	fd := readFileData(t, bad)
	for i := range fd.Segments {
		if fd.Segments[i].ID == "s1" {
			fd.Segments[i].Result.EndPose.Time = 150
		}
	}
	writeFileDataRaw(t, bad, &fd)
	if m, err := Open(bad); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("legacy bad end time: err = %v map = %v, want ErrCorrupt nil", err, m)
	}

	// 时间合法但数值与历史轨迹不符：缺少逐帧依据，保留原打开方式。
	good := filepath.Join(t.TempDir(), "good.pose")
	m2, err := Create(good, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, good, m2.state)
	m2.Close()
	fd2 := readFileData(t, good)
	if len(fd2.Sources) != 0 {
		t.Fatalf("test setup wrong: sources = %+v", fd2.Sources)
	}
	for i := range fd2.Segments {
		if fd2.Segments[i].ID == "s1" {
			fd2.Segments[i].Result.EndPose.X = 99
			fd2.Segments[i].Result.EndPose.Variance = 42
		}
	}
	writeFileDataRaw(t, good, &fd2)
	m3, err := Open(good)
	if err != nil {
		t.Fatalf("legacy file keeps existing open behavior for unverifiable values: %v", err)
	}
	// 重复导入仍返回文件中保存的那份结果。
	res, err := m3.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, MoveVariance: 0.25},
		{Time: 200, DX: 1, MoveVariance: 0.25},
	}})
	if err != nil || res.EndPose.X != 99 {
		t.Fatalf("legacy duplicate import = %+v err=%v, want saved result x=99", res.EndPose, err)
	}
	m3.Close()
}

// 旧地图上追加依据完整的新段：兼容只跳过旧范围无法确认的数值核对，新
// 段的末位姿仍须与首次导入一致，时间也须命中真实帧。
func TestOpenSegmentResultMixedLegacyNewSegment(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, segCheckConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	// t=300 自 (2,0) 前进 0.5 → (2.5,0)；旧两帧运动方差为 0，t=200 保存
	// 方差为 0，新帧运动方差 0.5 累加后末帧方差为 0.5。
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, MoveVariance: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	// 合法混合文件可打开。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("legal mixed file: %v", err)
	}
	m3.Close()

	// 新段结果位置被篡改：依据完整，拒绝。
	setSegmentEndPose(t, path, "new", Pose{Time: 300, X: 7, Y: 0, Variance: 0.5})
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("new fully-based segment mismatch: err = %v map = %v", err, m)
	}

	// 新段结果时间不命中真实帧：同样拒绝。
	fd := readFileData(t, path)
	for i := range fd.Segments {
		if fd.Segments[i].ID == "new" {
			fd.Segments[i].Result.EndPose = Pose{Time: 250, X: 2.5, Variance: 0.5}
		}
	}
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("new segment bad end time: err = %v map = %v", err, m)
	}
}
