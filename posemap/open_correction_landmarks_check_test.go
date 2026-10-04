package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// buildCorrectionLandmarksMap 构造一份做过校正的地图：t=100 帧两次观测 L、
// t=200 帧观测 L 和 M、t=300 帧无观测；校正 c1 覆盖 t=100..300，路标
// 出现列表恰为 {L/1, M/1}（L 在范围内被三帧次/同帧多条观测涉及仍只记
// 一条）。
func buildCorrectionLandmarksMap(t *testing.T, path string) {
	t.Helper()
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}, {ID: "L"}}},
		{Time: 200, DX: 1, Observations: []Observation{{ID: "L"}, {ID: "M"}}},
		{Time: 300, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// 未篡改的文件正常打开，路标列表与提交顺序、内容保持不变。
func TestOpenCorrectionLandmarkListAccepted(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectionLandmarksMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	defer m.Close()
	recs, err := m.Corrections()
	if err != nil {
		t.Fatal(err)
	}
	if len(recs) != 1 {
		t.Fatalf("records = %+v", recs)
	}
	lms := recs[0].Landmarks
	if len(lms) != 2 || lms[0].ID != "L" || lms[0].Occurrence != 1 ||
		lms[1].ID != "M" || lms[1].Occurrence != 1 {
		t.Fatalf("landmark changes = %+v", lms)
	}
}

// 即使长度与校验和都有效，路标列表少记、多记、重复或错指出现编号，Open
// 都必须以 ErrCorrupt 拒绝整份文件、不返回可用地图且原文件不变。
func TestOpenRejectsBadCorrectionLandmarkList(t *testing.T) {
	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"missing affected occurrence", func(fd *fileData) {
			// 删掉 L/1：范围内 L 被多次观测却没有记录。
			fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[1:]
		}},
		{"missing other landmark", func(fd *fileData) {
			// 删掉 M/1。
			fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[:1]
		}},
		{"duplicate occurrence entry", func(fd *fileData) {
			// L/1 重复写两条（坐标、次数都合法也不行）。
			lcs := fd.Corrections[0].Landmarks
			fd.Corrections[0].Landmarks = append([]landmarkChangeJSON{lcs[0]}, lcs...)
		}},
		{"unknown landmark id", func(fd *fileData) {
			lcs := fd.Corrections[0].Landmarks
			extra := lcs[0]
			extra.ID = "GHOST"
			fd.Corrections[0].Landmarks = append(lcs, extra)
		}},
		{"nonexistent occurrence number", func(fd *fileData) {
			// 范围只涉及 L/1：改写成不存在的 L/3。
			fd.Corrections[0].Landmarks[0].Occurrence = 3
		}},
		{"empty list while range has observations", func(fd *fileData) {
			fd.Corrections[0].Landmarks = nil
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildCorrectionLandmarksMap(t, path)
			tamperFile(t, path, tc.fn)
			assertOpenCorrupt(t, path)
		})
	}
}

// 范围内没有任何路标观测时，空列表合法；同一份记录被写入非空列表必须拒绝。
func TestOpenCorrectionEmptyRangeList(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	// t=100 观测 L，但校正锚点取无观测的 t=200：范围 [200,200] 无观测。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L"}}},
		{Time: 200, DX: 1},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 200, Target: CorrectionTarget{X: 9, Y: 9, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 合法：空列表。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open with empty list: %v", err)
	}
	recs, _ := m2.Corrections()
	if len(recs) != 1 || len(recs[0].Landmarks) != 0 {
		t.Fatalf("landmarks = %+v, want empty", recs)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 非空列表必须拒绝，且文件保持不变。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = []landmarkChangeJSON{{
			ID: "L", Occurrence: 1,
			Before: Landmark{ID: "L", X: 2, Y: 2, Count: 1},
			After:  Landmark{ID: "L", X: 2, Y: 2, Count: 1},
		}}
	})
	assertOpenCorrupt(t, path)
}

// 校正没有改变某次出现的位置（恒等校正）：范围内有它的观测，列表仍必须
// 保留对应记录；删掉仍要拒绝。
func TestOpenIdentityCorrectionKeepsObservedOccurrence(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	m, err := Create(path, baseConfig())
	if err != nil {
		t.Fatal(err)
	}
	// t=100 位姿 (2,2)、方差 0.5；以该位姿为目标做恒等校正，范围含 L 观测。
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	rec, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 2, Y: 2, Variance: 0.5}})
	if err != nil {
		t.Fatal(err)
	}
	if len(rec.Landmarks) != 1 || rec.Landmarks[0].Before.X != rec.Landmarks[0].After.X {
		t.Fatalf("identity record = %+v", rec)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open identity correction: %v", err)
	}
	m2.Close()

	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = nil
	})
	assertOpenCorrupt(t, path)
}

// 记录结束帧之后追加的帧不属于范围：新帧观测的路标出现既不能被要求记录，
// 也不能被额外写入列表。
func TestOpenCorrectionRangeExcludesAppendedFrames(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectionLandmarksMap(t, path)

	// 校正后再导入 t=400 观测新路标 N：c1 仍只覆盖到 t=300，合法。
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 400, DX: 1, Observations: []Observation{{ID: "N"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("reopen with appended frame: %v", err)
	}
	m2.Close()

	// 把范围外才出现的 N/1 写进旧记录：必须拒绝。
	tamperFile(t, path, func(fd *fileData) {
		lcs := fd.Corrections[0].Landmarks
		extra := lcs[0]
		extra.ID = "N"
		fd.Corrections[0].Landmarks = append(lcs, extra)
	})
	assertOpenCorrupt(t, path)
}

// 同一标识失效后再现：校正范围跨越两次出现时，两次出现必须分别对应；
// 失效时刻（t=200）的观测属于旧出现，下一次出现的首次观测（t=300）属于
// 新出现。已失效的旧出现不能因当前区域查询不返回它而被排除。
func TestOpenCorrectionAcrossOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path) // t=100 L/1、t=200 L/1（当时失效）、t=300 L/2
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	// 合法：L/1 与 L/2 都在列表中。
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open: %v", err)
	}
	recs, _ := m2.Corrections()
	if len(recs) != 1 {
		t.Fatalf("records = %+v", recs)
	}
	lms := recs[0].Landmarks
	if len(lms) != 2 || lms[0].ID != "L" || lms[0].Occurrence != 1 ||
		lms[1].ID != "L" || lms[1].Occurrence != 2 {
		t.Fatalf("landmark changes = %+v", lms)
	}
	m2.Close()

	cases := []struct {
		name string
		fn   func(fd *fileData)
	}{
		{"drop invalidated old occurrence", func(fd *fileData) {
			// 只留 L/2：旧出现虽已失效，范围内（含失效端点）仍有它的观测。
			fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[1:]
		}},
		{"drop new occurrence", func(fd *fileData) {
			// 只留 L/1：漏掉新出现的首次观测。
			fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[:1]
		}},
		{"rename old to new", func(fd *fileData) {
			// L/1 改指 L/2：与原有 L/2 重复，真正的 L/1 缺失。
			fd.Corrections[0].Landmarks[0].Occurrence = 2
		}},
		{"same position does not merge occurrences", func(fd *fileData) {
			// 把 L/2 的编号改成 1：两次出现即使位置相同也分别对应。
			fd.Corrections[0].Landmarks[1].Occurrence = 1
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			p := filepath.Join(t.TempDir(), "m.pose")
			buildOccurrenceMap(t, p)
			m3, err := Open(p)
			if err != nil {
				t.Fatal(err)
			}
			if _, err := m3.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
				t.Fatal(err)
			}
			m3.Close()
			tamperFile(t, p, tc.fn)
			assertOpenCorrupt(t, p)
		})
	}
}

// 旧校正条目没有出现编号（occurrence 缺省为 0）时继续按第 1 次出现解释：
// 范围内确实观测了该路标第 1 次出现则合法；与显式 L/1 同时出现则视为重复。
func TestOpenCorrectionMissingOccurrenceNumberLegacy(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectionLandmarksMap(t, path)

	// L/1 的编号抹成 0：按第 1 次出现解释，与 M/1 一起仍恰好覆盖范围。
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks[0].Occurrence = 0
	})
	m, err := Open(path)
	if err != nil {
		t.Fatalf("occurrence 0 should mean occurrence 1: %v", err)
	}
	recs, _ := m.Corrections()
	if got := recs[0].Landmarks[0].Occurrence; got != 1 {
		t.Fatalf("loaded occurrence = %d, want 1", got)
	}
	m.Close()

	// 再补一条显式 L/1：0 与 1 指向同一出现，重复，拒绝。
	path2 := filepath.Join(t.TempDir(), "m.pose")
	buildCorrectionLandmarksMap(t, path2)
	tamperFile(t, path2, func(fd *fileData) {
		lcs := fd.Corrections[0].Landmarks
		zeroed := lcs[0]
		zeroed.Occurrence = 0
		fd.Corrections[0].Landmarks = append([]landmarkChangeJSON{zeroed}, lcs...)
	})
	assertOpenCorrupt(t, path2)
}

// 旧地图（无逐帧依据）保留既有打开规则：手工构造一条覆盖 null 旧帧的校正
// 记录，其路标列表即使与范围内观测不一致也不按新规则核对（不补造观测）。
func TestOpenSkipsLandmarkListCheckWhenRangeLacksBasis(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
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

	// 追加一帧有完整依据的新帧，然后手写一条覆盖 t=100..300（含 null 旧帧）
	// 的校正记录，故意只给空路标列表。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 0.5, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) {
		cr := corrRecJSON{ID: "span", Anchor: 100, EndTime: 300,
			Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}
		for j := 1; j < len(fd.Trajectory); j++ {
			cr.Poses = append(cr.Poses, poseChangeJSON{Before: fd.Trajectory[j], After: fd.Trajectory[j]})
		}
		fd.Corrections = append(fd.Corrections, cr)
	})
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("range lacking per-frame basis keeps legacy open rules: %v", err)
	}
	m3.Close()

	// 但在旧地图上新追加且依据完整的范围（仅 t=300）做的校正仍受校验：
	// 少记 K/1 必须拒绝。
	path2 := filepath.Join(t.TempDir(), "m.pose")
	m4, err := Create(path2, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m4.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path2, m4.state)
	m4.Close()
	m5, err := Open(path2)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m5.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 200, DX: 1, Observations: []Observation{{ID: "K"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	// 锚点取新帧 t=200：范围逐帧依据完整，K/1 在范围内被观测；用恒等目标
	// 避免与固定旧贡献的合并距离冲突。
	cur, _ := m5.PoseAt(200)
	if _, err := m5.Correct(Correction{ID: "cnew", Anchor: 200, Target: CorrectionTarget{
		X: cur.X, Y: cur.Y, Heading: cur.Heading, Variance: cur.Variance,
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m5.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path2, func(fd *fileData) {
		fd.Corrections[0].Landmarks = nil
	})
	if m6, err := Open(path2); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("fully-based appended range: err = %v, want ErrCorrupt, map=%v", err, m6)
	}
}

// 拒绝打开后原文件字节保持不变（在含校正与失效的复杂文件上再确认一次）。
func TestOpenBadCorrectionLandmarkListLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Correct(Correction{ID: "c1", Anchor: 100, Target: CorrectionTarget{X: 5, Y: 6, Variance: 1}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
	tamperFile(t, path, func(fd *fileData) {
		fd.Corrections[0].Landmarks = fd.Corrections[0].Landmarks[:1] // 丢掉 L/2
	})
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if m2, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m2)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}
