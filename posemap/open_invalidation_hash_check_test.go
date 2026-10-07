package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// 失效记录保存的集合指纹必须就是对记录实际列出的路标标识集合计算的指纹。
// 任务描述的基准情形：操作 A 同时撤下 L、M 的第 1 次出现，L 之后在新轨迹
// 中再现，A 保存的指纹却被换成仅含 L 的集合的指纹——失效结果指向、失效
// 时间、原因与操作标识的交叉关系全部正常，也必须按 ErrCorrupt 拒绝整份
// 文件，不返回可用地图对象，错误说明中能辨认出操作标识 A。
func TestOpenRejectsInvalidationHashOfDifferentSet(t *testing.T) {
	cases := []struct {
		name string
		hash string
	}{
		// 指纹对应仅含 L 的集合：记录列出的是 {L, M}。
		{"hash of subset", landmarkSetHash([]string{"L"})},
		// 指纹对应另一组路标（含记录中不存在的标识）。
		{"hash of superset", landmarkSetHash([]string{"L", "M", "N"})},
		// 指纹对应次序不同的同一集合时本应合法；这里对应的是完全不同的集合。
		{"hash of disjoint set", landmarkSetHash([]string{"ZZZ"})},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildInvalidationMap(t, path)
			fd := readFileData(t, path)
			fd.Invalidations[0].Hash = tc.hash
			writeFileDataRaw(t, path, &fd)

			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt", err)
			}
			if m != nil {
				t.Fatalf("Open returned usable map: %v", m)
			}
			if !strings.Contains(err.Error(), "A") {
				t.Fatalf("error does not identify the invalidation operation: %v", err)
			}
		})
	}
}

// 拒绝打开指纹矛盾的文件时，原文件必须保持原样：不能通过替换指纹、删除
// 操作或调整失效结果让矛盾文件继续打开。
func TestOpenInvalidationHashMismatchLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildInvalidationMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Hash = landmarkSetHash([]string{"L"})
	writeFileDataRaw(t, path, &fd)
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 多条失效记录中只要有一条指纹与其列出的集合不符，即使其他记录完全合法，
// 也拒绝整份文件。
func TestOpenRejectsSingleBadHashAmongMany(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildTwoInvalidationsMap(t, path)
	fd := readFileData(t, path)
	if len(fd.Invalidations) != 2 {
		t.Fatalf("invalidations = %+v", fd.Invalidations)
	}
	// A 的记录保持合法；B（撤下 L 第 2 次出现）的指纹被换成 {L, M} 的。
	fd.Invalidations[1].Hash = landmarkSetHash([]string{"L", "M"})
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	if !strings.Contains(err.Error(), "B") {
		t.Fatalf("error does not identify the invalidation operation: %v", err)
	}
}

// 指纹与列出的集合一致（仅保存次序不同不影响集合语义）的文件正常打开；
// 重新打开后以原原因提交 A，无论请求中 L、M 次序怎样排列，都仍返回首次
// 撤下的时间与两个路标的第 1 次出现，不撤下后来有效的 L。
func TestOpenInvalidationHashConsistentFileBehaves(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildInvalidationMap(t, path)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open valid file: %v", err)
	}
	defer m.Close()

	again, err := m.Invalidate(Invalidation{ID: "A", Reason: "gone", Landmarks: []string{"M", "L"}})
	if err != nil {
		t.Fatalf("resubmit A reversed: %v", err)
	}
	if again.Time != 200 || len(again.Landmarks) != 2 ||
		again.Landmarks[0] != (InvalidatedLandmark{ID: "L", Occurrence: 1}) ||
		again.Landmarks[1] != (InvalidatedLandmark{ID: "M", Occurrence: 1}) {
		t.Fatalf("resubmit result = %+v", again)
	}
	hL, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(hL.Appearances) != 2 || !hL.Appearances[1].Active {
		t.Fatalf("resubmit re-invalidated the new occurrence: %+v", hL.Appearances)
	}

	// 同一标识换原因或换路标集合仍按 invalidation_mismatch 拒绝。
	for _, req := range []Invalidation{
		{ID: "A", Reason: "moved", Landmarks: []string{"L", "M"}},
		{ID: "A", Reason: "gone", Landmarks: []string{"L"}},
		{ID: "A", Reason: "gone", Landmarks: []string{"L", "M", "N"}},
	} {
		_, err := m.Invalidate(req)
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectInvalidationMismatch {
			t.Fatalf("mismatch request %+v: err = %v, want invalidation_mismatch", req, err)
		}
	}
}

// 旧地图后来追加的失效记录同样要满足指纹一致性：部分轨迹缺少逐帧观测
// 来源（null 旧帧）不能成为跳过该核对的理由。
func TestOpenLegacyMixedFileInvalidationHashChecked(t *testing.T) {
	path := filepath.Join(t.TempDir(), "mixed.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}, {ID: "M", X: 2}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1}, {ID: "M", X: 2}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	// 旧地图上追加失效操作：撤下 L、M 的第 1 次出现。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Invalidate(Invalidation{ID: "A", Reason: "gone", Landmarks: []string{"L", "M"}}); err != nil {
		t.Fatalf("invalidate on legacy map: %v", err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	// 合法混合文件正常打开。
	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed file: %v", err)
	}
	if err := m3.Close(); err != nil {
		t.Fatal(err)
	}

	// 篡改失效记录的指纹（对应仅含 L 的集合）：即使文件缺少逐帧来源，
	// 也必须按 ErrCorrupt 拒绝。
	fd := readFileData(t, path)
	fd.Invalidations[0].Hash = landmarkSetHash([]string{"L"})
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered mixed file: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}
