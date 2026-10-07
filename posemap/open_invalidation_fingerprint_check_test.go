package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

// 任务描述中的矛盾文件：操作 A 在 t=200 同时撤下 L、M 的第 1 次出现，
// 之后 L 在 t=300 的新轨迹中再次出现，但 A 保存的集合指纹被换成仅含 L
// 的集合指纹。结果、时间、原因、操作标识与出现历史全部对应，只有指纹
// 属于另一组路标——打开时必须按 ErrCorrupt 拒绝整份地图，错误说明能辨
// 认出操作 A，且不返回可用地图对象。
func TestOpenRejectsInvalidationSetFingerprintMismatch(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildInvalidationMap(t, path)
	fd := readFileData(t, path)
	iv := &fd.Invalidations[0]
	if iv.ID != "A" || len(iv.Landmarks) != 2 {
		t.Fatalf("unexpected invalidation record: %+v", iv)
	}
	iv.Hash = landmarkSetHash([]string{"L"}) // 实际集合是 {L,M}
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	if !strings.Contains(err.Error(), "A") {
		t.Fatalf("error does not identify invalidation A: %v", err)
	}
}

// 内容一致的文件重新打开后，以原原因、任意 L/M 次序重复提交 A，都返回
// 首次撤下的时间与两个路标的第 1 次出现，不撤下后来有效的 L。
func TestOpenValidFingerprintResubmitAnyOrder(t *testing.T) {
	for _, order := range [][]string{{"L", "M"}, {"M", "L"}} {
		t.Run(strings.Join(order, "_"), func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildInvalidationMap(t, path)
			m, err := Open(path)
			if err != nil {
				t.Fatal(err)
			}
			defer m.Close()
			res, err := m.Invalidate(Invalidation{ID: "A", Reason: "gone", Landmarks: order})
			if err != nil {
				t.Fatalf("resubmit A in order %v: %v", order, err)
			}
			if res.Time != 200 || len(res.Landmarks) != 2 ||
				res.Landmarks[0] != (InvalidatedLandmark{ID: "L", Occurrence: 1}) ||
				res.Landmarks[1] != (InvalidatedLandmark{ID: "M", Occurrence: 1}) {
				t.Fatalf("resubmit result = %+v", res)
			}
			h, _ := m.LandmarkAppearances("L")
			if !h.Appearances[1].Active {
				t.Fatal("resubmit re-invalidated the later occurrence")
			}
		})
	}
}

// 不能靠“让文件重新自洽”来掩盖指纹矛盾：替换指纹的同时删掉条目或调整
// 失效结果，仍会被既有的正/反向核对拒绝。
func TestOpenFingerprintMismatchCannotBeSalvaged(t *testing.T) {
	cases := []struct {
		name   string
		mutate func(fd *fileData)
	}{
		{
			name: "hash and result both reduced to subset",
			mutate: func(fd *fileData) {
				// 把 M 条目从 A 的结果中删掉，并把指纹改成仅含 L：M 的第 1
				// 次出现仍标记由 A 撤下，反向核对必须拒绝。
				iv := &fd.Invalidations[0]
				kept := iv.Landmarks[:0]
				for _, l := range iv.Landmarks {
					if l.ID != "M" {
						kept = append(kept, l)
					}
				}
				iv.Landmarks = kept
				iv.Hash = landmarkSetHash([]string{"L"})
			},
		},
		{
			name: "hash of wholly unrelated set",
			mutate: func(fd *fileData) {
				// 指纹对应结果中完全没有的另一组路标。
				fd.Invalidations[0].Hash = landmarkSetHash([]string{"N", "O"})
			},
		},
		{
			name: "result enlarged to match foreign hash",
			mutate: func(fd *fileData) {
				// 给结果补一个 Z 条目以凑出 {L,M,Z} 的指纹：Z 路标不存在，
				// 正向交叉核对拒绝，不能用调整结果的方式让文件打开。
				iv := &fd.Invalidations[0]
				iv.Landmarks = append(iv.Landmarks, invalLMJSON{ID: "Z", Occurrence: 1})
				iv.Hash = landmarkSetHash([]string{"L", "M", "Z"})
			},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			buildInvalidationMap(t, path)
			fd := readFileData(t, path)
			tc.mutate(&fd)
			writeFileDataRaw(t, path, &fd)
			if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt, map=%v", err, m)
			}
		})
	}
}

// 旧地图（部分轨迹缺少逐帧观测来源）后来追加的失效记录同样必须满足指纹
// 一致性，不能因依据缺失而跳过。
func TestOpenLegacyFileFingerprintMismatchRejected(t *testing.T) {
	path := filepath.Join(t.TempDir(), "mixed.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}, {ID: "M", X: 2}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L", "M"}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	fd := readFileData(t, path)
	fd.Invalidations[0].Hash = landmarkSetHash([]string{"L"})
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("legacy mixed file: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 拒绝指纹矛盾文件时原文件字节必须保持不变。
func TestOpenFingerprintMismatchLeavesFileUntouched(t *testing.T) {
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
