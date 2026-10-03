package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// 第一次出现的失效时间（350）晚于第二次出现的首次观测（300）：两次出现的
// 时间区间重叠，300 时刻的观测同时被两次出现接纳。即使格式与校验值正确、
// 重叠范围内的观测暂时能归入某一次，打开也必须按 ErrCorrupt 拒绝，不返回
// 可用地图对象，且原文件内容不变。
func TestOpenRejectsOverlappingOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].InvalidTime = 350
	writeFileDataRaw(t, path, &fd)
	saved, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("overlapping occurrences: err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("Open returned usable map: %v", m)
	}
	got, err := os.ReadFile(path)
	if err != nil {
		t.Fatal(err)
	}
	if string(got) != string(saved) {
		t.Fatal("Open modified the rejected file")
	}
}

// 上一次失效时间与下一次首次观测时间相等（都是 300）也属于重叠：两个
// 端点都接纳所在时刻的观测，必须拒绝。
func TestOpenRejectsTouchingOccurrenceEndpoints(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].InvalidTime = 300
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("touching endpoints: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 单次出现内部首次观测时间晚于失效时间（记录本身自相矛盾）：拒绝。这里
// 同时去掉逐帧观测，确保拒绝只由出现记录的时间关系触发，而非观测归属。
func TestOpenRejectsFirstSeenAfterInvalidation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 250 // 失效时间为 200
	fd.Sources[0].Observations = nil
	fd.Sources[1].Observations = nil
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("first seen after invalidation: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 第二次出现的首次观测时间未知：只有旧文件的第 1 次出现允许未知，第 2 次
// 及以后必定在上一次失效后由本包创建，未知即损坏。
func TestOpenRejectsLaterOccurrenceWithoutFirstSeen(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[1].HasFirstSeen = false
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("later occurrence without first seen: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}

// 旧地图兼容：第 1 次出现首次观测时间未知（缺少逐帧来源）保留未知含义，
// 失效（200）后再出现（300）仍遵守时间次序，文件正常打开且记录原样保留。
func TestOpenLegacyUnknownFirstSeenStillOpens(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].HasFirstSeen = false
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 0
	writeFileDataRaw(t, path, &fd)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("legacy unknown first seen rejected: %v", err)
	}
	defer m.Close()
	h, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %+v", h.Appearances)
	}
	a1, a2 := h.Appearances[0], h.Appearances[1]
	if a1.HasFirstSeen || a1.Active || a1.InvalidTime != 200 {
		t.Fatalf("occ1 = %+v", a1)
	}
	if !a2.HasFirstSeen || a2.FirstSeenTime != 300 || !a2.Active {
		t.Fatalf("occ2 = %+v", a2)
	}
}

// 旧路标失效后的后续出现仍须遵守时间次序：第 1 次出现首次观测时间未知
// 不免除其失效时间（350）与第二次出现首次观测（300）的次序约束。
func TestOpenRejectsLegacyOverlapAfterInvalidation(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildOccurrenceMap(t, path)
	fd := readFileData(t, path)
	fd.Landmarks[0].Occurrences[0].HasFirstSeen = false
	fd.Landmarks[0].Occurrences[0].FirstSeenTime = 0
	fd.Landmarks[0].Occurrences[0].InvalidTime = 350
	writeFileDataRaw(t, path, &fd)

	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("legacy overlap: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}
