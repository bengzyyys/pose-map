package posemap

import (
	"errors"
	"os"
	"path/filepath"
	"testing"
)

// buildInvalidationMap 构造一份含失效与再现的地图：
//   - t=100 观测 L（本地 (1,0)）与 M（本地 (2,0)），t=200 再观测 L；
//   - 操作 A（原因 "gone"）在 t=200 同时撤下 L、M 的第 1 次出现；
//   - t=300 位姿前进到 (10,0)，L 再次出现（第 2 次，当前有效）。
//
// 这是任务描述中的基准情形：A 的记录指向 L 的第 1 次出现。
func buildInvalidationMap(t *testing.T, path string) {
	t.Helper()
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1, Y: 0}, {ID: "M", X: 2, Y: 0}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1, Y: 0}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "A", Reason: "gone", Landmarks: []string{"L", "M"}}); err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
		{Time: 300, DX: 10, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

// buildTwoInvalidationsMap 在 buildInvalidationMap 基础上追加操作 B：
// t=300 再以原因 "gone" 撤下 L 的第 2 次出现。这样 L 的两次出现分别由
// 两条真实操作撤下，可用于构造“旧出现被谎称成由另一条操作撤下”的篡改。
func buildTwoInvalidationsMap(t *testing.T, path string) {
	t.Helper()
	buildInvalidationMap(t, path)
	m, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.Invalidate(Invalidation{ID: "B", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
		t.Fatal(err)
	}
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}
}

func landmarkIndex(fd *fileData, id string) int {
	for i, lm := range fd.Landmarks {
		if lm.ID == id {
			return i
		}
	}
	return -1
}

func invalEntryIndex(iv *invalJSON, id string) int {
	for i, l := range iv.Landmarks {
		if l.ID == id {
			return i
		}
	}
	return -1
}

// 合法文件：A 的记录准确指向 L、M 的第 1 次出现。打开后历史状态、查询与
// 重复提交结果都与保存前一致——旧出现保持失效，新出现保持有效，重复提交
// A 返回首次失效的编号与时间，不再次撤下新出现。
func TestOpenInvalidationRecordMatchesHistory(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildInvalidationMap(t, path)

	m, err := Open(path)
	if err != nil {
		t.Fatalf("open valid file: %v", err)
	}
	defer m.Close()

	hL, err := m.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(hL.Appearances) != 2 {
		t.Fatalf("L appearances = %+v", hL.Appearances)
	}
	a1, a2 := hL.Appearances[0], hL.Appearances[1]
	if a1.Active || a1.InvalidTime != 200 || a1.InvalidReason != "gone" || a1.InvalidOpID != "A" {
		t.Fatalf("L occ1 = %+v", a1)
	}
	if !a2.Active || a2.Number != 2 || a2.FirstSeenTime != 300 {
		t.Fatalf("L occ2 = %+v", a2)
	}
	hM, err := m.LandmarkAppearances("M")
	if err != nil {
		t.Fatal(err)
	}
	if len(hM.Appearances) != 1 || hM.Appearances[0].Active ||
		hM.Appearances[0].InvalidOpID != "A" || hM.Appearances[0].InvalidTime != 200 {
		t.Fatalf("M occ1 = %+v", hM.Appearances[0])
	}

	// 区域查询只含当前有效的 L 第 2 次出现。
	lms := activeLandmarks(t, m)
	if len(lms) != 1 || lms["L"].Count != 1 || lms["L"].X != 10 {
		t.Fatalf("active landmarks = %+v", lms)
	}

	// 重复提交 A（次序相反也视为同一集合）：返回首次结果，不再次撤下 occ2。
	again, err := m.Invalidate(Invalidation{ID: "A", Reason: "gone", Landmarks: []string{"M", "L"}})
	if err != nil {
		t.Fatalf("resubmit A: %v", err)
	}
	if again.Time != 200 || len(again.Landmarks) != 2 ||
		again.Landmarks[0] != (InvalidatedLandmark{ID: "L", Occurrence: 1}) ||
		again.Landmarks[1] != (InvalidatedLandmark{ID: "M", Occurrence: 1}) {
		t.Fatalf("resubmit result = %+v", again)
	}
	hL, _ = m.LandmarkAppearances("L")
	if !hL.Appearances[1].Active {
		t.Fatal("resubmitting A re-invalidated the new occurrence")
	}
	if lms := activeLandmarks(t, m); len(lms) != 1 {
		t.Fatalf("active landmarks changed after resubmit: %+v", lms)
	}
}

// 各类“成功失效结果与出现历史矛盾”的篡改都必须在 Open 时按 ErrCorrupt
// 拒绝，且不返回可用地图对象。
func TestOpenRejectsInvalidationResultMismatches(t *testing.T) {
	cases := []struct {
		name    string
		builder func(t *testing.T, path string)
		mutate  func(fd *fileData)
	}{
		{
			name:    "nonexistent occurrence number",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// A 指向 L 不存在的第 3 次出现：不能拿最新出现补位。
				fd.Invalidations[0].Landmarks[invalEntryIndex(&fd.Invalidations[0], "L")].Occurrence = 3
			},
		},
		{
			name:    "points at still active occurrence",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// A 改指向 t=300 后仍有效的第 2 次出现。
				fd.Invalidations[0].Landmarks[invalEntryIndex(&fd.Invalidations[0], "L")].Occurrence = 2
			},
		},
		{
			name:    "second entry points at nonexistent occurrence",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// 同批操作中 M 的条目不合法：L 的条目本身完全合法，
				// 但任意一个条目不满足对应关系都要拒绝整个文件。
				fd.Invalidations[0].Landmarks[invalEntryIndex(&fd.Invalidations[0], "M")].Occurrence = 2
			},
		},
		{
			name:    "unknown landmark id",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				fd.Invalidations[0].Landmarks[invalEntryIndex(&fd.Invalidations[0], "M")].ID = "ZZZ"
			},
		},
		{
			name:    "invalidation time mismatch",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// 记录时间改成 250：出现保存的失效时间仍是 200。250 本身
				// 落在轨迹间隔内，且 occ1(失效 200) 与 occ2(首见 300) 的
				// 次序要求仍然成立——仅交叉关系矛盾也必须拒绝。
				fd.Invalidations[0].Time = 250
			},
		},
		{
			name:    "invalidation reason mismatch",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				fd.Invalidations[0].Reason = "moved"
			},
		},
		{
			name:    "occurrence claims another operation",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// L 第 1 次出现被谎称由操作 B 撤下（时间、原因保持与 A
				// 记录相同）：不能仅凭时间和原因一致就认可 A 的记录。
				i := landmarkIndex(fd, "L")
				fd.Landmarks[i].Occurrences[0].InvalidOpID = "B"
			},
		},
		{
			name:    "later operation pointed back at old occurrence",
			builder: buildTwoInvalidationsMap,
			mutate: func(fd *fileData) {
				// B 本应指向 L 第 2 次出现，被改成指向第 1 次：旧出现实际
				// 由 A 撤下，即使原因相同也不认可，也不接受改指。
				b := &fd.Invalidations[1]
				if b.ID != "B" {
					t.Fatalf("unexpected invalidation order: %+v", fd.Invalidations)
				}
				b.Landmarks[invalEntryIndex(b, "L")].Occurrence = 1
			},
		},
		{
			name:    "old occurrence claimed by real later operation",
			builder: buildTwoInvalidationsMap,
			mutate: func(fd *fileData) {
				// B 真实存在（撤下第 2 次出现）；把第 1 次出现的撤下者改成
				// B、失效时间改成 B 的时间 300 会先造成出现区间重叠，因此
				// 这里只改操作标识、保留 200/gone：记录 A 与出现上的操作
				// 标识不符，必须拒绝。
				i := landmarkIndex(fd, "L")
				fd.Landmarks[i].Occurrences[0].InvalidOpID = "B"
			},
		},
		{
			name:    "orphaned invalidation mark on occurrence",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// M 的第 1 次出现被标记为由不存在的操作 X 撤下，同时把 A
				// 记录里的 M 条目删掉：所有失效记录自身都能正向对上某个
				// 出现，但没有任何操作记录引用 (X, M, 1)，反向核对必须
				// 抓住这个凭空出现的失效标记。
				i := landmarkIndex(fd, "M")
				fd.Landmarks[i].Occurrences[0].InvalidOpID = "X"
				iv := &fd.Invalidations[0]
				kept := iv.Landmarks[:0]
				for _, l := range iv.Landmarks {
					if l.ID != "M" {
						kept = append(kept, l)
					}
				}
				iv.Landmarks = kept
			},
		},
		{
			name:    "operation moved onto later occurrence orphans old one",
			builder: buildInvalidationMap,
			mutate: func(fd *fileData) {
				// 把 A 对 L 的失效从第 1 次出现整体搬到第 2 次：A 记录时间
				// 改为 300 并指向 occ2，occ2 被填成与 A 完全一致的失效信息，
				// M 的失效时间同步改为 300。轨迹、出现次序、观测归属与计数
				// 仍然成立，且每条失效记录正向都能对上等三要素的出现——唯独
				// occ1 仍标记着 A@200 却没有任何操作条目引用，必须由反向
				// 核对拒绝，不能把旧结果顺势指向最新记录。
				iv := &fd.Invalidations[0]
				iv.Time = 300
				iv.Landmarks[invalEntryIndex(iv, "L")].Occurrence = 2
				iL := landmarkIndex(fd, "L")
				occ2 := &fd.Landmarks[iL].Occurrences[1]
				occ2.Active = false
				occ2.InvalidTime = 300
				occ2.InvalidReason = "gone"
				occ2.InvalidOpID = "A"
				iM := landmarkIndex(fd, "M")
				fd.Landmarks[iM].Occurrences[0].InvalidTime = 300
			},
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			path := filepath.Join(t.TempDir(), "m.pose")
			tc.builder(t, path)
			fd := readFileData(t, path)
			tc.mutate(&fd)
			writeFileDataRaw(t, path, &fd)

			m, err := Open(path)
			if !errors.Is(err, ErrCorrupt) {
				t.Fatalf("err = %v, want ErrCorrupt", err)
			}
			if m != nil {
				t.Fatalf("Open returned usable map: %v", m)
			}
		})
	}
}

// 拒绝打开时原文件内容必须保持不变：不能删掉错误条目、补造出现或改写
// 失效信息后继续加载。
func TestOpenInvalidationMismatchLeavesFileUntouched(t *testing.T) {
	path := filepath.Join(t.TempDir(), "m.pose")
	buildInvalidationMap(t, path)
	fd := readFileData(t, path)
	fd.Invalidations[0].Landmarks[invalEntryIndex(&fd.Invalidations[0], "L")].Occurrence = 3
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

// 兼容没有失效记录的旧地图：平铺保存的路标仍视为第 1 次有效出现，旧路标
// 缺少首次观测时间不会单独导致这项检查失败。
func TestOpenLegacyFileWithoutInvalidations(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.pose")
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
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open legacy file: %v", err)
	}
	defer m2.Close()
	h, err := m2.LandmarkAppearances("L")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 1 || !h.Appearances[0].Active || h.Appearances[0].HasFirstSeen {
		t.Fatalf("legacy L = %+v", h.Appearances)
	}
	if lms := activeLandmarks(t, m2); len(lms) != 2 {
		t.Fatalf("legacy active landmarks = %+v", lms)
	}
}

// 旧地图后来产生的失效操作也要检查对应关系，但不要求补齐原本缺失的逐帧
// 观测：合法的混合文件打开后查询与重复提交结果与保存前一致；篡改交叉
// 关系后仍按 ErrCorrupt 拒绝。
func TestOpenLegacyFileLaterInvalidationChecked(t *testing.T) {
	path := filepath.Join(t.TempDir(), "mixed.pose")
	cfg := Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0}
	m, err := Create(path, cfg)
	if err != nil {
		t.Fatal(err)
	}
	if _, err := m.ImportSegment(Segment{ID: "old", Frames: []Frame{
		{Time: 100, Observations: []Observation{{ID: "L", X: 1}}},
		{Time: 200, Observations: []Observation{{ID: "L", X: 1}}},
	}}); err != nil {
		t.Fatal(err)
	}
	writeLegacyV1File(t, path, m.state)
	m.Close()

	// 旧路标（第 1 次出现，首次观测时间未知）在 t=200 被失效，t=300 再现。
	m2, err := Open(path)
	if err != nil {
		t.Fatal(err)
	}
	first, err := m2.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil {
		t.Fatalf("invalidate on legacy map: %v", err)
	}
	if first.Time != 200 || first.Landmarks[0].Occurrence != 1 {
		t.Fatalf("first result = %+v", first)
	}
	if _, err := m2.ImportSegment(Segment{ID: "new", Frames: []Frame{
		{Time: 300, DX: 5, Observations: []Observation{{ID: "L"}}},
	}}); err != nil {
		t.Fatal(err)
	}
	if err := m2.Close(); err != nil {
		t.Fatal(err)
	}

	m3, err := Open(path)
	if err != nil {
		t.Fatalf("reopen mixed file: %v", err)
	}
	h, _ := m3.LandmarkAppearances("L")
	if len(h.Appearances) != 2 {
		t.Fatalf("mixed history = %+v", h.Appearances)
	}
	if h.Appearances[0].Active || h.Appearances[0].HasFirstSeen ||
		h.Appearances[0].InvalidOpID != "op1" || h.Appearances[0].InvalidTime != 200 {
		t.Fatalf("mixed occ1 = %+v", h.Appearances[0])
	}
	if !h.Appearances[1].Active || h.Appearances[1].FirstSeenTime != 300 {
		t.Fatalf("mixed occ2 = %+v", h.Appearances[1])
	}
	// 重复提交 op1 仍返回首次结果，不撤下新出现。
	again, err := m3.Invalidate(Invalidation{ID: "op1", Reason: "gone", Landmarks: []string{"L"}})
	if err != nil || again.Time != first.Time || again.Landmarks[0].Occurrence != 1 {
		t.Fatalf("resubmit after reopen: %v %+v", err, again)
	}
	if h, _ := m3.LandmarkAppearances("L"); !h.Appearances[1].Active {
		t.Fatal("resubmit re-invalidated the new occurrence")
	}
	if err := m3.Close(); err != nil {
		t.Fatal(err)
	}

	// 篡改混合文件中失效记录与旧出现的对应关系：拒绝。
	fd := readFileData(t, path)
	fd.Invalidations[0].Time = 250
	writeFileDataRaw(t, path, &fd)
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) {
		t.Fatalf("tampered mixed file: err = %v, want ErrCorrupt, map=%v", err, m)
	}
}
