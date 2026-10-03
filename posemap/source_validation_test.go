package posemap

import (
	"bytes"
	"encoding/binary"
	"encoding/json"
	"errors"
	"hash/crc32"
	"os"
	"path/filepath"
	"testing"
)

// writeFD 把手工构造的 fileData 编码为带合法 CRC 的完整地图文件并返回
// 其字节，供打开校验类测试构造“内容可解析、校验和正确但语义矛盾”的文件。
func writeFD(t *testing.T, path string, fd fileData) []byte {
	t.Helper()
	payload, err := json.Marshal(&fd)
	if err != nil {
		t.Fatal(err)
	}
	out := make([]byte, headerLen+len(payload)+4)
	copy(out[0:8], fileMagic)
	binary.BigEndian.PutUint16(out[8:10], fileVersion)
	binary.BigEndian.PutUint32(out[10:14], uint32(len(payload)))
	copy(out[headerLen:], payload)
	binary.BigEndian.PutUint32(out[len(out)-4:], crc32.ChecksumIEEE(out[:len(out)-4]))
	if err := os.WriteFile(path, out, 0o644); err != nil {
		t.Fatal(err)
	}
	return out
}

func sourceFDConfig() Config {
	return Config{MaxInterval: 1000, MergeDistance: 10.0}
}

func occJSON(num int, x, y float64, count int, firstSeen int64, hasFirst, active bool, invalidTime int64, reason string) occurrenceJSON {
	oj := occurrenceJSON{
		Number: num, X: x, Y: y, Count: count,
		FirstSeenTime: firstSeen, HasFirstSeen: hasFirst, Active: active,
	}
	if !active {
		oj.InvalidTime = invalidTime
		oj.InvalidReason = reason
	}
	return oj
}

func src(obs ...Observation) *frameSrcJSON {
	return &frameSrcJSON{Observations: obs}
}

// 逐帧观测指向根本不存在的路标：即使 CRC 正确也必须在打开时以
// ErrCorrupt 拒绝，且不返回可用地图。
func TestOpenRejectsObservationOfUnknownLandmark(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	fd := fileData{
		Version:    int(fileVersion),
		Config:     sourceFDConfig(),
		Trajectory: []Pose{{Time: 0}, {Time: 100, X: 1}},
		Sources:    []*frameSrcJSON{src(Observation{ID: "G"})},
	}
	raw := writeFD(t, path, fd)

	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("corrupt open returned usable map: %v", m)
	}
	// 拒绝不得改写原文件。
	got, err := os.ReadFile(path)
	if err != nil || !bytes.Equal(got, raw) {
		t.Fatalf("file modified on rejected open: %v", err)
	}
}

// 路标 100 首次出现、200 失效、300 再次出现：100/200 的观测属于第一次
// 出现（失效端点包含），250 的观测没有归属，不能划给第二次出现。
func TestOpenRejectsObservationBetweenOccurrences(t *testing.T) {
	path := filepath.Join(t.TempDir(), "map.pose")
	mkFD := func() fileData {
		return fileData{
			Version: int(fileVersion),
			Config:  sourceFDConfig(),
			Trajectory: []Pose{
				{Time: 0}, {Time: 100, X: 1}, {Time: 200, X: 2},
				{Time: 250, X: 2.5}, {Time: 300, X: 3},
			},
			Sources: []*frameSrcJSON{
				src(Observation{ID: "A"}), // 100：第 1 次出现
				src(Observation{ID: "A"}), // 200：恰为失效时间，仍属第 1 次
				src(Observation{ID: "A"}), // 250：无归属
				src(Observation{ID: "A"}), // 300：第 2 次出现
			},
			Landmarks: []landmarkJSON{{
				ID: "A",
				Occurrences: []occurrenceJSON{
					occJSON(1, 1, 0, 2, 100, true, false, 200, "gone"),
					occJSON(2, 3, 0, 1, 300, true, true, 0, ""),
				},
			}},
		}
	}

	raw := writeFD(t, path, mkFD())
	m, err := Open(path)
	if !errors.Is(err, ErrCorrupt) {
		t.Fatalf("gap frame: err = %v, want ErrCorrupt", err)
	}
	if m != nil {
		t.Fatalf("corrupt open returned map: %v", m)
	}
	got, _ := os.ReadFile(path)
	if !bytes.Equal(got, raw) {
		t.Fatal("rejected open modified the file")
	}

	// 同一文件去掉 250 那条无归属观测（帧本身保留）即可打开；失效历史
	// 中的 100/200 观测不因其出现已失效而被误拒。
	fd := mkFD()
	fd.Sources[2] = src() // 250 帧无观测
	writeFD(t, path, fd)
	m2, err := Open(path)
	if err != nil {
		t.Fatalf("open without gap observation: %v", err)
	}
	defer m2.Close()

	h, err := m2.LandmarkAppearances("A")
	if err != nil {
		t.Fatal(err)
	}
	if len(h.Appearances) != 2 {
		t.Fatalf("appearances = %v", h.Appearances)
	}
	a1, a2 := h.Appearances[0], h.Appearances[1]
	if a1.Active || a1.InvalidTime != 200 || a1.InvalidReason != "gone" || a1.FirstSeenTime != 100 || !a1.HasFirstSeen {
		t.Fatalf("occurrence 1 changed: %+v", a1)
	}
	if !a2.Active || a2.FirstSeenTime != 300 {
		t.Fatalf("occurrence 2 changed: %+v", a2)
	}
	// 打开成功不重新导入或合并观测：计数与文件中的值保持原样。
	if a1.Landmark.Count != 2 || a2.Landmark.Count != 1 {
		t.Fatalf("observation counts recomputed on open: %d %d", a1.Landmark.Count, a2.Landmark.Count)
	}
	// 当前位姿、时间查询、区域查询沿用原有行为。
	cur, _ := m2.CurrentPose()
	if cur.Time != 300 || cur.X != 3 {
		t.Fatalf("current pose = %+v", cur)
	}
	p, _ := m2.PoseAt(220)
	if p.Time != 200 {
		t.Fatalf("PoseAt(220) = %+v", p)
	}
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].ID != "A" || lms[0].Count != 1 {
		t.Fatalf("rect query = %v", lms)
	}
}

// 旧路标首次观测时间未知（has_first_seen 为假）时不设起始约束，但已有
// 失效时间仍约束历史观测：nil 来源的旧帧不要求补齐依据，而后来追加的、
// 确实带来源的帧必须能归入某次出现。
func TestOpenLegacyOccurrenceInvalidTimeStillBounds(t *testing.T) {
	mkFD := func(frameTime int64) fileData {
		return fileData{
			Version: int(fileVersion),
			Config:  sourceFDConfig(),
			Trajectory: []Pose{
				{Time: 0}, {Time: 200, X: 2}, {Time: frameTime, X: 2.5},
			},
			// 第 0 帧来自旧地图（无逐帧依据）；第 1 帧是后来追加的来源帧。
			Sources: []*frameSrcJSON{
				nil,
				src(Observation{ID: "A"}),
			},
			Landmarks: []landmarkJSON{{
				ID: "A",
				Occurrences: []occurrenceJSON{
					occJSON(1, 1, 0, 3, 0, false, false, 200, "gone"),
				},
			}},
		}
	}

	path := filepath.Join(t.TempDir(), "map.pose")

	// 追加帧在旧出现失效之后：无归属（也没有更新的出现），拒绝。
	writeFD(t, path, mkFD(250))
	if m, err := Open(path); !errors.Is(err, ErrCorrupt) || m != nil {
		t.Fatalf("frame 250: m=%v err=%v, want ErrCorrupt", m, err)
	}

	// 追加帧恰为失效时间：端点包含，旧出现起始时间未知也不设障碍，打开成功。
	writeFD(t, path, mkFD(200))
	m, err := Open(path)
	if err != nil {
		t.Fatalf("frame at invalid time must belong to the old occurrence: %v", err)
	}
	defer m.Close()
	h, _ := m.LandmarkAppearances("A")
	if len(h.Appearances) != 1 || h.Appearances[0].HasFirstSeen || h.Appearances[0].Active ||
		h.Appearances[0].InvalidTime != 200 || h.Appearances[0].Landmark.Count != 3 {
		t.Fatalf("legacy occurrence changed after open: %+v", h.Appearances)
	}
}

// 完全不带 sources 的历史旧地图照常打开，即使路标出现在多帧中也不要求
// 补齐依据。
func TestOpenOldFileWithoutSourcesStillOpens(t *testing.T) {
	path := filepath.Join(t.TempDir(), "old.pose")
	m, err := Create(path, Config{InitialTime: 0, MaxInterval: 1000, MergeDistance: 1.0})
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
	if err := m.Close(); err != nil {
		t.Fatal(err)
	}

	m2, err := Open(path)
	if err != nil {
		t.Fatalf("legacy file must still open: %v", err)
	}
	defer m2.Close()
	lms, _ := m2.LandmarksInRect(Rect{MinX: -1e9, MinY: -1e9, MaxX: 1e9, MaxY: 1e9})
	if len(lms) != 1 || lms[0].Count != 2 {
		t.Fatalf("legacy landmarks = %v", lms)
	}
	// 失效原因、出现编号与观测计数保持原样。
	h, _ := m2.LandmarkAppearances("K")
	if len(h.Appearances) != 1 || h.Appearances[0].Number != 1 || !h.Appearances[0].Active ||
		h.Appearances[0].HasFirstSeen || h.Appearances[0].Landmark.Count != 2 {
		t.Fatalf("legacy occurrence = %+v", h.Appearances)
	}
}
