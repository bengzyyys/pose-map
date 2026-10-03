package posemap

import (
	"os"
	"path/filepath"
	"testing"
)

// 回环校正中观测转换到地图坐标后溢出：校正目标、原始观测与校正后的机器人
// 位姿都只有有限数值，但路标地图坐标仍可能因数值过大产生无穷值。必须按
// non_finite 拒绝并指出异常观测所在帧的实际时间、路标标识与从 1 起的出现
// 编号，而不是误报为距离冲突，也不能只给锚点时间。
func TestCorrectionNonFiniteObservation(t *testing.T) {
	hugeCfg := Config{
		InitialTime: 0, InitialX: 0, InitialY: 0, InitialHeading: 0,
		InitialVariance: 0, MaxInterval: 1000, MergeDistance: 10,
	}

	// 任务示例：原点、朝向零，t=50/100/200 保存三个无运动帧，t=50 与
	// t=200 观测 L(1e308,0)；以 t=100 为锚点把目标位置改为 (1e308,0)。
	// 机器人位置仍有限，但 t=200 的路标地图坐标溢出。
	t.Run("overflow is not a distance conflict", func(t *testing.T) {
		path := filepath.Join(t.TempDir(), "map.bin")
		m, err := Create(path, hugeCfg)
		if err != nil {
			t.Fatalf("Create: %v", err)
		}
		defer m.Close()
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 50, Observations: []Observation{{ID: "L", X: 1e308}}},
			{Time: 100},
			{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		before, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}

		_, err = m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 1e308, Y: 0}})
		r, ok := AsRejectError(err)
		if !ok {
			t.Fatalf("err = %v, want *RejectError", err)
		}
		if r.Kind != RejectNonFinite {
			t.Fatalf("kind = %q, want non_finite (not landmark_conflict)", r.Kind)
		}
		if !r.HasTime || r.Time != 200 {
			t.Fatalf("time = %d(ok=%v), want offending frame time 200, not anchor 100", r.Time, r.HasTime)
		}
		if !r.HasLandmark || r.Landmark != "L" {
			t.Fatalf("landmark = %q(ok=%v), want L", r.Landmark, r.HasLandmark)
		}
		if !r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("occurrence = %d(ok=%v), want 1", r.Occurrence, r.HasOccurrence)
		}

		// 整次校正不生效：位姿、路标、观测次数、校正记录与本地文件保持原样。
		cur, _ := m.CurrentPose()
		if cur.Time != 200 || cur.X != 0 || cur.Y != 0 {
			t.Fatalf("pose changed after rejection: %+v", cur)
		}
		p, _ := m.PoseAt(100)
		if p.X != 0 || p.Y != 0 {
			t.Fatalf("anchor pose changed after rejection: %+v", p)
		}
		lms, _ := m.LandmarksInRect(Rect{MinX: -1e308, MinY: -1, MaxX: 1e308, MaxY: 1})
		if len(lms) != 1 || lms[0].X != 1e308 || lms[0].Y != 0 || lms[0].Count != 2 {
			t.Fatalf("landmark changed after rejection: %+v", lms)
		}
		if recs, _ := m.Corrections(); len(recs) != 0 {
			t.Fatalf("correction record leaked: %v", recs)
		}
		after, err := os.ReadFile(path)
		if err != nil {
			t.Fatal(err)
		}
		if len(after) != len(before) {
			t.Fatalf("data file changed size: %d -> %d", len(before), len(after))
		}
		for i := range before {
			if before[i] != after[i] {
				t.Fatalf("data file byte %d changed", i)
			}
		}
	})

	// 异常观测是该次出现重放到的第一条观测（此前没有任何已聚合观测）时，
	// 同样指出出错帧时间与路标，而不是退化为锚点上的笼统数值错误。
	t.Run("first observation of occurrence overflows", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100},
			{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		_, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 1e308}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite {
			t.Fatalf("err = %v, want non_finite", err)
		}
		if !r.HasTime || r.Time != 200 || !r.HasLandmark || r.Landmark != "L" ||
			!r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("reject = %+v, want time 200 landmark L occurrence 1", r)
		}
		lms, _ := m.LandmarksInRect(Rect{MinX: -1e308, MinY: -1, MaxX: 1e308, MaxY: 1})
		if len(lms) != 1 || lms[0].Count != 1 || lms[0].X != 1e308 {
			t.Fatalf("landmark changed after rejection: %+v", lms)
		}
	})

	// 校正跨越同一路标的多次出现时，错误对应异常观测实际所属的那一次，
	// 与该记录仍有效还是已经失效无关。
	t.Run("occurrence of offending observation is reported", func(t *testing.T) {
		wideCfg := hugeCfg
		wideCfg.MergeDistance = 1.5e308

		// 异常在已失效的第 1 次出现：t=200 的观测属于失效记录。
		m := newMap(t, wideCfg)
		if _, err := m.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, Observations: []Observation{{ID: "L", X: 0}}},
			{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 300, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatal(err)
		}
		_, err := m.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 1e308}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite {
			t.Fatalf("err = %v, want non_finite", err)
		}
		if !r.HasTime || r.Time != 200 || !r.HasLandmark || r.Landmark != "L" ||
			!r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("reject = %+v, want time 200 landmark L occurrence 1 (invalidated)", r)
		}
		h, _ := m.LandmarkAppearances("L")
		if len(h.Appearances) != 2 || h.Appearances[0].Active || !h.Appearances[1].Active ||
			h.Appearances[0].Landmark.Count != 2 || h.Appearances[1].Landmark.Count != 1 {
			t.Fatalf("appearances changed after rejection: %+v", h)
		}

		// 异常在仍有效的第 2 次出现。
		m2 := newMap(t, wideCfg)
		if _, err := m2.ImportSegment(Segment{ID: "s1", Frames: []Frame{
			{Time: 100, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m2.Invalidate(Invalidation{ID: "iv", Reason: "gone", Landmarks: []string{"L"}}); err != nil {
			t.Fatal(err)
		}
		if _, err := m2.ImportSegment(Segment{ID: "s2", Frames: []Frame{
			{Time: 200, Observations: []Observation{{ID: "L", X: 1e308}}},
		}}); err != nil {
			t.Fatal(err)
		}
		_, err = m2.Correct(Correction{ID: "c", Anchor: 100, Target: CorrectionTarget{X: 1e308}})
		r, ok = AsRejectError(err)
		if !ok || r.Kind != RejectNonFinite {
			t.Fatalf("err = %v, want non_finite", err)
		}
		if !r.HasTime || r.Time != 200 || !r.HasLandmark || r.Landmark != "L" ||
			!r.HasOccurrence || r.Occurrence != 2 {
			t.Fatalf("reject = %+v, want time 200 landmark L occurrence 2 (active)", r)
		}
	})

	// 转换结果有限但确实超过合并距离时，仍返回原有的距离冲突及对应信息。
	t.Run("finite but beyond merge distance stays conflict", func(t *testing.T) {
		m := newMap(t, hugeCfg)
		if _, err := m.ImportSegment(Segment{ID: "s", Frames: []Frame{
			{Time: 100, Observations: []Observation{{ID: "L", X: 0}}},
			{Time: 200, Observations: []Observation{{ID: "L", X: 0}}},
		}}); err != nil {
			t.Fatal(err)
		}
		// t=200 观测转换后为 (1e308,0)，有限，但距 t=100 的 (0,0) 超距。
		_, err := m.Correct(Correction{ID: "c", Anchor: 200, Target: CorrectionTarget{X: 1e308}})
		r, ok := AsRejectError(err)
		if !ok || r.Kind != RejectLandmarkConflict {
			t.Fatalf("err = %v, want landmark_conflict", err)
		}
		if !r.HasTime || r.Time != 200 || !r.HasLandmark || r.Landmark != "L" ||
			!r.HasOccurrence || r.Occurrence != 1 {
			t.Fatalf("reject = %+v, want conflict at time 200 landmark L occurrence 1", r)
		}
	})
}
