package posemap

import "fmt"

// tally.go 承载打开地图时“逐帧观测归属各次出现并计数”的唯一一次扫描，
// 供 validateLoaded（见 persistence.go）中的两类次数核对共用同一套规则：
//
//   - 路标各次出现记录的总次数核对：来源观测条数 + 该次出现保存的固定旧
//     贡献，必须等于记录次数；
//   - 逐条校正记录的次数核对：校正前、后次数都必须等于截至该记录结束时间
//     已经接纳的来源观测条数 + 固定旧贡献。
//
// 归属判定完全经由观测重放器的时间边界（occurrenceOf，规则见
// occurrence.go）：一条观测只计入它所在帧当时所属的那次出现；同一标识失
// 效后再次出现的次数分别累计，失效时间上的观测属于旧出现、再现时间上的
// 观测属于新出现；两次出现之间没有归属的观测在此直接按损坏拒绝。同一帧
// 对同一路标的多条观测各计一次，null 旧帧（缺少逐帧依据）不增加计数、
// 也不按一条观测猜测；没有观测的帧自然不增长。位置重放（距离冲突与等权
// 平均核对）仍由 replay 单独进行，本扫描只负责归属与计数。

// sourceObservationTally 是对全部逐帧来源做一次归属扫描后的计数结果：
//
//   - totals：每条来源观测按归属的（路标，出现编号）累计的总条数；
//   - first：归属各次出现的最早来源观测帧时间，供首次观测时间核对；
//   - cumBySrc：只在调用方预登记的来源下标 k（对应帧 trajectory[k+1]）
//     保存“从首帧到该帧（两端含）”的累计计数快照，供校正记录按自己的
//     结束帧截取；各快照相互独立，后续帧的增长不改写旧快照。
type sourceObservationTally struct {
	totals   map[occurrenceKey]int
	first    map[occurrenceKey]int64
	cumBySrc map[int]map[occurrenceKey]int
}

// tallySourceObservations 按帧时间升序对 sources 做唯一一次归属与计数扫
// 描。trajectory[k+1] 是来源 sources[k] 所在帧的位姿；endSrcIdxs 给出需
// 要保留累计快照的来源下标（校正记录结束帧所在下标）。任何一条观测指向
// 未登记路标或不属于任何一次出现，立即返回包装 ErrCorrupt 的错误（与其
// 他打开核对相同的错误类别）。
func tallySourceObservations(replay *observationReplay, trajectory []Pose, sources []*frameSrcJSON, endSrcIdxs map[int]struct{}) (*sourceObservationTally, error) {
	t := &sourceObservationTally{
		totals:   make(map[occurrenceKey]int),
		first:    make(map[occurrenceKey]int64),
		cumBySrc: make(map[int]map[occurrenceKey]int, len(endSrcIdxs)),
	}
	// running 是随帧推进的累计计数；快照只在预登记下标从它复制，因此
	// totals（全程总数）与各结束帧快照来自同一次扫描、同一套归属判定。
	running := make(map[occurrenceKey]int)
	for k, src := range sources {
		if src != nil {
			frameTime := trajectory[k+1].Time
			for _, ob := range src.Observations {
				num, known := replay.occurrenceOf(ob.ID, frameTime)
				if !known {
					return nil, fmt.Errorf("%w: observation of unknown landmark %s at frame time %d", ErrCorrupt, ob.ID, frameTime)
				}
				if num == 0 {
					return nil, fmt.Errorf("%w: observation of landmark %s at frame time %d belongs to no occurrence", ErrCorrupt, ob.ID, frameTime)
				}
				key := occurrenceKey{id: ob.ID, num: num}
				t.totals[key]++
				running[key]++
				if cur, ok := t.first[key]; !ok || frameTime < cur {
					t.first[key] = frameTime
				}
			}
		}
		if _, need := endSrcIdxs[k]; need {
			snap := make(map[occurrenceKey]int, len(running))
			for key, c := range running {
				snap[key] = c
			}
			t.cumBySrc[k] = snap
		}
	}
	return t, nil
}

// legacyContributions 汇总每个（路标，出现编号）保存的固定旧观测贡献次
// 数：旧地图缺少逐帧依据的既有观测不进入 tally 的来源计数，而以这里的固
// 定贡献计入所属出现；旧版平铺路标视为第 1 次出现。两类次数核对都在
// tally 的来源条数上另加这一贡献，不能当作零，也不能按缺少来源的帧数推
// 测。
func legacyContributions(landmarks []landmarkJSON) map[occurrenceKey]int {
	out := make(map[occurrenceKey]int, len(landmarks))
	for _, lm := range landmarks {
		if len(lm.Occurrences) == 0 {
			out[occurrenceKey{id: lm.ID, num: 1}] = lm.LegacyCount
			continue
		}
		for _, oj := range lm.Occurrences {
			out[occurrenceKey{id: lm.ID, num: oj.Number}] = oj.LegacyCount
		}
	}
	return out
}
