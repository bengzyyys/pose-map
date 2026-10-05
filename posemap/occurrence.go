package posemap

// occurrence.go 承载“观测按所在帧时间归入当时那次出现”这一业务规则，
// 供观测重放器（observationReplay，见 replay.go）归组使用；提交回环校正
// 与打开地图时的核对都经由重放器共享这同一份时间边界判断，两处始终一致。

// occurrenceWindow 是一次出现接纳观测的时间边界：
//
//   - 已知首次观测时间的出现只接纳不早于该时间的观测（端点包含）；
//   - 已失效的出现仍接纳失效时间上的观测（端点包含），之后的观测不再
//     属于它；
//   - 仍有效的出现没有失效时间上界；
//   - 首次观测时间未知（旧文件缺少逐帧来源）的出现没有下界，继续接纳
//     更早的历史观测，不把零当作补造的开始时间。
//
// 同一路标各次出现的时间区间互不重叠（打开文件时由 validateLoaded 强制
// 校验，区间矛盾的文件按损坏拒绝），因此任一时刻至多有一次出现接纳
// 观测；旧观测永远归入它所在帧当时的那次出现，不因后来出现了更新的
// 有效记录而改属。
type occurrenceWindow struct {
	hasFirstSeen  bool
	firstSeenTime int64
	active        bool
	invalidTime   int64
}

// owns 判断时间 t 的观测是否属于这次出现。
func (w occurrenceWindow) owns(t int64) bool {
	if w.hasFirstSeen && t < w.firstSeenTime {
		return false
	}
	if !w.active && t > w.invalidTime {
		return false
	}
	return true
}

// occurrenceNumberAt 把时间 t 的一次观测归入当时的那次出现，返回 1 起
// 的出现编号；没有任何出现接纳时返回 0。
func occurrenceNumberAt(wins []occurrenceWindow, t int64) int {
	for i, w := range wins {
		if w.owns(t) {
			return i + 1
		}
	}
	return 0
}

// window 返回该出现状态对应的时间边界。
func (o *occurrenceState) window() occurrenceWindow {
	return occurrenceWindow{
		hasFirstSeen:  o.hasFirstSeen,
		firstSeenTime: o.firstSeenTime,
		active:        o.active,
		invalidTime:   o.invalidTime,
	}
}

// occurrenceWindows 返回该路标各次出现的时间边界（编号即下标+1），供
// 重放器登记归属边界使用。
func (lm *landmarkState) occurrenceWindows() []occurrenceWindow {
	wins := make([]occurrenceWindow, 0, len(lm.appearances))
	for _, o := range lm.appearances {
		wins = append(wins, o.window())
	}
	return wins
}

// occurrenceWindowsOf 从文件记录中提取一个路标各次出现的时间边界，供
// 打开文件时登记重放器的归属边界使用。旧版平铺路标视为一次首次观测时间未知、
// 仍有效的第 1 次出现（无下界、无上界）。
func occurrenceWindowsOf(lm landmarkJSON) []occurrenceWindow {
	if len(lm.Occurrences) == 0 {
		return []occurrenceWindow{{active: true}}
	}
	wins := make([]occurrenceWindow, 0, len(lm.Occurrences))
	for _, oj := range lm.Occurrences {
		wins = append(wins, occurrenceWindow{
			hasFirstSeen:  oj.HasFirstSeen,
			firstSeenTime: oj.FirstSeenTime,
			active:        oj.Active,
			invalidTime:   oj.InvalidTime,
		})
	}
	return wins
}
