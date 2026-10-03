package posemap

// 本文件集中维护“一条观测按其所在帧时间归入同一路标哪一次出现”的
// 业务规则，供打开文件时的归属校验与回环校正时的观测归并共同使用，
// 不在两处各写一套时间边界判断。
//
// 边界规则（对单次出现，t 为观测所在帧时间）：
//   - 已知首次观测时间（hasFirst）时，只包含不早于该时间的观测，恰好
//     处于首次观测时间的观测属于这次出现（下端点包含）；
//   - 已失效出现只包含不晚于失效时间的观测，失效时间上的观测仍属于它
//     （上端点包含），之后的观测不再属于它；
//   - 仍有效的出现没有失效时间上界；
//   - 旧文件第 1 次出现缺少逐帧来源，首次观测时间未知（hasFirst 为
//     假），接纳没有下界的历史观测，零时间不被补造为开始时间。
//
// 同一路标各次出现的区间互不重叠，观测只按“当时”的出现归属：已经
// 失效后再被观测会产生新的出现编号，不能因为当前存在更新的有效记录
// 就把旧观测并入新记录。

// occurrenceTimeBounds 是某次出现用于观测归属的时间边界。hasFirst 为假
// 表示首次观测时间未知（旧文件第 1 次出现），没有下界；active 为真
// 表示仍有效，没有失效时间上界。
type occurrenceTimeBounds struct {
	hasFirst    bool
	firstSeen   int64
	active      bool
	invalidTime int64
}

// timeBounds 返回该出现的时间边界。
func (o *occurrenceState) timeBounds() occurrenceTimeBounds {
	return occurrenceTimeBounds{
		hasFirst:    o.hasFirstSeen,
		firstSeen:   o.firstSeenTime,
		active:      o.active,
		invalidTime: o.invalidTime,
	}
}

// ownsTime 报告帧时间 t 上的一条观测是否属于本次出现。
func (b occurrenceTimeBounds) ownsTime(t int64) bool {
	if b.hasFirst && t < b.firstSeen {
		return false
	}
	if !b.active && t > b.invalidTime {
		return false
	}
	return true
}

// occurrenceNumberAt 返回帧时间 t 的观测应当归入的出现编号（1 起）；
// 路标不存在或 t 不落入任何出现的区间（例如落在两次出现之间）时返回 0。
// 各次出现按编号递增检查，区间互不重叠，首个包含 t 的出现即归属。
func (lm *landmarkState) occurrenceNumberAt(t int64) int {
	if lm == nil {
		return 0
	}
	for i, o := range lm.appearances {
		if o.timeBounds().ownsTime(t) {
			return i + 1
		}
	}
	return 0
}

// occurrenceIndexAtTime 在给定的出现边界序列中返回帧时间 t 的观测归属
// 的下标；没有任何出现包含 t 时返回 false。用于打开文件时在解码后的
// 记录（尚未构造 landmarkState）上做同样的归属判断。
func occurrenceIndexAtTime(bounds []occurrenceTimeBounds, t int64) (int, bool) {
	for i, b := range bounds {
		if b.ownsTime(t) {
			return i, true
		}
	}
	return 0, false
}
