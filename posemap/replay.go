package posemap

import "sort"

// replay.go 承载“按帧时间与同帧输入次序，把每条逐帧观测归入当时那次
// 出现并逐条重放”这一公共业务，供回环校正提交（Correct）与打开地图时的
// 位置核对（validateLoaded，见 persistence.go）共同使用。两条流程原本
// 各自维护一套“按（路标，出现编号）归组、保留不在本次范围内的固定旧贡
// 献、逐条重建位置”的实现，规则相同而代码重复；这里归并为同一份流程，
// 两处对同一份文件/状态必然得到一致的归组、接纳与等权平均结果。
//
// 统一规则与轨迹导入（见 aggregate.go）完全一致：
//
//   - 同一标识失效后再次出现是不同的出现，各次出现分别维护位置与次数，
//     旧出现的观测永不参与新出现的合并；观测按帧时间归入当时的那次出现
//     （归属规则见 occurrence.go，失效时间上的观测归入旧出现）；
//   - 观测按帧时间升序、同帧按输入次序逐条接纳，改变次序不能改变数值或
//     接受结果之外的任何东西——同帧重复观测各计一次；
//   - 重放范围之外、固定有效的观测以 occurrenceSeed 作为聚合起点：校正
//     时锚点之前的观测使用校正前位置（固定，不随本次校正改变）；旧文件
//     缺少逐帧依据的观测按文件中原有次数与平均位置作为固定贡献，不补造
//     观测；起点本身不做距离检查；
//   - 首条纳入的观测建立位置，其后每条与当时平均位置比较，距离恰等于
//     合并上限可以接受，严格超过才拒绝；位置始终是已接受观测的等权平均。
//
// 出现编号、有效状态与失效信息由调用方在重放器之外维护，重放器只读取
// 归属、重建位置与计数，不改变它们。

// occurrenceKey 标识某一路标的某一次出现。回环校正跨越同一路标的多次
// 出现、或打开文件核对历次出现时，都以它作为各次出现独立重放的分组键。
type occurrenceKey struct {
	id  string
	num int // 出现编号，从 1 开始
}

// occurrenceSeed 是某次出现“不在本次重放范围内、但固定有效”的已接受
// 观测贡献：fixedCount 条观测的等权平均 (fixedX, fixedY)。fixedCount 为
// 0 时没有固定贡献，该次出现从重放到的第一条观测开始建点。
//
// 两种固定来源：
//
//   - 回环校正：锚点之前已接受的观测，位置取校正前地图位置，不随本次
//     校正改变；
//   - 旧文件出现：缺少逐帧依据的观测无法重放，其贡献固定为文件中保存的
//     原有次数（legacyCount）与平均位置（legacyMX/Y）。
type occurrenceSeed struct {
	fixedCount int
	fixedX     float64
	fixedY     float64
}

// replayFrame 是参与重放的一帧依据：该帧重放时使用的地图坐标位姿与该帧
// 携带的原始观测（机器人自身坐标，同帧次序即切片次序）。frames 必须已按
// 帧时间升序排列。hasBasis 为假表示该帧来自不携带逐帧依据的旧文件（没有
// 可重放的观测，其贡献已包含在对应出现的固定起点中）；hasBasis 为真而
// observations 为空是合法的无观测帧。
type replayFrame struct {
	pose         Pose
	hasBasis     bool
	observations []Observation
}

// replayObservationError 描述重放中一条观测违反统一接纳规则的位置：实际
// 出错帧的时间、路标标识与其所属的出现编号。两种规则共用同一结构：
// 转换到地图坐标后为非有限值（non_finite，优先于距离冲突），或与当时
// 平均位置的距离严格超过合并上限（landmark_conflict）。
type replayObservationError struct {
	frameTime  int64
	landmarkID string
	occurrence int
}

// replayResult 是某次出现重放后的聚合结果。
type replayResult struct {
	key   occurrenceKey
	x     float64
	y     float64
	count int
}

// replayObservations 把 frames（按帧时间升序）中每一帧的逐帧观测归入
// seeds 提供的各次出现并按统一规则逐条重放。
//
// owner 把一条观测按其所在帧时间归入当时的那次出现，返回 1 起的编号；
// 返回 0 表示没有任何出现接纳（调用方已在别处做归属校验时不会发生）。
// seeds 给出每次出现的固定起点（无固定贡献时给 fixedCount=0 的零值即
// 可；未列入 seeds 的出现视为从零开始）。mergeDistance 是接纳门槛，不
// 是位置误差容限。
//
// 重放要么完整成功并返回按标识再按出现编号升序的各次出现结果，要么在第
// 一条违反规则的观测处停止并返回其位置与原因（obsNonFinite 优先于
// obsConflict），不产生部分结果。
func replayObservations(
	frames []replayFrame,
	owner func(id string, frameTime int64) int,
	seeds map[occurrenceKey]occurrenceSeed,
	mergeDistance float64,
) ([]replayResult, *replayObservationError, obsRejectReason) {
	// 先为每次出现建立聚合：仅有固定起点、本范围内没有任何观测的出现也占
	// 一个结果（打开核对时所有出现都要逐一对位置，旧贡献本身即其重放位
	// 置）；固定起点之后又有观测纳入时在同一聚合上增量合并。
	aggs := make(map[occurrenceKey]*observationAgg, len(seeds))
	for k, s := range seeds {
		a := &observationAgg{}
		if s.fixedCount > 0 {
			// 固定旧贡献作为聚合起点：校正范围之外的观测与旧文件缺少逐帧
			// 依据的观测都不随后续重放改变，也不补造观测。
			a.seed(s.fixedCount, s.fixedX, s.fixedY)
		}
		aggs[k] = a
	}
	ensureAgg := func(k occurrenceKey) *observationAgg {
		if a := aggs[k]; a != nil {
			return a
		}
		a := &observationAgg{}
		aggs[k] = a
		return a
	}

	for _, fr := range frames {
		if !fr.hasBasis {
			continue // 旧文件帧：观测已计入对应出现的固定起点
		}
		for _, ob := range fr.observations {
			num := owner(ob.ID, fr.pose.Time)
			if num == 0 {
				// 归属矛盾：调用方应已在来源校验中按损坏拒绝；这里不猜测
				// 归属，直接跳过会虚减次数，按损坏信号返回。
				return nil, &replayObservationError{frameTime: fr.pose.Time, landmarkID: ob.ID, occurrence: 0}, obsConflict
			}
			k := occurrenceKey{id: ob.ID, num: num}
			// 转换、非有限优先、首条建点、距离判定（恰含上限）、增量平均
			// 与计数全部走统一规则（见 aggregate.go）。
			switch ensureAgg(k).admit(fr.pose, ob, mergeDistance) {
			case obsNonFinite:
				return nil, &replayObservationError{frameTime: fr.pose.Time, landmarkID: ob.ID, occurrence: num}, obsNonFinite
			case obsConflict:
				return nil, &replayObservationError{frameTime: fr.pose.Time, landmarkID: ob.ID, occurrence: num}, obsConflict
			}
		}
	}

	keys := make([]occurrenceKey, 0, len(aggs))
	for k := range aggs {
		keys = append(keys, k)
	}
	sort.Slice(keys, func(i, j int) bool {
		if keys[i].id != keys[j].id {
			return keys[i].id < keys[j].id
		}
		return keys[i].num < keys[j].num
	})
	out := make([]replayResult, 0, len(keys))
	for _, k := range keys {
		a := aggs[k]
		out = append(out, replayResult{key: k, x: a.x, y: a.y, count: a.count})
	}
	return out, nil, obsAccepted
}
