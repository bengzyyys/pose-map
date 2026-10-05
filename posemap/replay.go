package posemap

// replay.go 承载“按每次出现归组观测、保留旧观测贡献、逐条重建位置”的
// 公共重放流程，供提交回环校正（Correct，见 correction.go）与打开地图
// 时的核对（validateLoaded，见 persistence.go）共同使用，保证两种操作
// 对同一份观测流沿用完全一致的规则：
//
//   - 归组：每条观测按所在帧时间归入当时的那次出现（时间边界规则见
//     occurrence.go）；同一标识失效后再次出现时各次出现分别归组，旧
//     出现不参与新出现的合并；失效时间上的观测仍归入旧出现。
//   - 旧贡献：旧地图缺少逐帧依据的观测以文件中保存的次数与平均位置作
//     为固定起点（seed），不补造观测，也不随后续重放改变。
//   - 重建：观测按帧时间升序、同帧按输入次序逐条接纳（单条接纳规则见
//     aggregate.go）：首条观测建立位置，后续观测与当时平均位置比较，
//     距离恰等于合并上限仍接纳、超过才拒绝；位置是全部已接受观测（含
//     固定旧贡献）的等权平均，同帧重复观测各计一次，不能通过改变次序
//     让数值或接受结果变化。
//
// 重放器只持有归组边界与聚合结果；出现编号、有效状态、失效信息等仍由
// 各自流程在重放器之外维护，重放不改变它们。

// occurrenceKey 标识某一路标的某一次出现（出现编号 1 起）。
type occurrenceKey struct {
	id  string
	num int
}

// replayFrame 是一帧的重放输入：帧时间（决定观测归入哪次出现）、转换
// 观测到地图坐标所用的位姿（校正重放时为校正后位姿，打开核对时为文件
// 中的当前位姿）以及该帧的原始观测（自身坐标，按输入次序）。
type replayFrame struct {
	time         int64
	pose         Pose
	observations []Observation
}

// replayRejectReason 区分重放中一条观测无法按规则接纳的原因。
type replayRejectReason int

const (
	replayUnknownLandmark replayRejectReason = iota // 观测的路标没有登记归属边界
	replayNoOccurrence                              // 观测时间不属于该路标的任何一次出现
	replayNonFinite                                 // 转换到地图坐标后为非有限数值
	replayConflict                                  // 与当时平均位置的距离超过合并上限
)

// replayReject 报告重放中第一条无法接纳的观测：所在帧的实际时间、路标
// 标识与归属的出现编号（无法归组时编号为 0）。
type replayReject struct {
	reason replayRejectReason
	time   int64
	id     string
	num    int
}

// observationReplay 是一组路标出现的观测重放器。使用次序：
//
//  1. track 登记各标识历次出现的时间边界；
//  2. seed 登记参与重建的出现及其固定旧观测贡献（只有登记过的出现参与
//     重建；归入未登记出现的观测被跳过——校正只重建受影响的出现，其余
//     出现的观测位置不变）；
//  3. replay 按帧时间升序逐帧逐条重放观测；
//  4. agg 读取各次出现重建出的计数与等权平均位置。
//
// observed 只读取归组边界、不影响聚合，可在上述任何阶段调用。
type observationReplay struct {
	mergeDistance float64
	windows       map[string][]occurrenceWindow
	aggs          map[occurrenceKey]*observationAgg
}

func newObservationReplay(mergeDistance float64) *observationReplay {
	return &observationReplay{
		mergeDistance: mergeDistance,
		windows:       make(map[string][]occurrenceWindow),
		aggs:          make(map[occurrenceKey]*observationAgg),
	}
}

// track 登记一个路标各次出现的时间边界（编号即下标+1）。
func (r *observationReplay) track(id string, wins []occurrenceWindow) {
	r.windows[id] = wins
}

// seed 登记一次出现参与重建，并以其固定旧观测贡献（legacyCount 条、等权
// 平均 (lx,ly)；legacyCount 为 0 表示无旧贡献）作为聚合起点。起点本身
// 不做距离检查（这些观测此前已按同一规则接纳）。
func (r *observationReplay) seed(id string, num, legacyCount int, lx, ly float64) {
	a := &observationAgg{}
	if legacyCount > 0 {
		a.seed(legacyCount, lx, ly)
	}
	r.aggs[occurrenceKey{id: id, num: num}] = a
}

// occurrenceOf 把时间 t 的一次观测归入该路标当时的那次出现，返回 1 起
// 的出现编号；known 为假表示该路标没有登记归属边界，known 为真而返回
// 0 表示该时间不属于它的任何一次出现。
func (r *observationReplay) occurrenceOf(id string, t int64) (num int, known bool) {
	wins, ok := r.windows[id]
	if !ok {
		return 0, false
	}
	return occurrenceNumberAt(wins, t), true
}

// occurrences 返回一个路标登记的出现次数；未登记时返回 0。
func (r *observationReplay) occurrences(id string) int {
	return len(r.windows[id])
}

// observed 归组一段帧序列中实际观测到的（路标，出现编号）：每条观测各
// 归组一次，同一出现只记一次；无法归组的观测（路标未登记或不属于任何
// 出现）不记入。帧序列只需给出时间与观测，位姿不参与归组。
func (r *observationReplay) observed(frames []replayFrame) map[occurrenceKey]struct{} {
	out := make(map[occurrenceKey]struct{})
	for _, fr := range frames {
		for _, ob := range fr.observations {
			if num, known := r.occurrenceOf(ob.ID, fr.time); known && num != 0 {
				out[occurrenceKey{id: ob.ID, num: num}] = struct{}{}
			}
		}
	}
	return out
}

// replay 逐帧重放观测：帧必须按时间升序给出，同帧观测按输入次序逐条
// 接纳。全部接纳返回 nil；否则返回第一条无法接纳的观测（已接纳的观测
// 留在聚合中，由调用方决定整次拒绝后如何处理——两种调用方都在校验全部
// 通过前不触碰真实状态）。
func (r *observationReplay) replay(frames []replayFrame) *replayReject {
	for _, fr := range frames {
		for _, ob := range fr.observations {
			num, known := r.occurrenceOf(ob.ID, fr.time)
			if !known {
				return &replayReject{reason: replayUnknownLandmark, time: fr.time, id: ob.ID}
			}
			if num == 0 {
				return &replayReject{reason: replayNoOccurrence, time: fr.time, id: ob.ID}
			}
			a := r.aggs[occurrenceKey{id: ob.ID, num: num}]
			if a == nil {
				continue // 该次出现未登记：不参与本次重建
			}
			switch a.admit(fr.pose, ob, r.mergeDistance) {
			case obsNonFinite:
				return &replayReject{reason: replayNonFinite, time: fr.time, id: ob.ID, num: num}
			case obsConflict:
				return &replayReject{reason: replayConflict, time: fr.time, id: ob.ID, num: num}
			}
		}
	}
	return nil
}

// agg 返回某次出现当前重建出的聚合（计数与等权平均位置）；该出现未登记
// 时返回 nil。
func (r *observationReplay) agg(id string, num int) *observationAgg {
	return r.aggs[occurrenceKey{id: id, num: num}]
}
