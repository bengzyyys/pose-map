# 本地定位与地图

这是一个在本机运行的本地定位与地图。`posemap` 包提供二维轨迹与路标
能力：按段导入有序帧、维护路标观测合并、查询历史位姿与矩形区域内路标，
并把全部数据持久化到本地文件，关闭后重新打开结果一致。

## 使用

```bash
go test ./...
```

## 基本流程

1. `posemap.Create(path, Config)` 用初始时间、位置、朝向、位置方差、
   最大帧间隔与路标合并距离创建一份地图（已有文件不会被覆盖）；
   已存在的地图用 `posemap.Open(path)` 打开。
2. `m.ImportSegment(Segment{ID, Frames})` 按段导入。整段校验通过才
   生效并落盘；失败返回可区分的 `*posemap.RejectError`（含从零开始的
   帧序号；路标/段冲突含标识），已提交数据不受影响。同一内容重复
   导入直接返回首次结果，同一标识不同内容会被拒绝。
3. 查询：`CurrentPose()`、`PoseAt(t)`（不晚于 t 的最后一份位姿）、
   `LandmarksInRect(Rect)`（含边界，按标识排序）。
4. `m.Close()` 关闭。之后用 `Open` 重新打开同一文件，查询与重复
   导入的结果与关闭前一致；文件损坏报 `ErrCorrupt`，版本不支持报
   `ErrUnsupportedVersion`，都不会被当作新地图覆盖。

`Ready()` 仍返回 true，表示基线包可以加载。
