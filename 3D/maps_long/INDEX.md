# maps_long index
#
# 工作图（导航默认读这个）：arena_trav.yaml + arena_trav.pgm
# 时刻快照目录：map_YYYYMMDD_HHMMSS/（内含同名 yaml/pgm + README.txt）
# 已被工作图替代的散落 *_backup_* 文件在 archive/asset_backups/maps_long/，不参与运行。
# 续建示例：
#   seed_map_yaml:=$SCOUT_WS_ROOT/3D/maps_long/arena_trav.yaml
#   或 seed_map_yaml:=$SCOUT_WS_ROOT/3D/maps_long/map_…/map_….yaml
# 细节见 ../README4.md §0

| name | time | coverage | cost bins | path |
|------|------|----------|-----------|------|
| `arena_trav` | (working) | — | — | `arena_trav.yaml` |
| `map_20260907_230002` | 2026-09-07 23:00:02 | 79.9% known | free=470761 mid=67085 high=1137 lethal=260150 | `map_20260907_230002/map_20260907_230002.yaml` |
