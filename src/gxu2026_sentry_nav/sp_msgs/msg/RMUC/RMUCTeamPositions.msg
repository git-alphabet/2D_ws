# =============================================================================
# RMUCTeamPositions.msg — 己方队友位置 (裁判系统 0x020B)
# =============================================================================
# 话题: /team_positions           频率: 1 Hz
# 来源: 电控/裁判系统串口解析节点
# 订阅: RmucSubTeamPositions → 写入黑板
# 引用: ParseSentryBlackboard → 协同决策
# =============================================================================

std_msgs/Header header

# ── 己方各机器人位置  ──
# 坐标系: 场地围挡在红方补给站附近交点为原点,
#          沿长边向蓝方为X正, 沿短边向红方停机坪为Y正 (单位: m)
float32 hero_x                 # offset 0: 英雄 x
float32 hero_y                 # offset 4: 英雄 y
float32 infantry3_x            # offset 16: 3号步兵 x
float32 infantry3_y            # offset 20: 3号步兵 y

# # ── 以下字段暂未启用 ──
# float32 engi_x                 # offset 8: 工程 x
# float32 engi_y                 # offset 12: 工程 y
# float32 infantry4_x            # offset 24: 4号步兵 x
# float32 infantry4_y            # offset 28: 4号步兵 y
