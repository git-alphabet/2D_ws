# =============================================================================
# RMUCEnemyMark.msg — 敌方易伤/己方标识 (裁判系统 0x020C)
# =============================================================================
# 话题: /enemy_mark               频率: 10 Hz
# 来源: 电控/裁判系统串口解析节点
# 订阅: RmucSubEnemyMark → 写入黑板
# 引用: ParseSentryBlackboard → SelectBestTarget
# =============================================================================

std_msgs/Header header

# ── 对方机器人易伤状态  ──
# 标记进度>=100时为true (1), <100时为false (0)
bool   enemy_hero_vuln         # bit0: 对方英雄易伤
bool   enemy_engi_vuln         # bit1: 对方工程易伤
bool   enemy_infantry3_vuln    # bit2: 对方3号步兵易伤
bool   enemy_infantry4_vuln    # bit3: 对方4号步兵易伤
bool   enemy_sentry_vuln       # bit4: 对方哨兵易伤

# ── 己方机器人特殊标识 ──
# 标记进度>=50时为true (1), <50时为false (0)
bool   ally_hero_marked        # bit5: 己方英雄被标记
bool   ally_engi_marked        # bit6: 己方工程被标记
bool   ally_infantry3_marked   # bit7: 己方3号步兵被标记
bool   ally_infantry4_marked   # bit8: 己方4号步兵被标记
bool   ally_sentry_marked      # bit9: 己方哨兵被标记
