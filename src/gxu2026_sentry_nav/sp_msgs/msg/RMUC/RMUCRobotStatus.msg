# =============================================================================
# RMUCRobotStatus.msg — 机器人综合状态 (0x0201/0x0202/0x0003/0x0101/0x0105)
# =============================================================================
# 话题: /robot_status          频率: 10 Hz
# 来源: 电控/裁判系统串口解析节点
# 订阅: RmucSubRobotStatus → 写入黑板 {robot_status}
# 引用: ParseSentryBlackboard, IsDead, IsHPBelow, DetectRespawnAndSetRecovery,
#       WaitAndHeal, DecideRespawnCmd
# =============================================================================

std_msgs/Header header

# ── 血量与射击热量 ──
uint16 current_hp            # [2B] 当前血量
uint16 shooter_heat          # [2B] 当前枪口热量

# ── 弹丸与允许发弹量 ──
uint16 ammo_allow            # [2B] 剩余允许发弹量

# ── 视觉检测 ──
bool    is_detect_enemy      # [1B] 是否检测到敌人
bool    captured_outpost     # [1B] 是否占领敌方前哨站

# ── 状态说明 ──
# 注意: is_dead 不是裁判系统协议字段，由 BT (ParseSentryBlackboard) 根据 current_hp <= 0 计算。
# 注意: 脱战状态(is_disengaged)不是裁判系统协议字段，
# 由 ParseSentryBlackboard 根据 shooter_heat 和 current_hp 变化自行计算。
# 规则: 存活状态下连续 6 秒未发射弹丸且未被扣血 = 脱战。

# ── 以下字段暂未启用 ──
# uint16 base_hp_max           # [2B] 己方基地血量上限
# uint16 max_hp                # [2B] 血量上限
# uint16 ammo_left             # [2B] 物理剩余弹丸数
# bool   can_remote_heal       # [1B] 是否可触发远程补血
# bool   can_remote_ammo       # [1B] 是否可触发远程补弹
# team_coins 已移至 RMUCProjectileAllowance.msg (remaining_coins, 0x0208 offset 4)
# uint16 heat_limit            # 热量上限
# uint16 cooling_rate          # 每秒冷却值
# bool    shooter_power_output # 0x0201 bit2: shooter 口 24V 输出 (1=有输出, 0=断电)
#                              # 虚弱状态下裁判系统会关闭 shooter 输出，
#                              # 但热量超限/初速超限/测速离线/允许发弹量=0 也会导致断电，
#                              # 需组合其他字段排除误判，详见 IsWeakness 插件。
# bool   can_respawn           # 复活读条是否完成
# uint8  respawn_countdown_s   # 复活读条倒计时 (秒)