"""
场地以米计。原点附近是建造区，(50, 50) 附近是采集区。开始时两台运输车都停在建造区，机械臂车 R3 在采集区，机械臂车 R4 在建造区。物体 A、B 要从采集区挖出并运到建造区，物体 C 已经放在建造点，不必转运。三件物体按 A、B、C 的顺序垒起来：A 较大，B 适中，C 较小。R3 擅长抓大物体、不擅长小物体，R4 相反。R1 与 R2 运送中型和小型物体的能力相当，大型物体的长途转运必须由两台运输车一起完成。A、B 的采集点相距较远。A 的流程是挖掘、装载、长途运到建造区入口、卸载、再装载、避障运到建造点、卸载、抓取、放置。B 的流程是挖掘、装载、直接长途运到建造点、卸载、抓取、放置。C 只抓取和放置。R1 的导航能力高于 R2，入口到建造点的避障由 R1 执行。运送大物体时，这条流程里各步骤的可靠性下降。deliver 和 nav 不单独定义成任务，相邻任务之间的路程用前一任务的终位置和后一任务的始位置计算。入口到建造点有固定相机，两个采集点到入口有定位锚点；协同任务同时有无线链路和视觉链路，一条失效时另一条仍能对齐各车信念。

要改场景时只改下面的表。compile_case 会补上前置、后置、执行图和求解。
"""
from __future__ import annotations

from hybridx import compile_case, comm_effect, move_distance, segment_effect


# 落点：名字, x 米, y 米, 材质。材质留空表示不登记这个格子。
SITES = [
    ('pit_a', 40.0, 58.0, 'regolith'),
    ('pit_b', 66.0, 36.0, 'regolith'),
    ('gate', 14.0, 6.0, None),
    ('stack', 2.0, 2.0, 'empty'),
    ('wait_c', 5.0, 3.0, 'module_C'),
]

# 机器人：编号, 机型, x, y, 朝向（度）, 电量。
ROBOTS = [
    ('R1', 'transport', 0.0, 0.0, 0.0, 1.0),
    ('R2', 'transport', 3.0, 1.0, 0.0, 1.0),
    ('R3', 'arm', 50.0, 50.0, 90.0, 0.90),
    ('R4', 'arm', 1.5, 4.5, 0.0, 0.90),
]

# 能力：机器人 → 任务类型 → 能力。缺一项就当作 0，不能做那件任务。
CAPS = {
    'R1': {
        'load_a': 0.74, 'unload_a': 0.72,
        'load_nav_a': 0.93, 'unload_nav_a': 0.92,
        'load_b': 0.88, 'unload_b': 0.86,
    },
    'R2': {
        'load_a': 0.73, 'unload_a': 0.71,
        'load_nav_a': 0.68, 'unload_nav_a': 0.66,
        'load_b': 0.87, 'unload_b': 0.85,
    },
    'R3': {
        'dig_a': 0.92, 'load_a': 0.90, 'unload_a': 0.88,
        'load_nav_a': 0.88, 'unload_nav_a': 0.86, 'grasp_a': 0.90, 'place_a': 0.90,
        'dig_b': 0.84, 'load_b': 0.82, 'unload_b': 0.80, 'grasp_b': 0.76, 'place_b': 0.74,
        'grasp_c': 0.42, 'place_c': 0.40,
    },
    'R4': {
        'dig_a': 0.38, 'load_a': 0.40, 'unload_a': 0.42,
        'load_nav_a': 0.44, 'unload_nav_a': 0.44, 'grasp_a': 0.42, 'place_a': 0.40,
        'dig_b': 0.72, 'load_b': 0.80, 'unload_b': 0.84, 'grasp_b': 0.88, 'place_b': 0.86,
        'grasp_c': 0.93, 'place_c': 0.94,
    },
}

# 动作示例：名字, 种类, 左参数, 右参数。
# 点可以写落点名，也可以直接写 (x, y)。t1 的里程由两端落点算出。求解不读这张表。
ACTIONS = [
    ('a1_cost', 'a1', 0.02, None),
    ('a2_to_pit_a', 'a2', (0.0, 0.0), 'pit_a'),
    ('a2_into_zone', 'a2', 'pit_a', 'gate'),
    ('a2_nav_stack', 'a2', 'gate', 'stack'),
    ('a2_direct_b', 'a2', 'pit_b', 'stack'),
    ('a3_belief', 'a3', {'road': 'clear'}, None),
    ('m1_reach', 'm1', (0.0, 0.0, 0.0), (0.4, 0.2, -0.1)),
    ('m1_pick', 'm1', (0.4, 0.2, -0.1), (0.6, 0.3, 0.0)),
    ('m2_grasp', 'm2', 'grasp', None),
    ('m2_release', 'm2', 'release', None),
    ('t1_road_a', 't1', 'pit_a', 'gate'),
    ('t1_nav_a', 't1', 'gate', 'stack'),
    ('t1_road_b', 't1', 'pit_b', 'stack'),
    ('t2_load', 't2', 1.0, None),
    ('t2_unload', 't2', -1.0, None),
]

# 定位设施：名字, 种类, 可靠性, 盖住的落点。
# 一段移动的两端都在落点里，才给这段加上一条感知通路。
FACILITIES = [
    ('cam_yard', 'camera', 0.85, ('gate', 'stack')),
    ('uwb_pit', 'uwb', 0.80, ('pit_a', 'pit_b', 'gate')),
]

# 通信：名字, 可靠性, 至少几台同时在场才起作用。
LINKS = [
    ('wireless', 0.90, 2),
    ('visual', 0.75, 2),
]

# 任务：编号, 类型, 可靠性, 结束落点, 电量门槛, 耗电, 依赖, 角色, 放置时额外清空的落点。
# 角色是 (机型, 槽, 能力阈值)。槽写 '' 表示这一型只有一台；两台运输车用 '1' 和 '2'。
# 类型以 dig / load / unload / grasp / place 开头，后面的状态变化按这个词生成。
# 两台及以上角色时，彼此距离须 ≤ 1。放置把物体写到 stack。
TASKS = [
    ('T_dig_A', 'dig_a', 0.82, 'pit_a', 0.35, 0.05, (), (('arm', '', 0.80),), None),
    ('T_load_A', 'load_a', 0.82, 'pit_a', 0.35, 0.05, ('T_dig_A',), (
        ('transport', '1', 0.65), ('transport', '2', 0.65), ('arm', '', 0.85),
    ), None),
    ('T_unload_A', 'unload_a', 0.82, 'gate', 0.25, 0.05, ('T_load_A',), (
        ('transport', '1', 0.65), ('transport', '2', 0.65), ('arm', '', 0.80),
    ), None),
    ('T_load_nav_A', 'load_nav_a', 0.82, 'gate', 0.25, 0.04, ('T_unload_A',), (
        ('transport', '', 0.80), ('arm', '', 0.80),
    ), None),
    ('T_unload_nav_A', 'unload_nav_a', 0.82, 'stack', 0.20, 0.04, ('T_load_nav_A',), (
        ('transport', '', 0.80), ('arm', '', 0.80),
    ), None),
    ('T_grasp_A', 'grasp_a', 0.82, 'stack', 0.20, 0.03, ('T_unload_nav_A',), (('arm', '', 0.80),), None),
    ('T_place_A', 'place_a', 0.82, 'stack', 0.20, 0.04, ('T_grasp_A',), (('arm', '', 0.85),), None),
    ('T_dig_B', 'dig_b', 0.96, 'pit_b', 0.30, 0.03, (), (('arm', '', 0.78),), None),
    ('T_load_B', 'load_b', 0.96, 'pit_b', 0.30, 0.02, ('T_dig_B',), (
        ('transport', '', 0.80), ('arm', '', 0.78),
    ), None),
    ('T_unload_B', 'unload_b', 0.96, 'stack', 0.20, 0.02, ('T_load_B',), (
        ('transport', '', 0.80), ('arm', '', 0.78),
    ), None),
    ('T_grasp_B', 'grasp_b', 0.96, 'stack', 0.20, 0.02, ('T_unload_B',), (('arm', '', 0.80),), None),
    ('T_place_B', 'place_b', 0.96, 'stack', 0.20, 0.02, ('T_grasp_B', 'T_place_A'), (('arm', '', 0.80),), None),
    ('T_grasp_C', 'grasp_c', 0.97, 'wait_c', 0.20, 0.02, (), (('arm', '', 0.85),), None),
    ('T_place_C', 'place_c', 0.97, 'stack', 0.20, 0.02, ('T_grasp_C', 'T_place_B'), (('arm', '', 0.85),), 'wait_c'),
]

# 生成图：名字, 步骤, 依赖。
# 依赖写 None 时，去掉移动后按顺序连成链。写 () 时各步并行。
# move、move_ang、deliver、nav 留在步骤里，装配时删掉。
GENERATORS = [
    ('carve_a', ('move', 'dig_a', 'move'), None),
    ('ship_a', ('load_a', 'move', 'deliver', 'move', 'unload_a'), None),
    ('navigate_a', ('load_nav_a', 'move', 'nav', 'move', 'unload_nav_a'), None),
    ('assemble_a', ('move_ang', 'grasp_a', 'move_ang', 'place_a', 'move_ang'), None),
    ('build_a', ('carve_a', 'ship_a', 'navigate_a', 'assemble_a'), None),
    ('carve_b', ('move', 'dig_b', 'move'), None),
    ('ship_b', ('load_b', 'move', 'deliver', 'move', 'unload_b'), None),
    ('assemble_b', ('move_ang', 'grasp_b', 'move_ang', 'place_b', 'move_ang'), None),
    ('build_b', ('carve_b', 'ship_b', 'assemble_b'), None),
    ('grasp_c_gen', ('move_ang', 'grasp_c', 'move_ang'), None),
    ('assemble_c', ('move_ang', 'place_c', 'move_ang'), None),
    ('build_c', ('grasp_c_gen', 'assemble_c'), None),
    ('build', ('build_a', 'build_b', 'build_c'), ()),
]

SEED = 1

case = compile_case(
    SITES, ROBOTS, CAPS, ACTIONS, FACILITIES, LINKS, TASKS, GENERATORS, seed=SEED,
)
points = case.points
robots = case.robots
env = case.env
actions = case.actions
facilities = case.facilities
comm_links = case.comm_links
tasks = case.tasks
tasks_by_type = case.tasks_by_type
operad = case.operad
trees = case.trees
translator = case.translator
translated = case.translated
solver = case.solver
plan = case.plan

PIT_A = points['pit_a']
PIT_B = points['pit_b']
GATE = points['gate']
STACK = points['stack']
WAIT_C = points['wait_c']


if __name__ == '__main__':
    print('======== 1.1 机器人 ========')
    for robot in robots.values():
        robot.print()
        print()
    print('======== 1.1 环境 ========')
    env.print()
    print()
    print('======== 1.2 动作 ========')
    for action in actions.values():
        action.print()
    print()
    print('======== 1.3 设施与通信 ========')
    for facility in facilities.values():
        facility.print()
    for link in comm_links.values():
        link.print()
    print()
    present = list(facilities.values())
    segment_effect('pit_a', 'gate', move_distance(PIT_A, GATE), present).print()
    segment_effect('gate', 'stack', move_distance(GATE, STACK), present).print()
    segment_effect('pit_b', 'stack', move_distance(PIT_B, STACK), present).print()
    print()
    crew = list(comm_links.values())
    comm_effect(crew, 3).print()
    comm_effect(crew, 3, failed=('wireless',)).print()
    comm_effect(crew, 3, failed=('wireless', 'visual')).print()
    comm_effect(crew, 1).print()
    print()
    print('======== 2 任务 ========')
    for task in tasks.values():
        task.print()
        print()
    print('======== 3.1 生成规则 ========')
    operad.print()
    print()
    print('======== 3.1 生成图 ========')
    for tree in trees.values():
        tree.print()
        print()
    print('======== 4 翻译 ========')
    for item in translated.values():
        item.print()
        print()
    print('======== 5 求解 ========')
    plan.print()
    print()
