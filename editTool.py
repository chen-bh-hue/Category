"""
图形化编辑 hybridx_apply.py。

每页是一张可以直接填写的表：落点、机器人、能力、动作、设施、通信、任务、生成图。
保存前用 compile_case 装配检查，通过后仍按表格写回。
"""
from __future__ import annotations

import ast
import importlib
import os
from pathlib import Path

import hybridx_apply as apply
from hybridx import MOVE_TYPES, compile_case

APPLY_PATH = Path(__file__).resolve().parent / 'hybridx_apply.py'
ROLES = ('transport', 'arm')
ROLE_NAME = {'transport': '运输车', 'arm': '机械臂'}
ACTION_KINDS = ('a1', 'a2', 'a3', 'm1', 'm2', 't1', 't2')
FACILITY_KINDS = ('camera', 'uwb', 'tag')
MOVE_NAMES = tuple(sorted(MOVE_TYPES))
SITE_ALIAS = {
    'pit_a': 'PIT_A',
    'pit_b': 'PIT_B',
    'gate': 'GATE',
    'stack': 'STACK',
    'wait_c': 'WAIT_C',
}


def py_num(value: float) -> str:
    text = f'{float(value):.12g}'
    if 'e' in text or 'E' in text:
        return text
    if '.' not in text:
        return text + '.0'
    return text


def py_obj(value) -> str:
    if isinstance(value, bool) or value is None:
        return repr(value)
    if isinstance(value, float):
        return py_num(value)
    if isinstance(value, int) and not isinstance(value, bool):
        return str(value)
    if isinstance(value, tuple):
        body = ', '.join(py_obj(item) for item in value)
        if len(value) == 1:
            body += ','
        return f'({body})'
    if isinstance(value, list):
        return py_obj(tuple(value))
    if isinstance(value, dict):
        body = ', '.join(f'{py_obj(key)}: {py_obj(item)}' for key, item in value.items())
        return '{' + body + '}'
    return repr(value)


def parse_literal(text: str):
    stripped = text.strip()
    if not stripped:
        return None
    try:
        return ast.literal_eval(stripped)
    except (SyntaxError, ValueError):
        return stripped


def parse_map(text: str) -> list[tuple[str, object]]:
    rows = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if '=' not in line:
            raise ValueError(f'缺少等号: {line}')
        key, value = line.split('=', 1)
        rows.append((key.strip(), parse_literal(value)))
    return rows


def format_map(rows: list[tuple[str, object]]) -> str:
    return '\n'.join(f'{key} = {py_obj(value)}' for key, value in rows)


def load_model(path: Path = APPLY_PATH) -> dict:
    importlib.reload(apply)
    doc = ast.get_docstring(ast.parse(path.read_text(encoding='utf-8'))) or apply.__doc__ or ''
    return {
        'doc': doc.strip(),
        'sites': [
            {'name': name, 'x': float(x), 'y': float(y), 'mat': material}
            for name, x, y, material in apply.SITES
        ],
        'robots': [
            {
                'rid': rid,
                'rtype': rtype,
                'x': float(x),
                'y': float(y),
                'theta_deg': float(degree),
                'bat': float(bat),
                'capability': [
                    (key, float(value))
                    for key, value in apply.CAPS.get(rid, {}).items()
                ],
            }
            for rid, rtype, x, y, degree, bat in apply.ROBOTS
        ],
        'actions': [
            {'name': name, 'kind': kind, 'left': left, 'right': right}
            for name, kind, left, right in apply.ACTIONS
        ],
        'facilities': [
            {
                'name': name,
                'kind': kind,
                'reliability': float(reliability),
                'sites': list(where),
            }
            for name, kind, reliability, where in apply.FACILITIES
        ],
        'comms': [
            {'name': name, 'reliability': float(reliability), 'min_crew': int(min_crew)}
            for name, reliability, min_crew in apply.LINKS
        ],
        'tasks': [
            {
                'tid': tid,
                'ttype': ttype,
                'reliability': float(reliability),
                'site': site,
                'bat_need': float(bat_need),
                'bat_cost': float(bat_cost),
                'deps': list(dep_ids),
                'roles': [
                    {'rtype': rtype, 'slot': slot, 'capability': float(threshold)}
                    for rtype, slot, threshold in role_rows
                ],
                'clear': clear,
            }
            for tid, ttype, reliability, site, bat_need, bat_cost, dep_ids, role_rows, clear in apply.TASKS
        ],
        'generators': [
            {
                'name': name,
                'steps': list(steps),
                'deps': None if deps is None else [tuple(pair) for pair in deps],
            }
            for name, steps, deps in apply.GENERATORS
        ],
        'seed': int(apply.SEED),
    }


def tables_of(model: dict) -> tuple:
    sites = [
        (row['name'], float(row['x']), float(row['y']), row['mat'] or None)
        for row in model['sites']
    ]
    robots = [
        (row['rid'], row['rtype'], float(row['x']), float(row['y']), float(row['theta_deg']), float(row['bat']))
        for row in model['robots']
    ]
    caps = {
        row['rid']: {key: float(value) for key, value in row['capability']}
        for row in model['robots']
    }
    actions = [
        (row['name'], row['kind'], row['left'], row['right'])
        for row in model['actions']
    ]
    facilities = [
        (row['name'], row['kind'], float(row['reliability']), tuple(row['sites']))
        for row in model['facilities']
    ]
    links = [
        (row['name'], float(row['reliability']), int(row['min_crew']))
        for row in model['comms']
    ]
    tasks = [
        (
            row['tid'],
            row['ttype'],
            float(row['reliability']),
            row['site'],
            float(row['bat_need']),
            float(row['bat_cost']),
            tuple(row['deps']),
            tuple((item['rtype'], item.get('slot') or '', float(item['capability'])) for item in row['roles']),
            row['clear'] or None,
        )
        for row in model['tasks']
    ]
    generators = []
    for item in model['generators']:
        if item['deps'] is None:
            generators.append((item['name'], tuple(item['steps']), None))
        else:
            generators.append((item['name'], tuple(item['steps']), tuple(tuple(pair) for pair in item['deps'])))
    return sites, robots, caps, actions, facilities, links, tasks, generators, int(model.get('seed', 1))


def validate(model: dict) -> None:
    if not any(item['name'] == 'build' for item in model['generators']):
        raise ValueError('生成图里必须有名为 build 的规则，求解从它开始')
    names = [row['name'] for row in model['sites']]
    if len(names) != len(set(names)):
        raise ValueError('落点名称重复')
    rids = [row['rid'] for row in model['robots']]
    if len(rids) != len(set(rids)):
        raise ValueError('机器人编号重复')
    tids = [row['tid'] for row in model['tasks']]
    types = [row['ttype'] for row in model['tasks']]
    if len(tids) != len(set(tids)):
        raise ValueError('任务编号重复')
    if len(types) != len(set(types)):
        raise ValueError('任务类型重复')
    action_names = [row['name'] for row in model['actions']]
    if len(action_names) != len(set(action_names)):
        raise ValueError('动作名称重复')
    compile_case(*tables_of(model)[:8], seed=int(model.get('seed', 1)))


def _tuple_lines(rows: list[str], indent: str = '    ') -> str:
    if not rows:
        return ''
    return '\n'.join(f'{indent}{line}' for line in rows)


def _site_code(row: dict) -> str:
    mat = py_obj(row['mat'] or None)
    return f"({py_obj(row['name'])}, {py_num(row['x'])}, {py_num(row['y'])}, {mat}),"


def _robot_code(row: dict) -> str:
    return (
        f"({py_obj(row['rid'])}, {py_obj(row['rtype'])}, "
        f"{py_num(row['x'])}, {py_num(row['y'])}, {py_num(row['theta_deg'])}, {py_num(row['bat'])}),"
    )


def _caps_code(robots: list[dict]) -> str:
    blocks = []
    for row in robots:
        items = row['capability']
        if not items:
            inner = '{}'
        elif len(items) <= 2:
            inner = '{ ' + ', '.join(f'{py_obj(key)}: {py_num(value)}' for key, value in items) + ' }'
        else:
            lines = [f'        {py_obj(key)}: {py_num(value)},' for key, value in items]
            inner = '{\n' + '\n'.join(lines) + '\n    }'
        blocks.append(f"    {py_obj(row['rid'])}: {inner},")
    return '\n'.join(blocks)


def _action_code(row: dict) -> str:
    return f"({py_obj(row['name'])}, {py_obj(row['kind'])}, {py_obj(row['left'])}, {py_obj(row['right'])}),"


def _facility_code(row: dict) -> str:
    return (
        f"({py_obj(row['name'])}, {py_obj(row['kind'])}, {py_num(row['reliability'])}, "
        f"{py_obj(tuple(row['sites']))}),"
    )


def _comm_code(row: dict) -> str:
    return f"({py_obj(row['name'])}, {py_num(row['reliability'])}, {int(row['min_crew'])}),"


def _task_code(row: dict) -> str:
    deps = py_obj(tuple(row['deps']))
    roles = tuple(
        (item['rtype'], item.get('slot') or '', float(item['capability']))
        for item in row['roles']
    )
    clear = py_obj(row['clear'] or None)
    return (
        f"({py_obj(row['tid'])}, {py_obj(row['ttype'])}, {py_num(row['reliability'])}, "
        f"{py_obj(row['site'])}, {py_num(row['bat_need'])}, {py_num(row['bat_cost'])}, "
        f"{deps}, {py_obj(roles)}, {clear}),"
    )


def _generator_code(row: dict) -> str:
    steps = py_obj(tuple(row['steps']))
    if row['deps'] is None:
        deps = 'None'
    else:
        deps = py_obj(tuple(tuple(pair) for pair in row['deps']))
    return f"({py_obj(row['name'])}, {steps}, {deps}),"


def render(model: dict) -> str:
    doc = (model.get('doc') or 'hybridx.py 的具体定义').strip()
    aliases = []
    names = {row['name'] for row in model['sites']}
    for site, alias in SITE_ALIAS.items():
        if site in names:
            aliases.append(f"{alias} = points[{py_obj(site)}]")
    parts = [
        '"""',
        doc,
        '"""',
        'from __future__ import annotations',
        '',
        'from hybridx import compile_case, comm_effect, move_distance, segment_effect',
        '',
        '',
        '# 落点：名字, x 米, y 米, 材质。材质留空表示不登记这个格子。',
        'SITES = [',
        _tuple_lines([_site_code(row) for row in model['sites']]),
        ']',
        '',
        '# 机器人：编号, 机型, x, y, 朝向（度）, 电量。',
        'ROBOTS = [',
        _tuple_lines([_robot_code(row) for row in model['robots']]),
        ']',
        '',
        '# 能力：机器人 → 任务类型 → 能力。缺一项就当作 0，不能做那件任务。',
        'CAPS = {',
        _caps_code(model['robots']),
        '}',
        '',
        '# 动作示例：名字, 种类, 左参数, 右参数。',
        '# 点可以写落点名，也可以直接写 (x, y)。t1 的里程由两端落点算出。求解不读这张表。',
        'ACTIONS = [',
        _tuple_lines([_action_code(row) for row in model['actions']]),
        ']',
        '',
        '# 定位设施：名字, 种类, 可靠性, 盖住的落点。',
        '# 一段移动的两端都在落点里，才给这段加上一条感知通路。',
        'FACILITIES = [',
        _tuple_lines([_facility_code(row) for row in model['facilities']]),
        ']',
        '',
        '# 通信：名字, 可靠性, 至少几台同时在场才起作用。',
        'LINKS = [',
        _tuple_lines([_comm_code(row) for row in model['comms']]),
        ']',
        '',
        '# 任务：编号, 类型, 可靠性, 结束落点, 电量门槛, 耗电, 依赖, 角色, 放置时额外清空的落点。',
        "# 角色是 (机型, 槽, 能力阈值)。槽写 '' 表示这一型只有一台；两台运输车用 '1' 和 '2'。",
        '# 类型以 dig / load / unload / grasp / place 开头，后面的状态变化按这个词生成。',
        '# 两台及以上角色时，彼此距离须 ≤ 1。放置把物体写到 stack。',
        'TASKS = [',
        _tuple_lines([_task_code(row) for row in model['tasks']]),
        ']',
        '',
        '# 生成图：名字, 步骤, 依赖。',
        '# 依赖写 None 时，去掉移动后按顺序连成链。写 () 时各步并行。',
        '# move、move_ang、deliver、nav 留在步骤里，装配时删掉。',
        'GENERATORS = [',
        _tuple_lines([_generator_code(row) for row in model['generators']]),
        ']',
        '',
        f'SEED = {int(model.get("seed", 1))}',
        '',
        'case = compile_case(',
        '    SITES, ROBOTS, CAPS, ACTIONS, FACILITIES, LINKS, TASKS, GENERATORS, seed=SEED,',
        ')',
        'points = case.points',
        'robots = case.robots',
        'env = case.env',
        'actions = case.actions',
        'facilities = case.facilities',
        'comm_links = case.comm_links',
        'tasks = case.tasks',
        'tasks_by_type = case.tasks_by_type',
        'operad = case.operad',
        'trees = case.trees',
        'translator = case.translator',
        'translated = case.translated',
        'solver = case.solver',
        'plan = case.plan',
        '',
        *aliases,
        '',
        '',
        "if __name__ == '__main__':",
        "    print('======== 1.1 机器人 ========')",
        '    for robot in robots.values():',
        '        robot.print()',
        '        print()',
        "    print('======== 1.1 环境 ========')",
        '    env.print()',
        '    print()',
        "    print('======== 1.2 动作 ========')",
        '    for action in actions.values():',
        '        action.print()',
        '    print()',
        "    print('======== 1.3 设施与通信 ========')",
        '    for facility in facilities.values():',
        '        facility.print()',
        '    for link in comm_links.values():',
        '        link.print()',
        '    print()',
        '    present = list(facilities.values())',
        "    segment_effect('pit_a', 'gate', move_distance(PIT_A, GATE), present).print()",
        "    segment_effect('gate', 'stack', move_distance(GATE, STACK), present).print()",
        "    segment_effect('pit_b', 'stack', move_distance(PIT_B, STACK), present).print()",
        '    print()',
        '    crew = list(comm_links.values())',
        '    comm_effect(crew, 3).print()',
        "    comm_effect(crew, 3, failed=('wireless',)).print()",
        "    comm_effect(crew, 3, failed=('wireless', 'visual')).print()",
        '    comm_effect(crew, 1).print()',
        '    print()',
        "    print('======== 2 任务 ========')",
        '    for task in tasks.values():',
        '        task.print()',
        '        print()',
        "    print('======== 3.1 生成规则 ========')",
        '    operad.print()',
        '    print()',
        "    print('======== 3.1 生成图 ========')",
        '    for tree in trees.values():',
        '        tree.print()',
        '        print()',
        "    print('======== 4 翻译 ========')",
        '    for item in translated.values():',
        '        item.print()',
        '        print()',
        "    print('======== 5 求解 ========')",
        '    plan.print()',
        '    print()',
        '',
    ]
    return '\n'.join(parts)


def save_to(path: Path, model: dict) -> None:
    validate(model)
    path.write_text(render(model), encoding='utf-8')


def launch() -> None:
    import tkinter as tk
    from tkinter import messagebox, ttk

    font = ('Microsoft YaHei UI', 10)
    model = load_model()
    root = tk.Tk()
    root.title('HybridX 应用编辑器')
    root.geometry('1360x820')
    root.minsize(980, 640)

    status = tk.StringVar(value=str(APPLY_PATH))
    seed_var = tk.StringVar(value=str(model['seed']))
    refreshers = []
    flushers = []

    def fail(exc: Exception) -> None:
        messagebox.showerror('无法保存', str(exc), parent=root)

    def flush_all() -> None:
        for flusher in flushers:
            flusher()

    def reload_model() -> None:
        nonlocal model
        try:
            model = load_model()
        except Exception as exc:
            fail(exc)
            return
        seed_var.set(str(model['seed']))
        for refresher in refreshers:
            refresher()
        status.set('已重新加载')

    def save_model() -> None:
        try:
            flush_all()
            model['seed'] = int(seed_var.get())
            save_to(APPLY_PATH, model)
        except Exception as exc:
            fail(exc)
            return
        status.set('已写入 ' + str(APPLY_PATH))
        messagebox.showinfo('已保存', f'已写回\n{APPLY_PATH}', parent=root)

    def preview() -> None:
        try:
            flush_all()
            model['seed'] = int(seed_var.get())
            text = render(model)
            validate(model)
        except Exception as exc:
            fail(exc)
            return
        window = tk.Toplevel(root)
        window.title('将写入的内容')
        window.geometry('860x640')
        box = tk.Text(window, wrap='none', font=('Consolas', 10))
        box.pack(fill='both', expand=True)
        box.insert('1.0', text)

    bar = ttk.Frame(root, padding=6)
    bar.pack(fill='x')
    ttk.Button(bar, text='重新加载', command=reload_model).pack(side='left')
    ttk.Button(bar, text='预览', command=preview).pack(side='left', padx=6)
    ttk.Button(bar, text='保存', command=save_model).pack(side='left')
    ttk.Label(bar, text='种子').pack(side='left', padx=(16, 4))
    ttk.Entry(bar, textvariable=seed_var, width=8).pack(side='left')
    ttk.Label(bar, textvariable=status).pack(side='left', padx=12)

    book = ttk.Notebook(root)
    book.pack(fill='both', expand=True, padx=6, pady=6)

    def book_page(title: str):
        frame = ttk.Frame(book, padding=8)
        book.add(frame, text=title)
        return frame

    class Sheet:
        """一页可直接填写的表。columns 是 (标题, 宽度, 读出, 写入, 下拉选项或 None)。"""

        def __init__(self, parent, columns, blank, hint: str = ''):
            self.columns = columns
            self.blank = blank
            self.rows: list = []
            self.cells: list = []
            outer = ttk.Frame(parent)
            outer.pack(fill='both', expand=True)
            if hint:
                ttk.Label(outer, text=hint).pack(anchor='w', pady=(0, 4))
            tools = ttk.Frame(outer)
            tools.pack(fill='x', pady=(0, 4))
            ttk.Button(tools, text='加一行', command=self.add).pack(side='left')
            wrap = ttk.Frame(outer)
            wrap.pack(fill='both', expand=True)
            self.canvas = tk.Canvas(wrap, highlightthickness=0)
            ybar = ttk.Scrollbar(wrap, orient='vertical', command=self.canvas.yview)
            xbar = ttk.Scrollbar(wrap, orient='horizontal', command=self.canvas.xview)
            self.canvas.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
            ybar.pack(side='right', fill='y')
            xbar.pack(side='bottom', fill='x')
            self.canvas.pack(side='left', fill='both', expand=True)
            self.body = ttk.Frame(self.canvas)
            self.window = self.canvas.create_window((0, 0), window=self.body, anchor='nw')
            self.body.bind('<Configure>', self._fit)
            self.canvas.bind('<Enter>', self._arm_wheel)
            self.canvas.bind('<Leave>', self._disarm_wheel)

        def _fit(self, _event=None) -> None:
            self.canvas.configure(scrollregion=self.canvas.bbox('all'))

        def _wheel(self, event) -> None:
            self.canvas.yview_scroll(int(-event.delta / 120), 'units')

        def _arm_wheel(self, _event) -> None:
            self.canvas.bind_all('<MouseWheel>', self._wheel)

        def _disarm_wheel(self, _event) -> None:
            self.canvas.unbind_all('<MouseWheel>')

        def load(self, rows: list) -> None:
            self.rows = rows
            self.draw()

        def flush(self) -> None:
            for row, widgets in zip(self.rows, self.cells):
                for column, widget in zip(self.columns, widgets):
                    column[3](row, widget.get())

        def draw(self) -> None:
            for child in self.body.winfo_children():
                child.destroy()
            self.cells = []
            for index, column in enumerate(self.columns):
                ttk.Label(self.body, text=column[0]).grid(row=0, column=index + 1, sticky='w', padx=3, pady=(0, 4))
            for row_index, row in enumerate(self.rows):
                ttk.Button(
                    self.body, text='删除', width=5,
                    command=lambda index=row_index: self.remove(index),
                ).grid(row=row_index + 1, column=0, padx=2, pady=1)
                widgets = []
                for col_index, column in enumerate(self.columns):
                    var = tk.StringVar(value=column[2](row))
                    options = column[4] if len(column) > 4 else None
                    if options:
                        widget = ttk.Combobox(
                            self.body, textvariable=var, values=options, width=column[1], font=font,
                        )
                    else:
                        widget = ttk.Entry(self.body, textvariable=var, width=column[1], font=font)
                    widget.grid(row=row_index + 1, column=col_index + 1, padx=2, pady=1, sticky='ew')
                    widgets.append(var)
                self.cells.append(widgets)
            self.body.update_idletasks()
            self._fit()

        def add(self) -> None:
            self.flush()
            self.rows.append(self.blank())
            self.draw()

        def remove(self, index: int) -> None:
            self.flush()
            del self.rows[index]
            self.draw()

    def text_of(value) -> str:
        if value is None:
            return ''
        if isinstance(value, str):
            return value
        return py_obj(value)

    def set_text(row, key, text: str) -> None:
        row[key] = text.strip()

    def set_float(row, key, text: str) -> None:
        row[key] = float(text)

    def set_optional(row, key, text: str) -> None:
        stripped = text.strip()
        row[key] = None if stripped in ('', 'None', 'none') else stripped

    def bind(table, key: str) -> None:
        refreshers.append(lambda table=table, key=key: table.load(model[key]))
        flushers.append(table.flush)

    sites = Sheet(
        book_page('落点'),
        [
            ('名字', 14, lambda r: r['name'], lambda r, t: set_text(r, 'name', t)),
            ('x / 米', 10, lambda r: py_num(r['x']), lambda r, t: set_float(r, 'x', t)),
            ('y / 米', 10, lambda r: py_num(r['y']), lambda r, t: set_float(r, 'y', t)),
            ('材质', 14, lambda r: '' if r['mat'] is None else str(r['mat']), lambda r, t: set_optional(r, 'mat', t)),
        ],
        lambda: {'name': 'site', 'x': 0.0, 'y': 0.0, 'mat': None},
        '材质留空表示这个格子不登记。放置写到名为 stack 的落点。',
    )
    sites.load(model['sites'])
    bind(sites, 'sites')

    robots = Sheet(
        book_page('机器人'),
        [
            ('编号', 10, lambda r: r['rid'], lambda r, t: set_text(r, 'rid', t)),
            ('机型', 12, lambda r: r['rtype'], lambda r, t: set_text(r, 'rtype', t), ROLES),
            ('x / 米', 10, lambda r: py_num(r['x']), lambda r, t: set_float(r, 'x', t)),
            ('y / 米', 10, lambda r: py_num(r['y']), lambda r, t: set_float(r, 'y', t)),
            ('朝向 / 度', 10, lambda r: py_num(r['theta_deg']), lambda r, t: set_float(r, 'theta_deg', t)),
            ('电量', 8, lambda r: py_num(r['bat']), lambda r, t: set_float(r, 'bat', t)),
        ],
        lambda: {'rid': 'R', 'rtype': 'transport', 'x': 0.0, 'y': 0.0, 'theta_deg': 0.0, 'bat': 1.0, 'capability': []},
        '朝向用角度。能力在下一页按任务类型填写，空格表示不会做。',
    )
    robots.load(model['robots'])
    bind(robots, 'robots')

    caps_page = book_page('能力')
    caps_hint = ttk.Label(caps_page, text='行是机器人，列是任务类型。格子里填 0 到 1 的能力，留空表示不会做这件任务。')
    caps_hint.pack(anchor='w', pady=(0, 4))
    caps_wrap = ttk.Frame(caps_page)
    caps_wrap.pack(fill='both', expand=True)
    caps_canvas = tk.Canvas(caps_wrap, highlightthickness=0)
    caps_y = ttk.Scrollbar(caps_wrap, orient='vertical', command=caps_canvas.yview)
    caps_x = ttk.Scrollbar(caps_wrap, orient='horizontal', command=caps_canvas.xview)
    caps_canvas.configure(yscrollcommand=caps_y.set, xscrollcommand=caps_x.set)
    caps_y.pack(side='right', fill='y')
    caps_x.pack(side='bottom', fill='x')
    caps_canvas.pack(side='left', fill='both', expand=True)
    caps_body = ttk.Frame(caps_canvas)
    caps_canvas.create_window((0, 0), window=caps_body, anchor='nw')
    caps_body.bind('<Configure>', lambda _e: caps_canvas.configure(scrollregion=caps_canvas.bbox('all')))
    caps_state = {'vars': [], 'rids': [], 'types': []}

    def task_types() -> list[str]:
        found = []
        for row in model['tasks']:
            if row['ttype'] and row['ttype'] not in found:
                found.append(row['ttype'])
        return found

    def draw_caps() -> None:
        for child in caps_body.winfo_children():
            child.destroy()
        types = task_types()
        caps_state['types'] = types
        caps_state['rids'] = [row['rid'] for row in model['robots']]
        caps_state['vars'] = []
        ttk.Label(caps_body, text='机器人').grid(row=0, column=0, sticky='w', padx=3, pady=(0, 4))
        for index, ttype in enumerate(types):
            ttk.Label(caps_body, text=ttype).grid(row=0, column=index + 1, sticky='w', padx=3, pady=(0, 4))
        for row_index, robot in enumerate(model['robots']):
            known = dict(robot['capability'])
            ttk.Label(caps_body, text=robot['rid']).grid(row=row_index + 1, column=0, sticky='w', padx=3, pady=1)
            row_vars = []
            for col_index, ttype in enumerate(types):
                value = known.get(ttype, '')
                var = tk.StringVar(value='' if value == '' else py_num(float(value)))
                ttk.Entry(caps_body, textvariable=var, width=8, font=font).grid(
                    row=row_index + 1, column=col_index + 1, padx=2, pady=1,
                )
                row_vars.append(var)
            caps_state['vars'].append(row_vars)

    def flush_caps() -> None:
        by_rid = {row['rid']: row for row in model['robots']}
        for rid, row_vars in zip(caps_state['rids'], caps_state['vars']):
            robot = by_rid.get(rid)
            if robot is None:
                continue
            items = []
            for ttype, var in zip(caps_state['types'], row_vars):
                text = var.get().strip()
                if text:
                    items.append((ttype, float(text)))
            robot['capability'] = items

    draw_caps()
    refreshers.append(draw_caps)
    flushers.append(flush_caps)

    def action_write(key):
        def write(row, text: str) -> None:
            row[key] = parse_literal(text)
        return write

    actions = Sheet(
        book_page('动作'),
        [
            ('名称', 16, lambda r: r['name'], lambda r, t: set_text(r, 'name', t)),
            ('种类', 8, lambda r: r['kind'], lambda r, t: set_text(r, 'kind', t), ACTION_KINDS),
            ('左参数', 22, lambda r: text_of(r['left']), action_write('left')),
            ('右参数', 22, lambda r: text_of(r['right']), action_write('right')),
        ],
        lambda: {'name': 'a_new', 'kind': 'a1', 'left': 0.01, 'right': None},
        '点可以填落点名，或 (x, y)。a1、m2、t2 的右参数留空。求解不读这张表。',
    )
    actions.load(model['actions'])
    bind(actions, 'actions')

    def sites_write(row, text: str) -> None:
        row['sites'] = [part.strip() for part in text.split(',') if part.strip()]

    facilities = Sheet(
        book_page('设施'),
        [
            ('名字', 14, lambda r: r['name'], lambda r, t: set_text(r, 'name', t)),
            ('种类', 10, lambda r: r['kind'], lambda r, t: set_text(r, 'kind', t), FACILITY_KINDS),
            ('可靠性', 8, lambda r: py_num(r['reliability']), lambda r, t: set_float(r, 'reliability', t)),
            ('盖住的落点', 28, lambda r: ', '.join(r['sites']), sites_write),
        ],
        lambda: {'name': 'fac', 'kind': 'camera', 'reliability': 0.8, 'sites': ['gate', 'stack']},
        '落点用逗号分隔。一段移动的两端都在里面，这条设施才作用在该段上。',
    )
    facilities.load(model['facilities'])
    bind(facilities, 'facilities')

    comms = Sheet(
        book_page('通信'),
        [
            ('名字', 14, lambda r: r['name'], lambda r, t: set_text(r, 'name', t)),
            ('可靠性', 8, lambda r: py_num(r['reliability']), lambda r, t: set_float(r, 'reliability', t)),
            ('至少几台', 10, lambda r: str(r['min_crew']), lambda r, t: r.__setitem__('min_crew', int(t))),
        ],
        lambda: {'name': 'link', 'reliability': 0.8, 'min_crew': 2},
        '至少两台机器人同时在场时，这条链路才参与。同一任务上的多条链路互为备用。',
    )
    comms.load(model['comms'])
    bind(comms, 'comms')

    task_page = book_page('任务')
    role_tools = ttk.Frame(task_page)
    role_tools.pack(fill='x', pady=(0, 4))
    task_groups = {'n': max([3, *(len(row['roles']) for row in model['tasks'])] or [3])}

    def task_columns(groups: int):
        columns = [
            ('编号', 12, lambda r: r['tid'], lambda r, t: set_text(r, 'tid', t)),
            ('类型', 14, lambda r: r['ttype'], lambda r, t: set_text(r, 'ttype', t)),
            ('可靠性', 8, lambda r: py_num(r['reliability']), lambda r, t: set_float(r, 'reliability', t)),
            ('结束落点', 12, lambda r: r['site'], lambda r, t: set_text(r, 'site', t)),
            ('电量门槛', 8, lambda r: py_num(r['bat_need']), lambda r, t: set_float(r, 'bat_need', t)),
            ('耗电', 8, lambda r: py_num(r['bat_cost']), lambda r, t: set_float(r, 'bat_cost', t)),
            ('依赖', 22, lambda r: ', '.join(r['deps']), lambda r, t: r.__setitem__('deps', [part.strip() for part in t.split(',') if part.strip()])),
            ('清空落点', 12, lambda r: '' if not r['clear'] else str(r['clear']), lambda r, t: set_optional(r, 'clear', t)),
        ]
        for index in range(groups):
            def role(row, index=index):
                if index >= len(row['roles']):
                    return {'rtype': '', 'slot': '', 'capability': ''}
                item = row['roles'][index]
                return item

            columns.append((
                f'机型{index + 1}', 12,
                lambda r, index=index: role(r, index)['rtype'],
                lambda r, t, index=index: role_field(r, index, 'rtype', t.strip()),
                ('',) + ROLES,
            ))
            columns.append((
                f'槽{index + 1}', 6,
                lambda r, index=index: role(r, index).get('slot') or '',
                lambda r, t, index=index: role_field(r, index, 'slot', t.strip()),
            ))
            columns.append((
                f'阈值{index + 1}', 8,
                lambda r, index=index: '' if role(r, index)['capability'] == '' else py_num(float(role(r, index)['capability'])),
                lambda r, t, index=index: role_field(r, index, 'capability', '' if not t.strip() else float(t)),
            ))
        return columns

    def role_field(row, index: int, key: str, value) -> None:
        while len(row['roles']) <= index:
            row['roles'].append({'rtype': '', 'slot': '', 'capability': ''})
        row['roles'][index][key] = value

    def compact_roles(row) -> None:
        kept = []
        for item in row['roles']:
            if not str(item.get('rtype') or '').strip():
                continue
            kept.append({
                'rtype': item['rtype'],
                'slot': item.get('slot') or '',
                'capability': float(item['capability']),
            })
        row['roles'] = kept

    tasks = Sheet(
        task_page,
        task_columns(task_groups['n']),
        lambda: {
            'tid': 'T_new', 'ttype': 'dig_x', 'reliability': 1.0, 'site': 'stack',
            'bat_need': 0.2, 'bat_cost': 0.02, 'deps': [], 'clear': None,
            'roles': [{'rtype': 'arm', 'slot': '', 'capability': 0.8}],
        },
        '类型以 dig、load、unload、grasp、place 开头。依赖填任务编号，逗号分隔。槽 1 和 2 表示两台运输车。',
    )

    def flush_tasks() -> None:
        tasks.flush()
        for row in model['tasks']:
            compact_roles(row)

    def reload_tasks() -> None:
        task_groups['n'] = max([task_groups['n'], 3, *(len(row['roles']) for row in model['tasks'])] or [3])
        tasks.columns = task_columns(task_groups['n'])
        tasks.load(model['tasks'])

    def add_role_group() -> None:
        flush_tasks()
        task_groups['n'] += 1
        tasks.columns = task_columns(task_groups['n'])
        tasks.draw()

    ttk.Button(role_tools, text='加一组角色', command=add_role_group).pack(anchor='w')
    tasks.load(model['tasks'])
    refreshers.append(reload_tasks)
    flushers.append(flush_tasks)

    def steps_write(row, text: str) -> None:
        row['steps'] = [part.strip() for part in text.split(',') if part.strip()]

    def deps_text(row) -> str:
        if row['deps'] is None:
            return ''
        if row['deps'] == []:
            return '并行'
        return ', '.join(f'{left}-{right}' for left, right in row['deps'])

    def deps_write(row, text: str) -> None:
        stripped = text.strip()
        if not stripped:
            row['deps'] = None
            return
        if stripped in ('并行', '()'):
            row['deps'] = []
            return
        pairs = []
        for part in stripped.split(','):
            left, right = part.strip().split('-')
            pairs.append((int(left), int(right)))
        row['deps'] = pairs

    generators = Sheet(
        book_page('生成图'),
        [
            ('名称', 16, lambda r: r['name'], lambda r, t: set_text(r, 'name', t)),
            ('步骤', 52, lambda r: ', '.join(r['steps']), steps_write),
            ('依赖', 18, deps_text, deps_write),
        ],
        lambda: {'name': 'rule', 'steps': [], 'deps': None},
        '步骤用逗号分隔，移动词会保留。依赖留空是按顺序连成链，填“并行”则各步互不连接，也可填 0-1, 1-2。',
    )
    generators.load(model['generators'])
    bind(generators, 'generators')

    def on_tab(_event=None) -> None:
        try:
            flush_all()
        except Exception as exc:
            fail(exc)
            return
        if book.tab(book.select(), 'text') == '能力':
            draw_caps()

    book.bind('<<NotebookTabChanged>>', on_tab)

    closed = {'done': False}

    def close() -> None:
        if closed['done']:
            return
        closed['done'] = True
        try:
            root.unbind_all('<MouseWheel>')
        except tk.TclError:
            pass
        root.quit()
        try:
            root.destroy()
        except tk.TclError:
            pass

    root.protocol('WM_DELETE_WINDOW', close)
    root.bind('<Escape>', lambda _event: close())
    if os.environ.get('EDITTOOL_SMOKE'):
        def smoke() -> None:
            flush_all()
            validate(model)
            close()

        root.after(300, smoke)
    root.mainloop()


if __name__ == '__main__':
    launch()
