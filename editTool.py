"""
图形化编辑 hybridx_apply.py。

按落点、机器人、动作、设施、通信、任务、生成图分页改表。
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

    model = load_model()
    root = tk.Tk()
    root.title('HybridX 应用编辑器')
    root.geometry('1180x760')
    root.minsize(980, 640)

    status = tk.StringVar(value=str(APPLY_PATH))
    seed_var = tk.StringVar(value=str(model['seed']))

    def fail(exc: Exception) -> None:
        messagebox.showerror('无法保存', str(exc), parent=root)

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
    refreshers = []
    flushers = []

    def flush_all() -> None:
        for flusher in flushers:
            flusher()

    def add_editor(title: str, builder) -> None:
        frame = ttk.Frame(book, padding=8)
        book.add(frame, text=title)
        refresh, flush = builder(frame)
        refreshers.append(refresh)
        flushers.append(flush)

    def section_list(parent, rows, label, on_select, on_add, on_delete):
        left = ttk.Frame(parent)
        left.pack(side='left', fill='y', padx=(0, 8))
        ttk.Label(left, text=label).pack(anchor='w')
        listing = tk.Listbox(left, width=28, exportselection=False, font=('Microsoft YaHei UI', 10))
        listing.pack(fill='y', expand=True)
        buttons = ttk.Frame(left)
        buttons.pack(fill='x', pady=4)
        ttk.Button(buttons, text='添加', command=on_add).pack(side='left')
        ttk.Button(buttons, text='删除', command=on_delete).pack(side='left', padx=4)
        busy = {'on': False}

        def fill() -> None:
            busy['on'] = True
            keep = listing.curselection()
            listing.delete(0, 'end')
            for row in rows():
                listing.insert('end', label_of(row))
            if keep and keep[0] < listing.size():
                listing.selection_clear(0, 'end')
                listing.selection_set(keep[0])
            busy['on'] = False

        def current():
            sel = listing.curselection()
            if not sel:
                return None
            return rows()[sel[0]]

        def on_list(_event) -> None:
            if busy['on']:
                return
            try:
                on_select(current())
            except Exception as exc:
                messagebox.showerror('这一项还不能切换', str(exc), parent=root)

        listing.bind('<<ListboxSelect>>', on_list)
        return listing, fill, current

    def label_of(row) -> str:
        if 'rid' in row:
            return f"{row['rid']}  {ROLE_NAME.get(row['rtype'], row['rtype'])}"
        if 'tid' in row:
            return f"{row['tid']}  {row['ttype']}"
        if 'kind' in row and 'name' in row and 'left' in row:
            return f"{row['name']}  {row['kind']}"
        if 'kind' in row and 'name' in row and 'sites' in row:
            return f"{row['name']}  {row['kind']}"
        if 'min_crew' in row:
            return row['name']
        if 'steps' in row:
            return row['name']
        if 'mat' in row:
            return row['name']
        return str(row.get('name', row))

    def ask(title, fields_spec):
        dialog = tk.Toplevel(root)
        dialog.title(title)
        dialog.transient(root)
        values = {}
        for index, (key, caption, options) in enumerate(fields_spec):
            ttk.Label(dialog, text=caption).grid(row=index, column=0, sticky='w', padx=8, pady=4)
            var = tk.StringVar()
            if options:
                widget = ttk.Combobox(dialog, textvariable=var, values=options, state='readonly', width=28)
                var.set(options[0])
            else:
                widget = ttk.Entry(dialog, textvariable=var, width=30)
            widget.grid(row=index, column=1, padx=8, pady=4)
            values[key] = var
        chosen = {'ok': False}

        def accept() -> None:
            chosen['ok'] = True
            dialog.destroy()

        ttk.Button(dialog, text='确定', command=accept).grid(row=len(fields_spec), column=1, sticky='e', padx=8, pady=8)
        dialog.grab_set()
        dialog.wait_window()
        if not chosen['ok']:
            return None
        return {key: var.get().strip() for key, var in values.items()}

    def build_sites(parent):
        listing, fill, current = section_list(
            parent, lambda: model['sites'], '落点',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        fields = {key: tk.StringVar() for key in ('name', 'x', 'y', 'mat')}
        shown = {'row': None}
        captions = (('name', '名字'), ('x', 'x / 米'), ('y', 'y / 米'), ('mat', '材质，可空'))
        for index, (key, caption) in enumerate(captions):
            ttk.Label(form, text=caption).grid(row=index, column=0, sticky='w', pady=2)
            ttk.Entry(form, textvariable=fields[key], width=24).grid(row=index, column=1, sticky='w')
        ttk.Label(form, text='放置写到名为 stack 的落点。').grid(row=4, column=0, columnspan=2, sticky='w', pady=8)

        def show(row) -> None:
            flush()
            shown['row'] = row
            if row is None:
                return
            fields['name'].set(row['name'])
            fields['x'].set(str(row['x']))
            fields['y'].set(str(row['y']))
            fields['mat'].set('' if row['mat'] is None else str(row['mat']))

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['sites']:
                return
            row['name'] = fields['name'].get().strip()
            row['x'] = float(fields['x'].get())
            row['y'] = float(fields['y'].get())
            mat = fields['mat'].get().strip()
            row['mat'] = None if not mat or mat in ('None', 'none') else mat
            fill()

        def add() -> None:
            flush()
            model['sites'].append({'name': f'site_{len(model["sites"]) + 1}', 'x': 0.0, 'y': 0.0, 'mat': None})
            fill()
            listing.selection_set('end')
            show(model['sites'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['sites'][sel[0]]
            fill()

        fill()
        if model['sites']:
            listing.selection_set(0)
            show(model['sites'][0])
        return fill, flush

    def build_robots(parent):
        listing, fill, current = section_list(
            parent, lambda: model['robots'], '机器人',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        fields = {}
        shown = {'row': None}

        def entry(caption, key, width=18):
            ttk.Label(form, text=caption).grid(row=len(fields), column=0, sticky='w', pady=2)
            var = tk.StringVar()
            ttk.Entry(form, textvariable=var, width=width).grid(row=len(fields), column=1, sticky='w')
            fields[key] = var

        entry('编号', 'rid')
        ttk.Label(form, text='机型').grid(row=len(fields), column=0, sticky='w')
        kind = tk.StringVar()
        ttk.Combobox(form, textvariable=kind, values=ROLES, state='readonly', width=16).grid(row=len(fields), column=1, sticky='w')
        fields['rtype'] = kind
        for key, caption in (('x', 'x / 米'), ('y', 'y / 米'), ('theta_deg', '朝向 / 度'), ('bat', '电量')):
            entry(caption, key)
        ttk.Label(form, text='能力  类型 = 数值').grid(row=len(fields), column=0, sticky='nw', pady=(8, 0))
        capability = tk.Text(form, width=42, height=12, font=('Consolas', 10))
        capability.grid(row=len(fields), column=1, sticky='w', pady=(8, 0))
        fields['capability'] = capability

        def show(row) -> None:
            flush()
            shown['row'] = row
            if row is None:
                return
            for key in ('rid', 'rtype', 'x', 'y', 'theta_deg', 'bat'):
                fields[key].set(str(row[key]))
            capability.delete('1.0', 'end')
            capability.insert('1.0', format_map(row['capability']))

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['robots']:
                return
            row['rid'] = fields['rid'].get().strip()
            row['rtype'] = fields['rtype'].get().strip()
            for key in ('x', 'y', 'theta_deg', 'bat'):
                row[key] = float(fields[key].get())
            row['capability'] = [(key, float(value)) for key, value in parse_map(capability.get('1.0', 'end'))]
            fill()

        def add() -> None:
            flush()
            model['robots'].append({
                'rid': f'R{len(model["robots"]) + 1}',
                'rtype': 'transport',
                'x': 0.0, 'y': 0.0, 'theta_deg': 0.0, 'bat': 1.0,
                'capability': [],
            })
            fill()
            listing.selection_set('end')
            show(model['robots'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['robots'][sel[0]]
            fill()

        fill()
        if model['robots']:
            listing.selection_set(0)
            show(model['robots'][0])
        return fill, flush

    def build_actions(parent):
        listing, fill, current = section_list(
            parent, lambda: model['actions'], '动作',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        name = tk.StringVar()
        kind = tk.StringVar()
        left = tk.StringVar()
        right = tk.StringVar()
        shown = {'row': None}
        ttk.Label(form, text='名称').grid(row=0, column=0, sticky='w')
        ttk.Entry(form, textvariable=name, width=24).grid(row=0, column=1, sticky='w')
        ttk.Label(form, text='种类').grid(row=1, column=0, sticky='w')
        ttk.Combobox(form, textvariable=kind, values=ACTION_KINDS, state='readonly', width=12).grid(row=1, column=1, sticky='w')
        ttk.Label(form, text='左参数').grid(row=2, column=0, sticky='w')
        ttk.Entry(form, textvariable=left, width=36).grid(row=2, column=1, sticky='w')
        ttk.Label(form, text='右参数').grid(row=3, column=0, sticky='w')
        ttk.Entry(form, textvariable=right, width=36).grid(row=3, column=1, sticky='w')
        ttk.Label(
            form,
            text='点写落点名或 (x, y)。a1/t2 只填左参数。空的右参数存成 None。',
        ).grid(row=4, column=0, columnspan=2, sticky='w', pady=8)

        def show(row) -> None:
            flush()
            shown['row'] = row
            if row is None:
                return
            name.set(row['name'])
            kind.set(row['kind'])
            left.set('' if row['left'] is None else py_obj(row['left']) if not isinstance(row['left'], str) else row['left'])
            right.set('' if row['right'] is None else py_obj(row['right']) if not isinstance(row['right'], str) else row['right'])

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['actions']:
                return
            row['name'] = name.get().strip()
            row['kind'] = kind.get()
            row['left'] = parse_literal(left.get())
            row['right'] = parse_literal(right.get())
            fill()

        def add() -> None:
            flush()
            model['actions'].append({'name': 'a_new', 'kind': 'a1', 'left': 0.01, 'right': None})
            fill()
            listing.selection_set('end')
            show(model['actions'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['actions'][sel[0]]
            fill()

        fill()
        if model['actions']:
            listing.selection_set(0)
            show(model['actions'][0])
        return fill, flush

    def build_facilities(parent):
        listing, fill, current = section_list(
            parent, lambda: model['facilities'], '设施',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        name = tk.StringVar()
        kind = tk.StringVar()
        reliability = tk.StringVar()
        sites = tk.StringVar()
        shown = {'row': None}
        ttk.Label(form, text='名字').grid(row=0, column=0, sticky='w')
        ttk.Entry(form, textvariable=name, width=24).grid(row=0, column=1, sticky='w')
        ttk.Label(form, text='种类').grid(row=1, column=0, sticky='w')
        ttk.Combobox(form, textvariable=kind, values=FACILITY_KINDS, state='readonly', width=12).grid(row=1, column=1, sticky='w')
        ttk.Label(form, text='可靠性').grid(row=2, column=0, sticky='w')
        ttk.Entry(form, textvariable=reliability, width=12).grid(row=2, column=1, sticky='w')
        ttk.Label(form, text='落点，逗号分隔').grid(row=3, column=0, sticky='w')
        ttk.Entry(form, textvariable=sites, width=36).grid(row=3, column=1, sticky='w')

        def show(row) -> None:
            flush()
            shown['row'] = row
            if row is None:
                return
            name.set(row['name'])
            kind.set(row['kind'])
            reliability.set(str(row['reliability']))
            sites.set(', '.join(row['sites']))

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['facilities']:
                return
            row['name'] = name.get().strip()
            row['kind'] = kind.get()
            row['reliability'] = float(reliability.get())
            row['sites'] = [part.strip() for part in sites.get().split(',') if part.strip()]
            fill()

        def add() -> None:
            flush()
            names = [item['name'] for item in model['sites'][:2]] or ['gate', 'stack']
            model['facilities'].append({
                'name': f'fac_{len(model["facilities"]) + 1}',
                'kind': 'camera',
                'reliability': 0.8,
                'sites': names,
            })
            fill()
            listing.selection_set('end')
            show(model['facilities'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['facilities'][sel[0]]
            fill()

        fill()
        if model['facilities']:
            listing.selection_set(0)
            show(model['facilities'][0])
        return fill, flush

    def build_comms(parent):
        listing, fill, current = section_list(
            parent, lambda: model['comms'], '通信',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        name = tk.StringVar()
        reliability = tk.StringVar()
        min_crew = tk.StringVar()
        shown = {'row': None}
        ttk.Label(form, text='名字').grid(row=0, column=0, sticky='w')
        ttk.Entry(form, textvariable=name, width=24).grid(row=0, column=1, sticky='w')
        ttk.Label(form, text='可靠性').grid(row=1, column=0, sticky='w')
        ttk.Entry(form, textvariable=reliability, width=12).grid(row=1, column=1, sticky='w')
        ttk.Label(form, text='至少几台').grid(row=2, column=0, sticky='w')
        ttk.Entry(form, textvariable=min_crew, width=12).grid(row=2, column=1, sticky='w')

        def show(row) -> None:
            flush()
            shown['row'] = row
            if row is None:
                return
            name.set(row['name'])
            reliability.set(str(row['reliability']))
            min_crew.set(str(row['min_crew']))

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['comms']:
                return
            row['name'] = name.get().strip()
            row['reliability'] = float(reliability.get())
            row['min_crew'] = int(min_crew.get())
            fill()

        def add() -> None:
            flush()
            model['comms'].append({'name': f'link_{len(model["comms"]) + 1}', 'reliability': 0.8, 'min_crew': 2})
            fill()
            listing.selection_set('end')
            show(model['comms'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['comms'][sel[0]]
            fill()

        fill()
        if model['comms']:
            listing.selection_set(0)
            show(model['comms'][0])
        return fill, flush

    def build_tasks(parent):
        listing, fill, current = section_list(
            parent, lambda: model['tasks'], '任务',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        fields = {key: tk.StringVar() for key in (
            'tid', 'ttype', 'reliability', 'site', 'bat_need', 'bat_cost', 'deps', 'clear',
        )}
        shown = {'row': None}
        captions = (
            ('tid', '编号'), ('ttype', '类型'), ('reliability', '可靠性'),
            ('site', '结束落点'), ('bat_need', '电量门槛'), ('bat_cost', '耗电'),
            ('deps', '依赖编号，逗号分隔'), ('clear', '清空的落点，可空'),
        )
        for index, (key, caption) in enumerate(captions):
            ttk.Label(form, text=caption).grid(row=index, column=0, sticky='w', pady=2)
            ttk.Entry(form, textvariable=fields[key], width=32).grid(row=index, column=1, sticky='w')
        ttk.Label(form, text='角色').grid(row=8, column=0, sticky='nw', pady=(8, 0))
        roles = tk.Listbox(form, height=8, width=42, exportselection=False, font=('Consolas', 10))
        roles.grid(row=8, column=1, sticky='w', pady=(8, 0))
        role_bar = ttk.Frame(form)
        role_bar.grid(row=9, column=1, sticky='w', pady=4)

        def role_text(item) -> str:
            slot = item.get('slot') or ''
            mark = slot if slot else '（无槽）'
            return f"{ROLE_NAME.get(item['rtype'], item['rtype'])} {mark}  {item['capability']}"

        def show_roles(row) -> None:
            roles.delete(0, 'end')
            for item in row['roles']:
                roles.insert('end', role_text(item))

        def show(row) -> None:
            flush()
            shown['row'] = row
            roles.delete(0, 'end')
            if row is None:
                return
            fields['tid'].set(row['tid'])
            fields['ttype'].set(row['ttype'])
            fields['reliability'].set(str(row['reliability']))
            fields['site'].set(row['site'])
            fields['bat_need'].set(str(row['bat_need']))
            fields['bat_cost'].set(str(row['bat_cost']))
            fields['deps'].set(', '.join(row['deps']))
            fields['clear'].set('' if not row['clear'] else str(row['clear']))
            show_roles(row)

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['tasks']:
                return
            row['tid'] = fields['tid'].get().strip()
            row['ttype'] = fields['ttype'].get().strip()
            row['reliability'] = float(fields['reliability'].get())
            row['site'] = fields['site'].get().strip()
            row['bat_need'] = float(fields['bat_need'].get())
            row['bat_cost'] = float(fields['bat_cost'].get())
            row['deps'] = [part.strip() for part in fields['deps'].get().split(',') if part.strip()]
            clear = fields['clear'].get().strip()
            row['clear'] = None if not clear or clear in ('None', 'none') else clear
            fill()

        def add() -> None:
            flush()
            site = model['sites'][0]['name'] if model['sites'] else 'stack'
            model['tasks'].append({
                'tid': f'T_new_{len(model["tasks"]) + 1}',
                'ttype': 'dig_x',
                'reliability': 1.0,
                'site': site,
                'bat_need': 0.2,
                'bat_cost': 0.02,
                'deps': [],
                'roles': [{'rtype': 'arm', 'slot': '', 'capability': 0.8}],
                'clear': None,
            })
            fill()
            listing.selection_set('end')
            show(model['tasks'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['tasks'][sel[0]]
            fill()

        def add_role() -> None:
            row = shown['row']
            if row is None:
                return
            got = ask('添加角色', [
                ('rtype', '机型', ROLES),
                ('capability', '能力阈值', None),
                ('slot', '槽，可空', None),
            ])
            if not got:
                return
            row['roles'].append({
                'rtype': got['rtype'],
                'slot': got['slot'],
                'capability': float(got['capability']),
            })
            show_roles(row)

        def delete_role() -> None:
            row = shown['row']
            sel = roles.curselection()
            if row is None or not sel:
                return
            del row['roles'][sel[0]]
            show_roles(row)

        ttk.Button(role_bar, text='加角色', command=add_role).pack(side='left')
        ttk.Button(role_bar, text='删角色', command=delete_role).pack(side='left', padx=4)

        fill()
        if model['tasks']:
            listing.selection_set(0)
            show(model['tasks'][0])
        return fill, flush

    def build_operad(parent):
        listing, fill, current = section_list(
            parent, lambda: model['generators'], '生成规则',
            lambda row: show(row), lambda: add(), lambda: delete(),
        )
        form = ttk.Frame(parent)
        form.pack(side='left', fill='both', expand=True)
        name = tk.StringVar()
        deps = tk.StringVar()
        parallel = tk.BooleanVar()
        steps = tk.Listbox(form, height=10, font=('Consolas', 10), exportselection=False)
        step_var = tk.StringVar()
        shown = {'row': None}
        ttk.Label(form, text='名称').grid(row=0, column=0, sticky='w')
        ttk.Entry(form, textvariable=name, width=24).grid(row=0, column=1, sticky='w')
        ttk.Label(form, text='步骤').grid(row=1, column=0, sticky='nw')
        steps.grid(row=1, column=1, sticky='w')
        step_bar = ttk.Frame(form)
        step_bar.grid(row=2, column=1, sticky='w', pady=4)
        ttk.Entry(step_bar, textvariable=step_var, width=16).pack(side='left')
        ttk.Button(step_bar, text='加入', command=lambda: add_step()).pack(side='left', padx=4)
        ttk.Button(step_bar, text='移除', command=lambda: remove_step()).pack(side='left')
        ttk.Checkbutton(form, text='步骤之间不连边（并行）', variable=parallel).grid(row=3, column=1, sticky='w')
        ttk.Label(form, text='去掉移动后的下标，如 0-1, 1-2').grid(row=4, column=0, sticky='w')
        ttk.Entry(form, textvariable=deps, width=28).grid(row=4, column=1, sticky='w')
        ttk.Label(
            form,
            text='留空且不勾选并行时，相邻步骤连成一条链。' + '、'.join(MOVE_NAMES) + ' 会在装配时去掉。基元来自任务类型。',
        ).grid(row=5, column=0, columnspan=2, sticky='w', pady=6)

        def show(row) -> None:
            flush()
            shown['row'] = row
            steps.delete(0, 'end')
            if row is None:
                return
            name.set(row['name'])
            parallel.set(row['deps'] == [])
            if row['deps']:
                deps.set(', '.join(f'{a}-{b}' for a, b in row['deps']))
            else:
                deps.set('')
            for step in row['steps']:
                steps.insert('end', step)

        def parse_deps(text: str):
            pairs = []
            for part in text.split(','):
                part = part.strip()
                if not part:
                    continue
                left, right = part.split('-')
                pairs.append((int(left), int(right)))
            return pairs

        def flush() -> None:
            row = shown['row']
            if row is None or row not in model['generators']:
                return
            row['name'] = name.get().strip()
            row['steps'] = list(steps.get(0, 'end'))
            if parallel.get():
                row['deps'] = []
            elif deps.get().strip():
                row['deps'] = parse_deps(deps.get())
            else:
                row['deps'] = None
            fill()

        def add_step() -> None:
            text = step_var.get().strip()
            if text:
                steps.insert('end', text)
                step_var.set('')

        def remove_step() -> None:
            sel = steps.curselection()
            if sel:
                steps.delete(sel[0])

        def add() -> None:
            flush()
            model['generators'].append({'name': f'rule_{len(model["generators"]) + 1}', 'steps': [], 'deps': None})
            fill()
            listing.selection_set('end')
            show(model['generators'][-1])

        def delete() -> None:
            sel = listing.curselection()
            if not sel:
                return
            shown['row'] = None
            del model['generators'][sel[0]]
            fill()

        fill()
        if model['generators']:
            listing.selection_set(0)
            show(model['generators'][0])
        return fill, flush

    add_editor('落点', build_sites)
    add_editor('机器人', build_robots)
    add_editor('动作', build_actions)
    add_editor('设施', build_facilities)
    add_editor('通信', build_comms)
    add_editor('任务', build_tasks)
    add_editor('生成图', build_operad)

    closed = {'done': False}

    def close() -> None:
        if closed['done']:
            return
        closed['done'] = True
        root.quit()
        try:
            root.destroy()
        except tk.TclError:
            pass

    root.protocol('WM_DELETE_WINDOW', close)
    root.bind('<Escape>', lambda _event: close())
    if os.environ.get('EDITTOOL_SMOKE'):
        root.after(400, close)
    root.mainloop()


if __name__ == '__main__':
    launch()
