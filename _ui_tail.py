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

    def sheet(parent, columns, blank, rows, hint=''):
        table = Sheet(parent, columns, blank, hint)
        table.load(rows)
        refreshers.append(lambda table=table, key=rows: table.load(key) if False else None)
        # 重新加载时换成新的列表，所以刷新函数读 model 的当前列表。
        return table

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
                ROLES,
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

    ttk.Button(task_page, text='加一组角色', command=add_role_group).pack(anchor='w', pady=(4, 0))
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
        root.after(400, close)
    root.mainloop()


def book_page(title: str):
    raise RuntimeError('book_page 只在 launch 内使用')


if __name__ == '__main__':
    launch()
