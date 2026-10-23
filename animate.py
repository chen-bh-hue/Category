"""
把 hybridx_apply.py 的求解方案做成场地动画。

用法：
  python animate.py           保存 gif，并打开播放窗口
  python animate.py --no-gui  只保存 gif
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
from matplotlib.animation import PillowWriter
from matplotlib.figure import Figure

import hybridx_apply as app


OUT = Path(__file__).resolve().parent / 'figures' / 'plan.gif'
INK = '#1A1A1A'
MUTED = '#5C5C5C'
TR = '#1F4E89'
AR = '#A33B24'
OBJ_COLOR = {'A': '#C47B2B', 'B': '#2F6F4E', 'C': '#2C5F8A'}
KIND_NAME = {'dig': '挖掘', 'load': '装载', 'unload': '卸载', 'grasp': '抓取', 'place': '放置'}


def _kind(ttype: str) -> str:
    head = ttype.split('_', 1)[0]
    return head if head in KIND_NAME else ttype


def _letter(ttype: str) -> str:
    return ttype.rsplit('_', 1)[-1].upper()


def _action_name(ttype: str) -> str:
    name = KIND_NAME.get(_kind(ttype), ttype)
    if 'nav' in ttype and _kind(ttype) in ('load', 'unload'):
        return '避障' + name
    return name


def _crew_text(crew: dict[str, str]) -> str:
    names = {'transport': '运输车', 'arm': '机械臂'}
    parts = []
    for key, rid in crew.items():
        rtype, _, slot = key.partition(':')
        parts.append(f'{names.get(rtype, rtype)}{slot} {rid}')
    return '  '.join(parts)


def _lerp(start: tuple[float, float], end: tuple[float, float], t: float) -> tuple[float, float]:
    return (start[0] + (end[0] - start[0]) * t, start[1] + (end[1] - start[1]) * t)


def _dist(start: tuple[float, float], end: tuple[float, float]) -> float:
    return ((end[0] - start[0]) ** 2 + (end[1] - start[1]) ** 2) ** 0.5


def _group(crew: dict[str, str], rtype: str) -> list[str]:
    return [rid for key, rid in crew.items() if key.split(':', 1)[0] == rtype]


def _sites(task, crew: dict[str, str]) -> dict[str, tuple[float, float]]:
    found = {}
    for effect in task.post:
        if effect.target[0] == 'robot' and effect.key == 'pos' and effect.mode == 'set':
            found[crew[effect.target[1]]] = (float(effect.value[0]), float(effect.value[1]))
    return found


def _copy_state(pos, trail, visible, hold, ground, stacked) -> dict:
    return {
        'pos': {rid: tuple(point) for rid, point in pos.items()},
        'trail': {rid: [tuple(point) for point in points] for rid, points in trail.items()},
        'visible': dict(visible),
        'hold': {letter: (list(who) if isinstance(who, list) else who) for letter, who in hold.items()},
        'ground': {letter: None if point is None else tuple(point) for letter, point in ground.items()},
        'stacked': dict(stacked),
    }


def _apply_work(ttype: str, crew: dict[str, str], sites: dict[str, tuple[float, float]], visible, hold, ground, stacked) -> None:
    letter = _letter(ttype)
    kind = _kind(ttype)
    arms = _group(crew, 'arm')
    trucks = _group(crew, 'transport')
    spot = next(iter(sites.values()), None)
    if kind == 'dig':
        visible[letter] = True
        hold[letter] = arms[0] if arms else None
        ground[letter] = None
        stacked[letter] = False
    elif kind == 'load':
        visible[letter] = True
        hold[letter] = list(trucks)
        ground[letter] = None
        stacked[letter] = False
    elif kind == 'unload':
        visible[letter] = True
        hold[letter] = None
        ground[letter] = spot
        stacked[letter] = False
    elif kind == 'grasp':
        visible[letter] = True
        hold[letter] = arms[0] if arms else None
        ground[letter] = None
        stacked[letter] = False
    elif kind == 'place':
        visible[letter] = True
        hold[letter] = None
        ground[letter] = app.STACK
        stacked[letter] = True


def build_frames() -> list[dict]:
    plan = app.plan
    if not plan.feasible:
        raise ValueError('当前方案不可行，没有可播放的过程')
    meta = {rid: robot.rtype for rid, robot in app.robots.items()}
    pos = {rid: tuple(robot.state.pos) for rid, robot in app.robots.items()}
    trail = {rid: [pos[rid]] for rid in pos}
    visible = {'A': False, 'B': False, 'C': True}
    hold = {'A': None, 'B': None, 'C': None}
    ground = {'A': None, 'B': None, 'C': tuple(app.WAIT_C)}
    stacked = {'A': False, 'B': False, 'C': False}
    frames = []

    def push(title: str, index: int, phase: str, repeat: int = 1) -> None:
        shot = _copy_state(pos, trail, visible, hold, ground, stacked)
        shot['title'] = title
        shot['index'] = index
        shot['phase'] = phase
        shot['meta'] = meta
        frames.extend(shot for _ in range(repeat))

    push('起始布置', 0, 'work', 8)
    total = len(plan.order)
    for index, nid in enumerate(plan.order, start=1):
        task = app.solver.by_id[nid].task
        crew = plan.assign[nid]
        sites = _sites(task, crew)
        action = _action_name(task.ttype)
        letter = _letter(task.ttype)
        who = _crew_text(crew)
        start = {rid: pos[rid] for rid in sites}
        span = max((_dist(start[rid], sites[rid]) for rid in sites), default=0.0)
        steps = 1 if span < 0.4 else max(8, min(26, int(span / 3.2)))
        for step in range(1, steps + 1):
            t = step / steps
            for rid, dest in sites.items():
                pos[rid] = _lerp(start[rid], dest, t)
                trail[rid].append(pos[rid])
            push(f'{index}/{total}  前往{action} {letter}    {who}', index, 'move')
        _apply_work(task.ttype, crew, sites, visible, hold, ground, stacked)
        push(f'{index}/{total}  {action} {letter}    {who}', index, 'work', 8)
    push(f'完成  行程 {plan.travel:.2f} m', total, 'work', 14)
    return frames


def _object_point(frame: dict, letter: str):
    if frame['stacked'][letter]:
        return app.STACK
    who = frame['hold'][letter]
    if isinstance(who, list) and who:
        xs = [frame['pos'][rid][0] for rid in who]
        ys = [frame['pos'][rid][1] for rid in who]
        return (sum(xs) / len(xs), sum(ys) / len(ys))
    if isinstance(who, str):
        return frame['pos'][who]
    return frame['ground'][letter]


def _paint(ax, frame: dict, zoom: tuple[float, float, float, float] | None = None) -> None:
    ax.set_facecolor('white')
    ax.set_aspect('equal', adjustable='box')
    if zoom is None:
        ax.set_xlim(-6, 76)
        ax.set_ylim(-10, 70)
    else:
        ax.set_xlim(zoom[0], zoom[1])
        ax.set_ylim(zoom[2], zoom[3])
    ax.set_xlabel('$x$ (m)')
    ax.set_ylabel('$y$ (m)')
    ax.tick_params(direction='in', top=True, right=True)
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)
    for point, name in (
        (app.PIT_A, '采集 A'),
        (app.PIT_B, '采集 B'),
        (app.GATE, '入口'),
        (app.STACK, '建造点'),
    ):
        ax.scatter([point[0]], [point[1]], s=18, c='#E7D3A1', edgecolors='#8A6A3A', linewidths=0.6, zorder=1)
        if zoom is not None and not (zoom[0] <= point[0] <= zoom[1] and zoom[2] <= point[1] <= zoom[3]):
            continue
        ax.annotate(name, xy=point, xytext=(0, -9), textcoords='offset points', ha='center', va='top', fontsize=7, color=MUTED)
    for rid, points in frame['trail'].items():
        if len(points) < 2:
            continue
        color = TR if frame['meta'][rid] == 'transport' else AR
        ax.plot([p[0] for p in points], [p[1] for p in points], color=color, lw=0.6, alpha=0.35, zorder=2)
    for rid, point in frame['pos'].items():
        color = TR if frame['meta'][rid] == 'transport' else AR
        marker = 'o' if frame['meta'][rid] == 'transport' else '^'
        ax.scatter([point[0]], [point[1]], s=36, c=color, marker=marker, zorder=4)
        ax.annotate(rid, xy=point, xytext=(0, 7), textcoords='offset points', ha='center', va='bottom', fontsize=7, color=INK, zorder=4)
    stack_level = 0
    for letter in ('A', 'B', 'C'):
        if not frame['visible'][letter]:
            continue
        point = _object_point(frame, letter)
        if point is None:
            continue
        if frame['stacked'][letter]:
            offset = (0, 12 * stack_level)
            stack_level += 1
        elif frame['hold'][letter]:
            offset = (11, 2)
        else:
            offset = (0, 8)
        ax.annotate(
            letter, xy=point, xytext=offset, textcoords='offset points',
            ha='center', va='center', fontsize=8, color='white', fontweight='bold', zorder=5,
            bbox=dict(boxstyle='circle,pad=0.22', fc=OBJ_COLOR[letter], ec='none'),
        )
    if zoom is None:
        ax.text(
            0.01, 0.99, frame['title'], transform=ax.transAxes, va='top', ha='left', fontsize=8, color=INK,
            bbox=dict(boxstyle='round,pad=0.28', fc='white', ec='#D0D0D0', lw=0.4),
        )
        ax.set_title('任务过程', loc='left', fontweight='bold', color=INK, pad=6)
    else:
        ax.set_title('建造区', loc='left', fontsize=7, color=INK, pad=2)


def draw(ax, frame: dict, inset=None) -> None:
    ax.clear()
    _paint(ax, frame)
    if inset is None:
        return
    inset.clear()
    _paint(inset, frame, zoom=(-3, 20, -4, 14))
    inset.tick_params(labelsize=6)


def _style() -> None:
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'font.family': 'sans-serif',
        'font.sans-serif': ['Microsoft YaHei', 'SimHei', 'DejaVu Sans'],
        'axes.unicode_minus': False,
        'font.size': 8,
        'axes.labelsize': 8,
        'axes.titlesize': 9,
        'axes.linewidth': 0.6,
        'axes.edgecolor': INK,
        'axes.labelcolor': INK,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'xtick.color': INK,
        'ytick.color': INK,
    })


def save_gif(frames: list[dict], path: Path = OUT) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig = Figure(figsize=(7.4, 6.3), dpi=100)
    ax = fig.add_axes((0.08, 0.08, 0.88, 0.82))
    inset = fig.add_axes((0.63, 0.16, 0.28, 0.30))
    writer = PillowWriter(fps=16)
    with writer.saving(fig, str(path), dpi=100):
        for frame in frames:
            draw(ax, frame, inset)
            writer.grab_frame()
    return path


def launch(frames: list[dict]) -> None:
    import tkinter as tk
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

    root = tk.Tk()
    root.title('HybridX 任务动画')
    root.geometry('980x760')
    root.configure(bg='white')
    fig = Figure(figsize=(7.4, 6.3), dpi=100)
    ax = fig.add_axes((0.08, 0.08, 0.88, 0.82))
    inset = fig.add_axes((0.63, 0.16, 0.28, 0.30))
    canvas = FigureCanvasTkAgg(fig, master=root)
    canvas.get_tk_widget().pack(fill='both', expand=True)
    bar = tk.Frame(root, bg='white')
    bar.pack(fill='x', padx=8, pady=6)
    state = {'i': 0, 'play': True, 'after': None, 'closed': False}

    def show(index: int) -> None:
        state['i'] = index % len(frames)
        draw(ax, frames[state['i']], inset)
        canvas.draw_idle()

    def tick() -> None:
        if state['closed'] or not state['play']:
            return
        show(state['i'] + 1)
        state['after'] = root.after(60, tick)

    def toggle() -> None:
        state['play'] = not state['play']
        button.configure(text='暂停' if state['play'] else '播放')
        if state['play']:
            tick()

    def restart() -> None:
        show(0)
        state['play'] = True
        button.configure(text='暂停')
        if state['after'] is not None:
            root.after_cancel(state['after'])
        tick()

    button = tk.Button(bar, text='暂停', command=toggle, width=8)
    button.pack(side='left')
    tk.Button(bar, text='重头', command=restart, width=8).pack(side='left', padx=6)
    tk.Label(bar, text=str(OUT), bg='white', fg=MUTED).pack(side='left', padx=8)
    show(0)

    def close() -> None:
        if state['closed']:
            return
        state['closed'] = True
        state['play'] = False
        if state['after'] is not None:
            try:
                root.after_cancel(state['after'])
            except tk.TclError:
                pass
        try:
            canvas.get_tk_widget().destroy()
        except tk.TclError:
            pass
        try:
            root.quit()
        except tk.TclError:
            pass
        try:
            root.destroy()
        except tk.TclError:
            pass

    root.protocol('WM_DELETE_WINDOW', close)
    root.bind('<Escape>', lambda _event: close())
    if os.environ.get('ANIM_SMOKE'):
        root.after(400, close)
    else:
        state['after'] = root.after(60, tick)
    try:
        root.mainloop()
    finally:
        close()


def main() -> None:
    _style()
    frames = build_frames()
    path = save_gif(frames)
    print(f'{path}  {len(frames)} 帧')
    if '--no-gui' in sys.argv:
        return
    launch(frames)


if __name__ == '__main__':
    main()
