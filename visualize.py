"""
把 hybridx_apply.py 里的具体定义画成英文期刊风格的图，保存为 png，并打开查看窗口。

用法：
  python visualize.py           保存 png 并打开界面
  python visualize.py --no-gui  只保存 png
"""
from __future__ import annotations

import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

import matplotlib

matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.figure import Figure
from matplotlib.patches import FancyBboxPatch, Rectangle

import hybridx_apply as app
from hybridx import A1, A2, A3, ArmRobot, M1, M2, T1, T2, TransportRobot


OUT_DIR = Path(__file__).resolve().parent / 'figures'

INK = '#1A1A1A'
MUTED = '#4D4D4D'
RULE = '#C8C8C8'
TR = '#1F4E89'
AR = '#A33B24'

STYLE = {
    'a1': ('#F4F4F4', '#5E5E5E'),
    'a2': ('#E7F3EF', '#1B7F6E'),
    'a3': ('#F0ECF7', '#6A5AA8'),
    'm1': ('#F8EEEA', '#A33B24'),
    'm2': ('#F6E4DC', '#8C2F1B'),
    't1': ('#E8F0F8', '#1F4E89'),
    't2': ('#E3EAF6', '#1A3E75'),
    'gen': ('#F7F8F8', '#2C4A54'),
}

LANE = {'transport': 0.0, 'arm': -0.70}
NODE_H = 0.22
A1_H = 0.16
A1_DROP = 0.28
COL_GAP = 0.28
LINE = 0.142


def _font() -> None:
    plt.rcParams.update({
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'STIXGeneral', 'DejaVu Serif'],
        'mathtext.fontset': 'stix',
        'font.size': 8,
        'axes.labelsize': 8,
        'axes.titlesize': 8,
        'axes.linewidth': 0.6,
        'axes.edgecolor': INK,
        'axes.labelcolor': INK,
        'xtick.labelsize': 7,
        'ytick.labelsize': 7,
        'xtick.direction': 'in',
        'ytick.direction': 'in',
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.major.size': 2.6,
        'ytick.major.size': 2.6,
        'xtick.color': INK,
        'ytick.color': INK,
        'axes.unicode_minus': False,
        'pdf.fonttype': 42,
        'ps.fonttype': 42,
    })


def _figure(w: float, h: float):
    fig = Figure(figsize=(w, h), dpi=120)
    fig.patch.set_facecolor('white')
    return fig


def _save(fig, path: Path) -> None:
    fig.savefig(path, dpi=300, bbox_inches='tight', pad_inches=0.035, facecolor='white')


def _num(value: float) -> str:
    return f'{value:g}'


def _point(p) -> str:
    return '(' + ', '.join(_num(float(v)) for v in p) + ')'


def _width(text: str, size: float = 7.0) -> float:
    plain = text.replace('$', '').replace('_', '').replace('{', '').replace('}', '')
    return max(0.48, len(plain) * size / 72 * 0.50 + 0.16)


def _panel(ax, text: str, x: float, y: float) -> None:
    ax.text(x, y, text, ha='left', va='bottom', fontsize=8, fontweight='bold', color=INK, zorder=5)


def _node(ax, x: float, y: float, text: str, kind: str, w: float, h: float):
    face, edge = STYLE.get(kind, STYLE['gen'])
    ax.add_patch(FancyBboxPatch(
        (x - w / 2, y - h / 2), w, h,
        boxstyle='square,pad=0',
        facecolor=face, edgecolor=edge, linewidth=0.7, zorder=3, mutation_aspect=None,
    ))
    ax.text(x, y, text, ha='center', va='center', fontsize=7, color=INK, zorder=4)
    return {'l': x - w / 2, 'r': x + w / 2, 't': y + h / 2, 'b': y - h / 2, 'x': x, 'y': y}


def _edge(ax, src: dict, dst: dict, dashed: bool = False) -> None:
    color = '#6A6A6A' if dashed else INK
    ls = (0, (2.2, 1.4)) if dashed else 'solid'
    x1, y1, x2, y2 = src['r'], src['y'], dst['l'], dst['y']
    if abs(y1 - y2) < 1e-6:
        ax.annotate(
            '', xy=(x2, y2), xytext=(x1, y1),
            arrowprops=dict(arrowstyle='-|>', color=color, lw=0.55, mutation_scale=6,
                            linestyle=ls, shrinkA=0, shrinkB=0),
            zorder=2,
        )
        return
    elbow = min(x1 + 0.16, (x1 + x2) / 2)
    ax.plot([x1, elbow], [y1, y1], color=color, lw=0.55, ls=ls, solid_capstyle='butt', zorder=2)
    ax.plot([elbow, elbow], [y1, y2], color=color, lw=0.55, ls=ls, solid_capstyle='butt', zorder=2)
    ax.annotate(
        '', xy=(x2, y2), xytext=(elbow, y2),
        arrowprops=dict(arrowstyle='-|>', color=color, lw=0.55, mutation_scale=6,
                        linestyle=ls, shrinkA=0, shrinkB=0),
        zorder=2,
    )


def _fork(ax, src: dict, dst: dict) -> None:
    ax.annotate(
        '', xy=(dst['x'], dst['t']), xytext=(src['x'], src['b']),
        arrowprops=dict(arrowstyle='-|>', color='#6A6A6A', lw=0.5, mutation_scale=5,
                        linestyle=(0, (1.6, 1.2)), shrinkA=0, shrinkB=0),
        zorder=2,
    )


def _layers(ids: list[str], edges: list[tuple[str, str]]) -> dict[str, int]:
    indeg = {i: 0 for i in ids}
    succ: dict[str, list[str]] = {i: [] for i in ids}
    for src, dst in edges:
        if src in indeg and dst in indeg and dst not in succ[src]:
            succ[src].append(dst)
            indeg[dst] += 1
    layer = {i: 0 for i in ids}
    pending = dict(indeg)
    ready = [i for i in ids if pending[i] == 0]
    while ready:
        cur = ready.pop(0)
        for nxt in succ[cur]:
            layer[nxt] = max(layer[nxt], layer[cur] + 1)
            pending[nxt] -= 1
            if pending[nxt] == 0:
                ready.append(nxt)
    return layer


def _draw_chain(ax, labels: list[tuple[str, str]], origin: tuple[float, float]) -> dict:
    """在单车道上画一条链。labels 为 (文字, 颜色键)。返回包围盒。"""
    x0, y0 = origin
    boxes = []
    cursor = x0
    for text, kind in labels:
        w = _width(text)
        box = _node(ax, cursor + w / 2, y0, text, kind, w, NODE_H)
        boxes.append(box)
        cursor += w + COL_GAP
    for src, dst in zip(boxes, boxes[1:]):
        _edge(ax, src, dst)
    if not boxes:
        return {'l': x0, 'r': x0, 't': y0, 'b': y0}
    return {
        'l': min(b['l'] for b in boxes),
        'r': max(b['r'] for b in boxes),
        't': max(b['t'] for b in boxes),
        'b': min(b['b'] for b in boxes),
    }


def _role_tag(role: str) -> str:
    return 'Tr' if role == 'transport' else 'Ar'


def _exec_labels(nodes) -> dict[str, str]:
    count = Counter((node.role, node.op) for node in nodes if node.op != 'a1')
    seen: Counter = Counter()
    labels = {}
    for node in nodes:
        if node.op == 'a1':
            labels[node.nid] = 'a1'
            continue
        seen[(node.role, node.op)] += 1
        base = f'{_role_tag(node.role)} {node.op}'
        if count[(node.role, node.op)] > 1:
            labels[node.nid] = f'{base}$_{{{seen[(node.role, node.op)]}}}$'
        else:
            labels[node.nid] = base
    return labels


def _layout_exec(graph):
    by_id = {node.nid: node for node in graph.nodes}
    labels = _exec_labels(graph.nodes)
    forks = {node.nid for node in graph.nodes if node.op == 'a1'}
    main = [node.nid for node in graph.nodes if node.nid not in forks]
    main_edges = [(s, d) for s, d in graph.edges if d not in forks]
    fork_edges = [(s, d) for s, d in graph.edges if d in forks]
    layer = _layers(main, main_edges)
    columns: dict[int, list[str]] = defaultdict(list)
    for nid in main:
        columns[layer[nid]].append(nid)

    pos = {}
    widths = {}
    cursor = 0.0
    for lvl in sorted(columns):
        group = columns[lvl]
        col_w = max(_width(labels[nid]) for nid in group)
        for nid in group:
            pos[nid] = (cursor + col_w / 2, LANE[by_id[nid].role])
            widths[nid] = _width(labels[nid])
        cursor += col_w + COL_GAP
    for src, dst in fork_edges:
        if src in pos:
            pos[dst] = (pos[src][0], pos[src][1] - A1_DROP)
            widths[dst] = _width('a1')
    return by_id, labels, pos, widths, main_edges, fork_edges


def _paint_exec(ax, graph, dx: float = 0.0, dy: float = 0.0) -> dict:
    by_id, labels, pos, widths, main_edges, fork_edges = _layout_exec(graph)
    boxes = {}
    for nid, (x, y) in pos.items():
        kind = by_id[nid].op
        h = A1_H if kind == 'a1' else NODE_H
        boxes[nid] = _node(ax, x + dx, y + dy, labels[nid], kind, widths[nid], h)
    for src, dst in main_edges:
        _edge(ax, boxes[src], boxes[dst])
    for src, dst in fork_edges:
        _fork(ax, boxes[src], boxes[dst])
    return {
        'l': min(b['l'] for b in boxes.values()),
        'r': max(b['r'] for b in boxes.values()),
        't': max(b['t'] for b in boxes.values()),
        'b': min(b['b'] for b in boxes.values()),
    }


def _style_map(ax) -> None:
    ax.set_facecolor('white')
    ax.set_aspect('equal', adjustable='box')
    ax.tick_params(top=True, right=True, direction='in')
    ax.grid(False)
    for spine in ax.spines.values():
        spine.set_linewidth(0.6)


def _robot_note(robot) -> str:
    state = robot.state
    if isinstance(state, TransportRobot):
        return f'{robot.rid}\n$b$ = {state.bat:.2f}\npayload = {state.full:g}, $s$ = {state.dis:g}'
    if isinstance(state, ArmRobot):
        return f'{robot.rid}\n$b$ = {state.bat:.2f}\ngripper = {state.grip:g}'
    return robot.rid


def _mat_points():
    points = []
    for key, name in app.env.mat.items():
        parts = str(key).split(',')
        points.append((float(parts[0]), float(parts[1]), str(name)))
    return points


def _mark_materials(ax, points, size: float = 0.56) -> None:
    half = size / 2
    for x, y, name in points:
        ax.add_patch(Rectangle((x - half, y - half), size, size,
                               facecolor='#E7D3A1', edgecolor='#8A6A3A', linewidth=0.6, zorder=1))
        ax.text(x, y - half - 0.15, name, ha='center', va='top', fontsize=6.5, fontstyle='italic', color=MUTED)


def draw_scene():
    fig = _figure(7.16, 3.05)
    build, dig = fig.subplots(1, 2)
    _style_map(build)
    _style_map(dig)
    build.set_title('(a)  Build zone', loc='left', pad=3, fontweight='bold')
    dig.set_title('(b)  Collection zone', loc='left', pad=3, fontweight='bold')

    mats = _mat_points()
    _mark_materials(build, [item for item in mats if item[0] < 25], size=0.7)
    _mark_materials(dig, [item for item in mats if item[0] >= 25], size=1.6)

    for robot in app.robots.values():
        x, y = robot.state.pos
        ax = build if x < 25 else dig
        color = TR if robot.rtype == 'transport' else AR
        ax.scatter([x], [y], s=22, c=color, zorder=3, linewidths=0)
        reach = 0.7 if x < 25 else 1.4
        heading = robot.state.theta
        ax.annotate(
            '', xy=(x + reach * math.cos(heading), y + reach * math.sin(heading)), xytext=(x, y),
            arrowprops=dict(arrowstyle='-|>', color=color, lw=0.6, mutation_scale=6),
        )
        notes = {
            'R1': ((10, -34), 'left'),
            'R2': ((16, -30), 'left'),
            'R4': ((12, 6), 'left'),
            'R3': ((0, 8), 'center'),
        }
        xytext, align = notes.get(robot.rid, ((0, 8), 'center'))
        ax.annotate(
            _robot_note(robot), xy=(x, y), xytext=xytext, textcoords='offset points',
            fontsize=6.5, color=INK, ha=align, va='bottom',
        )
    build.set_xlim(-1.6, 18.0)
    build.set_ylim(-4.8, 10.5)
    dig.set_xlim(34.0, 72.0)
    dig.set_ylim(30.0, 66.0)
    for ax in (build, dig):
        ax.set_xlabel('$x$ (m)', labelpad=1)
        ax.set_ylabel('$y$ (m)', labelpad=1)
    fig.subplots_adjust(left=0.07, right=0.98, bottom=0.16, top=0.88, wspace=0.28)
    fig.text(0.5, 0.02, 'Build zone near the origin and collection zone near (50, 50).  $b$: battery,  $s$: mileage.',
             ha='center', va='bottom', fontsize=7, color=MUTED)
    return fig


def _morphism_rows():
    rows = {'shared': [], 'arm': [], 'transport': []}
    for name, action in app.actions.items():
        if isinstance(action, A1):
            rows['shared'].append((name, 'a1', rf'$\Delta b = -{_num(action.bat_decline_num)}$'))
        elif isinstance(action, A2):
            rows['shared'].append((name, 'a2', rf'${_point(action.src)} \rightarrow {_point(action.dst)}$'))
        elif isinstance(action, A3):
            body = ', '.join(f'{k}: {v}' for k, v in action.content.items()) or 'pending'
            rows['shared'].append((name, 'a3', f'belief $\\leftarrow$ {{{body}}}'))
        elif isinstance(action, M1):
            rows['arm'].append((name, 'm1', rf'${_point(action.src_angle)} \rightarrow {_point(action.dst_angle)}$'))
        elif isinstance(action, M2):
            rows['arm'].append((name, 'm2', action.act))
        elif isinstance(action, T1):
            rows['transport'].append((name, 't1', rf'$\Delta s = +{_num(action.dis_delta_num)}$'))
        elif isinstance(action, T2):
            sign = '+' if action.load_num >= 0 else ''
            rows['transport'].append((name, 't2', rf'$\Delta q = {sign}{_num(action.load_num)}$'))
    return rows


def draw_morphisms():
    groups = _morphism_rows()
    order = [('(a)  Shared', 'shared'), ('(b)  Manipulator', 'arm'), ('(c)  Transporter', 'transport')]
    n = max(len(v) for v in groups.values())
    fig = _figure(7.16, 0.42 + n * 0.30)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, 7.16)
    ax.set_ylim(0, 0.42 + n * 0.30)
    ax.axis('off')
    col_w = 7.16 / 3
    top = 0.42 + n * 0.30
    for i, (title, key) in enumerate(order):
        x = i * col_w
        ax.text(x + 0.08, top - 0.20, title, fontsize=8, fontweight='bold', color=INK, va='center')
        ax.plot([x + 0.08, x + col_w - 0.12], [top - 0.34, top - 0.34], color=INK, lw=0.5)
        for j, (name, symbol, spec) in enumerate(groups[key]):
            y = top - 0.56 - j * 0.30
            face, edge = STYLE[symbol]
            ax.add_patch(Rectangle((x + 0.08, y - 0.09), 0.34, 0.18, facecolor=face, edgecolor=edge, lw=0.6))
            ax.text(x + 0.25, y, symbol, ha='center', va='center', fontsize=6.5, color=INK)
            ax.text(x + 0.48, y, name, ha='left', va='center', fontsize=7, fontstyle='italic', color=INK)
            ax.text(x + 1.28, y, spec, ha='left', va='center', fontsize=7, color=INK)
    return fig


def _fmt_value(value) -> str:
    if isinstance(value, float):
        return _num(value)
    if isinstance(value, dict):
        parts = []
        for key, item in value.items():
            if isinstance(key, tuple):
                key = '(' + ','.join(str(part) for part in key) + ')'
            shown = str(item).lower() if isinstance(item, bool) else item
            parts.append(f'{key}: {shown}')
        return '{' + ', '.join(parts) + '}'
    if isinstance(value, tuple) and value and isinstance(value[0], (int, float)):
        return _point(value)
    return str(value)


def _who(name: str) -> str:
    base, _, slot = name.partition(':')
    label = {'transport': 'Tr', 'arm': 'Ar', 'env': 'env'}.get(base, base)
    return label if not slot else f'{label}{slot}'


def _pre_text(cond) -> str:
    if len(cond.who) == 1:
        return f'{_who(cond.who[0])}.{cond.key} {cond.op} {_num(cond.value)}'
    return f'{cond.key}({_who(cond.who[0])}, {_who(cond.who[1])}) {cond.op} {_num(cond.value)}'


def _post_text(effect) -> str:
    if effect.target[0] == 'env':
        owner = 'env'
    else:
        owner = _who(effect.target[1])
    value = _fmt_value(effect.value)
    if effect.mode == 'add':
        return f'{owner}.{effect.key} += {value}'
    if effect.mode == 'merge':
        return f'{owner}.{effect.key} $\\cup$ {value}'
    return f'{owner}.{effect.key} = {value}'


def _join(items: list[str], limit: int = 46) -> list[str]:
    lines: list[str] = []
    current = ''
    for item in items:
        trial = item if not current else f'{current};  {item}'
        if current and len(trial) > limit:
            lines.append(current)
            current = item
        else:
            current = trial
    if current:
        lines.append(current)
    return lines or ['none']


def _task_lines(task) -> list[tuple[str, str]]:
    roles = ', '.join(f'{_who(role.key())} {role.capability:.2f}' for role in task.roles)
    deps = []
    for dep in task.deps:
        deps.append(f'after {dep.ref}' if dep.by == 'id' else f'after any {dep.ref}')
    lines = [
        ('title', task.tid),
        ('type', task.ttype),
        ('body', f'Roles   {roles}'),
        ('body', f'R        {task.reliability:.2f}'),
    ]
    pre = _join([_pre_text(item) for item in task.pre])
    post = _join([_post_text(item) for item in task.post], limit=42)
    dep_lines = _join(deps, limit=40)
    lines.append(('body', f'Pre      {pre[0]}'))
    lines.extend(('body', f'         {item}') for item in pre[1:])
    lines.append(('body', f'Post     {post[0]}'))
    lines.extend(('body', f'         {item}') for item in post[1:])
    lines.append(('body', f'Deps     {dep_lines[0]}'))
    lines.extend(('body', f'         {item}') for item in dep_lines[1:])
    return lines


def draw_tasks():
    blocks = [_task_lines(task) for task in app.tasks.values()]
    cols = 3
    col_w = 2.32
    gap_x, gap_y = 0.10, 0.14
    rows = (len(blocks) + cols - 1) // cols
    heights = []
    for row in range(rows):
        chunk = blocks[row * cols:(row + 1) * cols]
        heights.append(max(len(block) for block in chunk) * LINE + 0.08)
    width = cols * col_w + (cols - 1) * gap_x
    height = sum(heights) + (rows - 1) * gap_y
    fig = _figure(width, height)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.axis('off')
    y_top = height
    for row in range(rows):
        chunk = blocks[row * cols:(row + 1) * cols]
        for col, lines in enumerate(chunk):
            x = col * (col_w + gap_x)
            ax.add_patch(Rectangle((x, y_top - heights[row]), col_w, heights[row], facecolor='white', edgecolor=RULE, lw=0.45, zorder=1))
            ax.add_patch(Rectangle((x, y_top - heights[row]), 0.028, heights[row], facecolor='#2C4A54', edgecolor='none', zorder=2))
            for i, (kind, text) in enumerate(lines):
                y = y_top - 0.03 - (i + 0.5) * LINE
                if kind == 'title':
                    ax.text(x + 0.07, y, text, fontsize=7.5, fontweight='bold', color=INK, va='center')
                elif kind == 'type':
                    ax.text(x + 0.07, y, text, fontsize=7, fontstyle='italic', color=MUTED, va='center')
                else:
                    label, _, body = text.partition('  ')
                    ax.text(x + 0.07, y, label.strip(), fontsize=6.3, color=MUTED, va='center')
                    ax.text(x + 0.40, y, body.strip(), fontsize=6.3, color=INK, va='center')
        y_top -= heights[row] + gap_y
    return fig


def _tag(index: int) -> str:
    return chr(ord('a') + index)


def _components(graph) -> list[list[tuple[str, str]]]:
    """每个连通分量按依赖排成一条从左到右的链。"""
    ids = [node.nid for node in graph.nodes]
    by_id = {node.nid: node for node in graph.nodes}
    link = {nid: [] for nid in ids}
    succ = {nid: [] for nid in ids}
    for src, dst in graph.edges:
        link[src].append(dst)
        link[dst].append(src)
        succ[src].append(dst)
    seen: set[str] = set()
    chains = []
    for nid in ids:
        if nid in seen:
            continue
        stack = [nid]
        comp: list[str] = []
        while stack:
            cur = stack.pop()
            if cur in seen:
                continue
            seen.add(cur)
            comp.append(cur)
            stack.extend(link[cur])
        indeg = {name: 0 for name in comp}
        inside = set(comp)
        for name in comp:
            for nxt in succ[name]:
                if nxt in inside:
                    indeg[nxt] += 1
        ready = [name for name in comp if indeg[name] == 0]
        order: list[str] = []
        while ready:
            ready.sort(key=ids.index)
            cur = ready.pop(0)
            order.append(cur)
            for nxt in succ[cur]:
                if nxt not in indeg:
                    continue
                indeg[nxt] -= 1
                if indeg[nxt] == 0:
                    ready.append(nxt)
        chains.append([(by_id[name].ttype, 'gen') for name in order])
    return chains


def _chain_size(labels: list[tuple[str, str]]) -> tuple[float, float]:
    width = sum(_width(text) for text, _ in labels) + COL_GAP * max(0, len(labels) - 1)
    return width, NODE_H


def draw_operad():
    blocks = []
    for name, graph in app.trees.items():
        chains = _components(graph)
        width = max(_chain_size(chain)[0] for chain in chains)
        height = 0.24 + len(chains) * NODE_H + max(0, len(chains) - 1) * 0.16
        blocks.append((name, chains, width, height))

    margin = 7.3
    gap_x, gap_y = 0.32, 0.22
    placed = []
    x = y = row_h = 0.0
    max_x = 0.0
    for name, chains, width, height in blocks:
        if x > 0 and x + width > margin:
            x = 0.0
            y -= row_h + gap_y
            row_h = 0.0
        placed.append((name, chains, x, y, width, height))
        x += width + gap_x
        row_h = max(row_h, height)
        max_x = max(max_x, x - gap_x)
    height = -(y - row_h)
    fig = _figure(max(max_x, 1.0), height)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, max(max_x, 1.0))
    ax.set_ylim(0, height)
    ax.axis('off')
    for index, (name, chains, left, top, _width_block, _height_block) in enumerate(placed):
        base = top + height
        _panel(ax, f'({_tag(index)})  {name}', left, base - 0.16)
        cursor = base - 0.24 - NODE_H / 2
        for chain in chains:
            _draw_chain(ax, chain, (left, cursor))
            cursor -= NODE_H + 0.16
    return fig


def _env_note(translation) -> str:
    if not translation.env:
        return ''
    return 'R $\\rightarrow$ E:  ' + ';  '.join(_post_text(effect) for effect in translation.env)


def draw_translation():
    blocks = []
    for name, item in app.translated.items():
        probe = _figure(1, 1)
        pax = probe.add_axes((0, 0, 1, 1))
        box = _paint_exec(pax, item.graph)
        plt.close(probe)
        note = _env_note(item)
        blocks.append((name, item, note, box))

    legend = [('a1', 'battery'), ('a2', 'motion'), ('a3', 'belief'), ('m1', 'joint'), ('m2', 'gripper'), ('t1', 'road'), ('t2', 'payload')]
    legend_w = sum(0.18 + _width(f'{kind}  {word}', 6.5) + 0.10 for kind, word in legend)
    width = max([block[3]['r'] - block[3]['l'] for block in blocks] + [legend_w]) + 0.02
    slots = []
    cursor = 0.0
    for i, (name, item, note, box) in enumerate(blocks):
        cursor -= 0.16
        title_y = cursor
        graph_top = cursor - 0.04
        dy = graph_top - box['t']
        cursor = box['b'] + dy
        note_y = None
        if note:
            note_y = cursor - 0.08
            cursor = note_y - 0.12
        slots.append((name, item, note, box, title_y, dy, note_y))
        if i != len(blocks) - 1:
            cursor -= 0.16
    legend_y = cursor - 0.16
    bottom = legend_y - 0.08
    shift = -bottom
    height = shift + 0.02
    fig = _figure(width, height)
    ax = fig.add_axes((0, 0, 1, 1))
    ax.set_xlim(0, width)
    ax.set_ylim(0, height)
    ax.axis('off')

    for i, (name, item, note, box, title_y, dy, note_y) in enumerate(slots):
        _panel(ax, f'({_tag(i)})  {name}', 0.0, title_y + shift)
        _paint_exec(ax, item.graph, -box['l'], dy + shift)
        if note_y is not None:
            ax.text(0.0, note_y + shift, note, fontsize=6.5, color=MUTED, va='top')

    x = 0.0
    for kind, word in legend:
        face, edge = STYLE[kind]
        ax.add_patch(Rectangle((x, legend_y + shift - 0.05), 0.14, 0.10, facecolor=face, edgecolor=edge, lw=0.45, zorder=3))
        ax.text(x + 0.17, legend_y + shift, f'{kind}  {word}', fontsize=6.5, va='center', color=INK)
        x += 0.17 + _width(f'{kind}  {word}', 6.5) + 0.08
    return fig


def build_figures() -> list[tuple[str, object]]:
    _font()
    return [
        ('Fig. 1  Scene', draw_scene()),
        ('Fig. 2  Morphisms', draw_morphisms()),
        ('Fig. 3  Primitive tasks', draw_tasks()),
        ('Fig. 4  Operad', draw_operad()),
        ('Fig. 5  Translation', draw_translation()),
    ]


def save_figures(pages: list[tuple[str, object]]) -> list[Path]:
    OUT_DIR.mkdir(exist_ok=True)
    for stale in OUT_DIR.glob('*.png'):
        stale.unlink()
    names = [
        'fig1_scene.png',
        'fig2_morphisms.png',
        'fig3_tasks.png',
        'fig4_operad.png',
        'fig5_translation.png',
    ]
    paths = []
    for name, (_, fig) in zip(names, pages):
        path = OUT_DIR / name
        _save(fig, path)
        paths.append(path)
    return paths


def launch(pages: list[tuple[str, object]]) -> None:
    import tkinter as tk
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

    root = tk.Tk()
    root.title('HybridX figures')
    root.geometry('1180x780')
    root.configure(bg='white')

    side = tk.Frame(root, bg='#F2F2F2', width=210)
    side.pack(side='left', fill='y')
    tk.Label(side, text='Figures', bg='#F2F2F2', fg=INK, font=('Times New Roman', 12, 'bold')).pack(anchor='w', padx=12, pady=(12, 4))
    listing = tk.Listbox(side, font=('Times New Roman', 11), activestyle='none', bg='white', fg=INK, bd=0, highlightthickness=0)
    listing.pack(fill='both', expand=True, padx=10, pady=6)
    for title, _ in pages:
        listing.insert('end', title)
    tk.Label(side, text=str(OUT_DIR), bg='#F2F2F2', fg=MUTED, font=('Times New Roman', 8), justify='left', wraplength=180).pack(anchor='w', padx=12, pady=10)

    main = tk.Frame(root, bg='white')
    main.pack(side='right', fill='both', expand=True)
    holder = {'canvas': None, 'toolbar': None}

    def show(_event=None):
        sel = listing.curselection()
        if not sel:
            return
        fig = pages[sel[0]][1]
        if holder['canvas'] is not None:
            holder['canvas'].get_tk_widget().destroy()
            holder['toolbar'].destroy()
        canvas = FigureCanvasTkAgg(fig, master=main)
        canvas.draw()
        toolbar = NavigationToolbar2Tk(canvas, main)
        toolbar.update()
        canvas.get_tk_widget().pack(fill='both', expand=True)
        holder['canvas'] = canvas
        holder['toolbar'] = toolbar

    listing.bind('<<ListboxSelect>>', show)
    listing.selection_set(0)
    show()

    def close():
        canvas = holder.get('canvas')
        toolbar = holder.get('toolbar')
        holder['canvas'] = None
        holder['toolbar'] = None
        for widget in (toolbar, canvas.get_tk_widget() if canvas is not None else None):
            if widget is None:
                continue
            try:
                widget.destroy()
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
    try:
        root.mainloop()
    finally:
        close()


def main() -> None:
    pages = build_figures()
    paths = save_figures(pages)
    for path in paths:
        print(path)
    if '--no-gui' in sys.argv:
        return
    launch(pages)


if __name__ == '__main__':
    main()
