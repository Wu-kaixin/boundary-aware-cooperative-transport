"""Visualize the frozen enclosure audit; truth polygons are evaluator overlays."""
from collections import deque

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Polygon as PolygonPatch
import numpy as np

from research_io import ROOT, guard, read, sha
from dbact.control.observed_reference import rotation
from dbact_sim.experiments.deployment import dump


def main():
    guard()
    public = ROOT/'docs/results/free_ray_enclosure_v1'
    protocol = ROOT/'experiments/ccta2027/free_ray_enclosure_v1.json'
    plan, analysis = read(protocol), read(public/'analysis.json')
    assert sha(protocol) == analysis['protocol_sha256']
    out = ROOT/plan['run_dir']
    if (public/'figure_receipt.json').exists():
        raise RuntimeError('Preserve the published figure')
    fig, axes = plt.subplots(2, 2, figsize=(10, 10), sharex=True, sharey=True, layout='constrained')
    titles = ['Near circle', 'Near L', 'Far C', 'Far skew box']
    graphs, inputs = {}, {str(protocol.relative_to(ROOT)): sha(protocol)}
    for ax, case, title in zip(axes.flat, plan['cases'], titles):
        cid = case['case_id']
        packet_path, enclosure_path = out/cid/'sanitized_packets.json', public/(cid+'_enclosures.json')
        assert sha(packet_path) == analysis['products'][cid+'/sanitized_packets.json']
        assert sha(enclosure_path) == analysis['products'][cid+'/enclosures.json']
        packets, records = read(packet_path)['packets'], read(enclosure_path)
        inputs[str(packet_path.relative_to(ROOT))] = sha(packet_path)
        inputs[str(enclosure_path.relative_to(ROOT))] = sha(enclosure_path)
        starts = np.concatenate([p['starts'] for p in packets])
        ends = np.concatenate([p['ends'] for p in packets])
        points = np.concatenate([np.asarray(p['target_points']).reshape(-1, 2) for p in packets])
        positions = np.asarray([p['starts'][0] for p in packets])
        ax.add_collection(LineCollection(np.stack([starts, ends], axis=1), colors='#56A4CE', linewidths=.35, alpha=.4))
        snapshot = plan['snapshots'][cid]
        old_folder = ROOT/plan['original_run_dir']/cid
        receipt = read(old_folder/'case_result.json')
        assert sha(old_folder/'case_result.json') == snapshot['receipt_sha256']
        assert sha(old_folder/'replay.npz') == receipt['products']['replay.npz']
        with np.load(old_folder/'replay.npz', allow_pickle=False) as z:
            k = snapshot['control_index']
            vertices = z['cargo/cargo_0/local_vertices']@rotation(z['cargo/cargo_0/angles'][k]).T+z['cargo/cargo_0/centers'][k]
        ax.add_patch(PolygonPatch(vertices, fc='#AAAAAA', ec='#444444', lw=1.2, zorder=3))
        if len(points):
            ax.scatter(*points.T, s=4, color='#D55E00', zorder=4)
        ax.scatter(*positions.T, marker='o', s=25, facecolor='white', edgecolor='#0072B2', zorder=5)
        total = next(r for r in records if r['arm'] == 'all_team_evaluator_only')
        if total['status'] == 'BOUNDED_BY_FREE_RAYS':
            ax.add_patch(PolygonPatch(total['vertices'], fc='none', ec='#009E73', lw=1.8, zorder=4))
        ax.set_title(f"{title} | t = {snapshot['epoch']:.2f} s\nAll-team area {100*total['area_ratio']:.2f}% | one-hop 100%", fontsize=11)
        ax.set(xlim=(0, 8), ylim=(0, 8), xticks=range(0, 9, 2), yticks=range(0, 9, 2))
        ax.set_aspect('equal')
        ax.grid(alpha=.14)
        # Descriptive connectivity of this already-frozen single snapshot.
        # No relaying is executed or counted as a new information arm.
        neighbors = {r['agent_id']: set(r['packet_agents'])-{r['agent_id']}
            for r in records if r['arm'] == 'one_hop'}
        unseen, components, diameters = set(neighbors), [], []
        while unseen:
            seed = min(unseen)
            component, queue = {seed}, deque([seed])
            while queue:
                for other in neighbors[queue.popleft()]-component:
                    component.add(other)
                    queue.append(other)
            unseen -= component
            components.append(sorted(component))
            longest = 0
            for node in component:
                distances, queue = {node: 0}, deque([node])
                while queue:
                    current = queue.popleft()
                    for other in neighbors[current]-distances.keys():
                        distances[other] = distances[current]+1
                        queue.append(other)
                longest = max(longest, max(distances.values()))
            diameters.append(longest)
        graphs[cid] = dict(components=components, component_diameter_hops=diameters)
    axes[1, 0].set_xlabel('x (m)'); axes[1, 1].set_xlabel('x (m)')
    axes[0, 0].set_ylabel('y (m)'); axes[1, 0].set_ylabel('y (m)')
    legend = [Line2D([], [], color='#56A4CE', label='Measured free segments'),
        Line2D([], [], color='#D55E00', marker='.', ls='', label='Noisy target returns'),
        Line2D([], [], color='#009E73', label='All-team center enclosure'),
        Line2D([], [], color='#777777', lw=5, label='True object (evaluation only)')]
    fig.legend(handles=legend, loc='outside lower center', ncol=2, fontsize=10)
    fig.suptitle('Single acquisition snapshots: local closure fails in all four conditions', fontsize=13)
    for suffix in ('png', 'pdf'):
        fig.savefig(public/f'free_ray_snapshots.{suffix}', dpi=190)
    plt.close(fig)
    dump(public/'figure_receipt.json', dict(script_sha256=sha(__file__), inputs=inputs,
        connectivity_scope='Descriptive static graph only; no relaying or new control episode',
        graphs=graphs, products={f'free_ray_snapshots.{s}': sha(public/f'free_ray_snapshots.{s}') for s in ('png', 'pdf')}))
    print(graphs)


if __name__ == '__main__':
    main()
