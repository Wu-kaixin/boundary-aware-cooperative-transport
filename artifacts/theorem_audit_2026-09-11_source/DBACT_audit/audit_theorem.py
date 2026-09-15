"""Read-only audit of the submitted DBACT proof against the frozen repository.

The observer evaluates the PDF reference density, not each robot's local map.
Polygon edge Gaussian integrals are analytic; cell integrals use independent
polar quadrature. Numerical observations are not formal error certificates.
No controller, parameter, or repository file is modified.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from numpy.polynomial.legendre import leggauss
from scipy.integrate import quad, cumulative_trapezoid
from scipy.special import erf
import yaml


def dump(path, value):
    Path(path).write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")


class ReferenceObserver:
    """Exact edge density and star-shaped clipped Voronoi disk integration."""

    def __init__(self, domain, radius, sigma, floor, cage, lead, threshold, direction,
                 ntheta=256, nradial=16):
        self.domain = np.asarray(domain, dtype=float)
        self.R, self.sigma, self.floor = radius, sigma, floor
        self.cage, self.lead, self.threshold = cage, lead, threshold
        self.direction = np.asarray(direction, dtype=float)
        theta = 2 * np.pi * (np.arange(ntheta) + .5) / ntheta
        self.unit = np.column_stack((np.cos(theta), np.sin(theta)))
        self.dtheta = 2 * np.pi / ntheta
        z, w = leggauss(nradial)
        self.radial_nodes, self.radial_weights = (z + 1) / 2, w / 2

    def edges(self, vertices):
        edges = np.roll(vertices, -1, axis=0) - vertices
        lengths = np.linalg.norm(edges, axis=1)
        tangent = edges / lengths[:, None]
        normal = np.column_stack((tangent[:, 1], -tangent[:, 0]))
        offset = self.cage + (self.lead - self.cage) * np.clip(
            normal @ self.direction / self.threshold, 0, 1)
        start = vertices + offset[:, None] * normal
        return start, tangent, lengths

    def density(self, queries, vertices):
        queries = np.asarray(queries)
        start, tangent, lengths = self.edges(vertices)
        result = np.full(len(queries), self.floor)
        scale = math.sqrt(2) * self.sigma
        for p, u, length in zip(start, tangent, lengths):
            delta = queries - p
            along = delta @ u
            perpendicular2 = np.maximum(np.sum(delta * delta, axis=1) - along * along, 0)
            result += self.sigma * math.sqrt(np.pi / 2) * np.exp(
                -perpendicular2 / (2 * self.sigma ** 2)) * (
                    erf((length - along) / scale) + erf(along / scale))
        return result

    def total_mass(self, vertices):
        xmin, xmax, ymin, ymax = self.domain
        answer = self.floor * (xmax - xmin) * (ymax - ymin)
        scale = math.sqrt(2) * self.sigma
        start, tangent, lengths = self.edges(vertices)
        for p, u, length in zip(start, tangent, lengths):
            def integrand(s):
                x, y = p + s * u
                return np.pi * self.sigma**2 / 2 * (
                    erf((xmax-x)/scale) - erf((xmin-x)/scale)) * (
                    erf((ymax-y)/scale) - erf((ymin-y)/scale))
            answer += quad(integrand, 0, length, epsabs=1e-11, epsrel=1e-11)[0]
        return float(answer)

    def evaluate(self, positions, vertices):
        p = np.asarray(positions)
        n, nt = len(p), len(self.unit)
        reach = np.full((n, nt), self.R)
        xmin, xmax, ymin, ymax = self.domain
        # Each ray starts at its site, which lies in the rectangle.
        if np.any(p[:, 0] < xmin-1e-10) or np.any(p[:, 0] > xmax+1e-10) or \
                np.any(p[:, 1] < ymin-1e-10) or np.any(p[:, 1] > ymax+1e-10):
            raise ValueError("Observer requires sites inside D")
        for axis, low, high in [(0, xmin, xmax), (1, ymin, ymax)]:
            v = self.unit[:, axis]
            wall = np.where(v[None, :] > 0,
                            (high-p[:, axis, None]) / np.maximum(v[None, :], 1e-30),
                            (p[:, axis, None]-low) / np.maximum(-v[None, :], 1e-30))
            reach = np.minimum(reach, wall)
        for i in range(n):
            difference = p - p[i]
            squared = np.sum(difference * difference, axis=1)
            dot = difference @ self.unit.T
            denominator = 2 * np.maximum(dot, 1e-30)
            cap = squared[:, None] / denominator
            cap[dot <= 1e-14] = np.inf
            cap[i] = np.inf
            reach[i] = np.minimum(reach[i], np.min(cap, axis=0))
        reach = np.maximum(reach, 0)
        r = reach[:, :, None] * self.radial_nodes
        q = p[:, None, None, :] + r[:, :, :, None] * self.unit[None, :, None, :]
        phi = self.density(q.reshape(-1, 2), vertices).reshape(r.shape)
        w = self.dtheta * reach[:, :, None] * self.radial_weights * r
        weighted = phi * w
        mass = np.sum(weighted, axis=(1, 2))
        shift = np.sum(weighted[:, :, :, None] * r[:, :, :, None] *
                       self.unit[None, :, None, :], axis=(1, 2))
        centroid = p + shift / mass[:, None]
        gradient = -2 * shift
        H = self.R**2 * self.total_mass(vertices) + np.sum((r*r-self.R**2) * weighted)
        return dict(mass=mass, centroid=centroid, gradient=gradient,
                    gradient2=float(np.sum(gradient*gradient)),
                    centroid2=float(np.sum((centroid-p)**2)), H=float(H))


def constants(config, repo):
    from dbact_sim.scenarios import build_cargoes, controller_params_from_config
    from dbact.geometry import polygon_perimeter
    p = controller_params_from_config(config)
    cargo = build_cargoes(config, seed=0)[0]
    L = polygon_perimeter(cargo.vertices)
    N, R, kc, u = config['agents']['count'], p.local_radius, p.kp_cage, p.max_speed
    width = config['domain']['xmax'] - config['domain']['xmin']
    height = config['domain']['ymax'] - config['domain']['ymin']
    r0 = min(R, p.d_min/2, width/2, height/2)
    mlo = p.base_density*np.pi*r0*r0/4
    mhi = np.pi*R*R*(p.base_density+L)
    a = kc/(2*mhi)
    d_coarse_terms = math.sqrt(N)*(kc*2*R + max(kc*R-u, 0) + 2*u)
    d_alternative = math.sqrt(N)*(u+kc*R)
    d = min(d_coarse_terms, d_alternative)
    CK, LK = 2*np.pi*p.sigma**2, math.sqrt(2)*np.pi**1.5*p.sigma
    M = p.base_density*width*height + CK*L
    h = 2*R/(p.grid_resolution-1)
    rho = math.sqrt(2)*h/2
    return dict(N=N, R=R, kc=kc, umax=u, ds=p.d_min, sigma=p.sigma,
                phi0=p.base_density, dt=config['dt'], grid=p.grid_resolution,
                Lmax=L, CK=CK, LK=LK, m_minus=mlo, m_plus=mhi,
                a_minus=a, alpha=a/2, d_terms_b0=d_coarse_terms,
                d_speed_alternative=d_alternative, dbar=d,
                beta_static=d*d/(2*a), B_gradient_static=d*d/(a*a),
                B_gradient_geometry_cell=4*N*mhi*mhi*R*R,
                M_total_upper=M, B_gradient_geometry_total=4*R*R*M*M,
                B_centroid_static=d*d/(a*a*4*mlo*mlo), B_centroid_geometry=N*R*R,
                ratio_to_cell_geometry=(1+u/(kc*R))**2,
                ratio_to_total_geometry=d*d/(a*a)/(4*R*R*M*M),
                eta_m_floor=p.base_density*(4*np.pi*R*rho+np.pi*rho*rho),
                gamma_dt=p.gamma_agent*config['dt'],
                safety_neighbor_radius=2*u/p.gamma_agent +
                math.sqrt((2*u/p.gamma_agent)**2+p.d_min**2),
                motion_bound_status='No certified true object translation/rotation input bounds identified; static value is an optimistic baseline, not a valid moving-case guarantee.')


def counterexamples(config):
    from dbact.local_cvt import LocalCVT
    from dbact.boundary_density import BoundaryAwareDensity
    from dbact.types import AgentState, ControlCommand
    from dbact.controller import DBACTController
    from dbact_sim.scenarios import controller_params_from_config
    cvt = LocalCVT(local_radius=.8, grid_resolution=20, comm_range=1.6)
    density = BoundaryAwareDensity.from_targets([[4, 4]], .2, [1], .001)
    x = np.linspace(-.8, .8, 20)[13]
    grid = []
    for eps in [1e-4, 1e-6, 1e-8]:
        cs, counts = [], []
        for sign in [-1, 1]:
            agents = [AgentState('a', [4, 4]), AgentState('b', [4+2*x+sign*eps, 4])]
            r = cvt.compute(0, agents, [1], density, (0, 8, 0, 8))
            cs.append(r.centroid)
            counts.append(r.owned_samples)
        jump = float(np.linalg.norm(cs[1]-cs[0]))
        grid.append(dict(epsilon=eps, owned_samples=counts, centroid_jump=jump,
                         quotient=jump/(2*eps)))
    params = controller_params_from_config(config)
    controller = DBACTController(params, (0, 8, 0, 8), seed=0)
    a = params.d_min/math.sqrt(2)
    agents = [AgentState('a', [0, a]), AgentState('b', [a, 0])]
    velocities = [np.array([-1, -1])*params.max_speed/math.sqrt(2), np.zeros(2)]
    before = np.vstack([x.position for x in agents])
    free = before + config['dt']*np.vstack(velocities)
    controller.apply_commands(agents, [ControlCommand(x.agent_id, v) for x, v in zip(agents, velocities)], config['dt'])
    after = np.vstack([x.position for x in agents])
    return dict(grid_switch=grid, clipping=dict(ds=params.d_min,
                 initial_distance=float(np.linalg.norm(before[0]-before[1])),
                 unclipped_distance=float(np.linalg.norm(free[0]-free[1])),
                 clipped_distance=float(np.linalg.norm(after[0]-after[1]))))


def observer_check(config):
    from dbact_sim.scenarios import build_cargoes
    vertices = build_cargoes(config, seed=0)[0].vertices
    ob = ReferenceObserver((0, 8, 0, 8), .8, .2, .001, .105, .26, .25, [1, 0], 256, 16)
    queries = np.array([[4, 4], [3.8, 4.3], [5, 3.2]])
    exact = ob.density(queries, vertices)
    start, tangent, length = ob.edges(vertices)
    numeric = np.full(len(queries), ob.floor)
    for i, q in enumerate(queries):
        for p, u, L in zip(start, tangent, length):
            numeric[i] += quad(lambda s: np.exp(-np.sum((q-p-s*u)**2)/(2*ob.sigma**2)), 0, L, epsabs=1e-12)[0]
    max_error = float(np.max(np.abs(exact-numeric)))
    if max_error > 1e-9:
        raise AssertionError('Analytic reference-density cross-check failed')
    return dict(analytic_density_vs_adaptive_arclength_max_abs=max_error)


def run_case(config, repo, out, seed, frames, stride):
    from dbact_sim.environment import SimulationEnvironment
    from dbact_sim.scenarios import controller_params_from_config
    started = time.perf_counter()
    env = SimulationEnvironment(config, seed=seed)
    env.controller.trace_enabled = True
    p = controller_params_from_config(config)
    goal = env.goal_directions[env.cargoes[0].object_id]
    args = (env.domain, p.local_radius, p.sigma, p.base_density,
            p.cage_offset, p.lead_offset, p.lead_threshold, goal)
    observer = ReferenceObserver(*args, ntheta=256, nradial=16)
    refined = ReferenceObserver(*args, ntheta=1024, nradial=32)
    samples, safety, refinements = [], [], []
    all_positions, all_vertices, all_velocities = [], [], []
    original_apply = env.controller.apply_commands

    def apply_with_record(agents, commands, dt):
        positions = np.vstack([a.position.copy() for a in agents])
        velocities = np.vstack([c.velocity for c in commands])
        i, j = np.triu_indices(len(agents), 1)
        delta = positions[i]-positions[j]
        dv = velocities[i]-velocities[j]
        speed2 = np.sum(dv*dv, axis=1)
        tau = np.clip(-np.sum(delta*dv, axis=1)/np.maximum(speed2, 1e-30), 0, dt)
        hold_min = float(np.min(np.linalg.norm(delta+tau[:, None]*dv, axis=1)))
        h = np.sum(delta*delta, axis=1)-p.d_min**2
        half1 = 2*np.sum(delta*velocities[i], axis=1)+p.gamma_agent*h/2
        half2 = -2*np.sum(delta*velocities[j], axis=1)+p.gamma_agent*h/2
        if len(safety) % stride == 0:
            r = observer.evaluate(positions, env.cargoes[0].vertices)
            d = velocities - p.kp_cage*(r['centroid']-positions)
            hats = np.array([x.cvt_centroid for x in env.controller.diagnostics])
            valid = np.all(np.isfinite(hats), axis=1)
            observed_centroid2 = float(np.sum((hats[valid]-positions[valid])**2))
            samples.append(dict(time=env.t, H=r['H'], gradient2=r['gradient2'],
                         centroid2=r['centroid2'], disturbance2=float(np.sum(d*d)),
                         min_reference_mass=float(np.min(r['mass'])),
                         max_reference_mass=float(np.max(r['mass'])),
                         local_cvt_centroid2_partial=observed_centroid2,
                         local_cvt_agents=int(np.sum(valid))))
            if len(safety) in sorted(set([0, (frames//4//stride)*stride,
                                             (frames//2//stride)*stride,
                                             (3*frames//4//stride)*stride])):
                rr = refined.evaluate(positions, env.cargoes[0].vertices)
                refinements.append(dict(frame=len(safety),
                    gradient2_coarse=r['gradient2'], gradient2_fine=rr['gradient2'],
                    centroid2_coarse=r['centroid2'], centroid2_fine=rr['centroid2'],
                    centroid_max_difference=float(np.max(np.linalg.norm(r['centroid']-rr['centroid'], axis=1)))))
        all_positions.append(positions)
        all_vertices.append(env.cargoes[0].vertices.copy())
        all_velocities.append(velocities)
        original_apply(agents, commands, dt)
        end = np.vstack([a.position for a in agents])
        clip = np.linalg.norm(end-(positions+dt*velocities), axis=1)
        safety.append(dict(time=env.t, min_distance_during_held_step=hold_min,
                           min_pair_halfrow=float(min(np.min(half1), np.min(half2))),
                           clipping_agents=int(np.sum(clip > 1e-12)),
                           max_position_clip=float(np.max(clip)),
                           nominal_saturated_agents=sum(d.speed_nominal > p.max_speed+1e-12 for d in env.controller.diagnostics),
                           max_filter_correction=max(d.modification for d in env.controller.diagnostics),
                           mode_counts=env.controller.mode_counts()))

    env.controller.apply_commands = apply_with_record
    env.run(frames)
    positions = np.vstack([a.position for a in env.agents])
    r = observer.evaluate(positions, env.cargoes[0].vertices)
    samples.append(dict(time=env.t, H=r['H'], gradient2=r['gradient2'],
                        centroid2=r['centroid2'], disturbance2=float('nan'),
                        min_reference_mass=float(np.min(r['mass'])),
                        max_reference_mass=float(np.max(r['mass'])),
                        local_cvt_centroid2_partial=float('nan'), local_cvt_agents=0))
    t = np.array([s['time'] for s in samples])
    for metric in ['gradient2', 'centroid2']:
        y = np.array([s[metric] for s in samples])
        average = np.divide(cumulative_trapezoid(y, t, initial=0), t,
                            out=y.copy(), where=t>0)
        for row, avg in zip(samples, average):
            row[metric+'_time_average'] = float(avg)
    case_out = out / ('seed_'+str(seed))
    case_out.mkdir(parents=True, exist_ok=True)
    with (case_out/'residuals.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(samples[0])); writer.writeheader(); writer.writerows(samples)
    dump(case_out/'safety_diagnostics.json', safety)
    dump(case_out/'observer_refinement.json', refinements)
    summary = env.summary()
    dump(case_out/'simulation_summary.json', summary)
    np.savez_compressed(case_out/'trajectory.npz', positions=np.array(all_positions+[positions]),
                        vertices=np.array(all_vertices+[env.cargoes[0].vertices.copy()]),
                        velocities=np.array(all_velocities), direction=goal, dt=env.dt)
    result = dict(seed=seed, frames=frames, T=env.t, goal_direction=goal.tolist(),
            gradient2_initial=samples[0]['gradient2'], gradient2_final=samples[-1]['gradient2'],
            gradient2_average=samples[-1]['gradient2_time_average'],
            centroid2_initial=samples[0]['centroid2'], centroid2_final=samples[-1]['centroid2'],
            centroid2_average=samples[-1]['centroid2_time_average'],
            max_gradient2=max(s['gradient2'] for s in samples),
            max_centroid2=max(s['centroid2'] for s in samples),
            clip_events=sum(s['clipping_agents'] for s in safety),
            min_continuous_hold_distance=min(s['min_distance_during_held_step'] for s in safety),
            min_pair_halfrow=min(s['min_pair_halfrow'] for s in safety),
            saturation_fraction=sum(s['nominal_saturated_agents'] for s in safety)/(frames*len(env.agents)),
            max_reference_centroid_refinement_error=max(s['centroid_max_difference'] for s in refinements),
            observer_stride_frames=stride, observer_period_seconds=stride*env.dt,
            solver=summary['solver'], final_phase=summary['phases'],
            wall_seconds=time.perf_counter()-started)
    dump(case_out/'audit_summary.json', result)
    print(json.dumps(result, ensure_ascii=False), flush=True)
    return result


def make_plot(out, results, values):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = ['#126E82', '#B85C38', '#6847A0']
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5))
    for case, color in zip(results, colors):
        data = np.genfromtxt(out/('seed_'+str(case['seed']))/'residuals.csv', delimiter=',', names=True)
        for row, metric in enumerate(['gradient2', 'centroid2']):
            axes[row, 0].plot(data['time'], data[metric], color=color, alpha=.30, lw=1)
            axes[row, 0].plot(data['time'], data[metric+'_time_average'], color=color, lw=2,
                              label='seed '+str(case['seed'])+' time average')
    for i, metric in enumerate(['gradient', 'centroid']):
        ax = axes[i, 1]
        obs = [r[metric+'2_average'] for r in results]
        geometric = values['B_gradient_geometry_total'] if i == 0 else values['B_centroid_geometry']
        theory = values['B_'+metric+'_static']
        labels = ['seed '+str(r['seed']) for r in results] + ['geometry', 'static bound*']
        vals = obs+[geometric, theory]
        ax.bar(labels, vals, color=colors[:len(results)]+['#64748B', '#BF3650'])
        ax.set_yscale('log')
        for x, y in enumerate(vals):
            ax.text(x, y*1.25, f'{y:.3g}', ha='center', fontsize=9)
        ax.set_ylim(max(min(obs)*.45, 1e-8), max(vals)*8)
        ax.set_title('Same quantity, logarithmic scale')
    axes[0, 0].set_title('Reference squared gradient: instantaneous and time average')
    axes[1, 0].set_title('Reference squared centroid residual: instantaneous and time average')
    axes[0, 0].set_ylabel(r'$\|g^\star\|^2$')
    axes[1, 0].set_ylabel(r'$\sum_i\|p_i-c_i^\star\|^2$')
    for ax in axes[:, 0]:
        ax.set_xlabel('Time (s)'); ax.legend(fontsize=8); ax.grid(alpha=.18)
    fig.suptitle('DBACT proof audit | unchanged default controller | 3 seeds | N = 16', fontsize=14)
    fig.text(.05, .015, '* Static asymptotic expression with the speed-only disturbance bound. Moving-case guarantee is not certified.\n'
             'Residuals: independent numerical reference observer every 0.25 s; averages: trapezoidal estimates. Faint lines: instantaneous values.', fontsize=9)
    fig.tight_layout(rect=(0, .07, 1, .95))
    fig.savefig(out/'theorem_vs_observed.png', dpi=180)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    parser.add_argument('--frames', type=int, default=600)
    parser.add_argument('--seeds', nargs='+', type=int, default=[2, 5, 8])
    parser.add_argument('--observer-stride', type=int, default=5)
    args = parser.parse_args()
    args.repo = args.repo.resolve(); args.out.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(args.repo/'src'))
    config_path = args.repo/'configs/sim/d/l_shape_closed_loop.yaml'
    config = yaml.safe_load(config_path.read_text())
    check = observer_check(config)
    dump(args.out/'observer_validation.json', check)
    c = constants(config, args.repo)
    dump(args.out/'constants.json', c)
    dump(args.out/'counterexamples.json', counterexamples(config))
    git = lambda *x: subprocess.check_output(['git', '-C', str(args.repo), *x], text=True).strip()
    dump(args.out/'provenance.json', dict(commit=git('rev-parse', 'HEAD'),
        dirty=bool(git('status', '--porcelain')), config_path=str(config_path.relative_to(args.repo)),
        config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(),
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(), seeds=args.seeds,
        frames=args.frames, observer_stride=args.observer_stride,
        reference='PDF equation (1), true polygon edges, fixed task direction and configured fixed dc/dlead',
        scope='Diagnostic runs of current sampled/clipped controller; no claim of Theorem 1 applicability.'))
    results = [run_case(config, args.repo, args.out, s, args.frames, args.observer_stride) for s in args.seeds]
    dump(args.out/'run_audit_summary.json', results)
    make_plot(args.out, results, c)


if __name__ == '__main__':
    main()
