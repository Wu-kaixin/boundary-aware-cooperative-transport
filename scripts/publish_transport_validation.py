"""Publish verified validation tables and figures, preserving every failure."""
import json
from pathlib import Path
import shutil

import validate_transport_prior as study
from dbact_sim.experiments.deployment import dump
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Patch
from dbact_sim.experiments.study import cached_result

DEST = study.ROOT / 'docs/results/transport_validation_v1'
LABEL = {'boundary_cvt': 'Boundary CVT', 'local_spacing': 'Spacing', 'nearest_observed_target': 'Nearest'}
CONDITIONS = {
    'near_circle': 'N: circle', 'near_ellipse_parallel': 'N: ellipse, parallel',
    'near_ellipse_perpendicular': 'N: ellipse, perpendicular',
    'near_rectangle_parallel': 'N: rectangle, parallel', 'near_l_reference': 'N: L, reference',
    'near_c_open_away': 'N: C, opening away', 'near_u_tray_s0_r90': 'N: U tray, s0 / 90 deg',
    'near_skew_box_s1_r90': 'N: skew box, s1 / 90 deg',
    'far_circle_pos0': 'F: circle, pos0', 'far_skew_box_s0_r0_pos0': 'F: skew box, pos0',
    'far_c_open_away_pos0': 'F: C, pos0', 'far_c_open_away_pos1': 'F: C, pos1',
}


def audit_stops(starts):
    """Post-hoc diagnosis of all CVT arms and failed controls, never control input.

    Report the navigation input actually used by phase guards, not the distinct
    legacy scalar called final_estimated_progress in the general receipt.
    """
    audits = []
    for row in starts['episodes']:
        if row['method'] != 'boundary_cvt' and row['safe_completed']:
            continue
        folder = study.ROOT / row['case_path']
        assert study.sha(folder / 'case_result.json') == starts['input_receipts_sha256'][row['case_path'] + '/case_result.json']
        raw_receipt = study.read(folder / 'case_result.json')
        result = cached_result(folder, raw_receipt['identity'])
        if result['execution_status'] != 'COMPLETED':
            audits.append(dict(case_path=row['case_path'], status='EXECUTION_ERROR'))
            continue
        steps = study.read(folder / 'paired_metrics.json')['steps']
        cfg = study.read(folder / 'resolved_case.json')['config']
        with np.load(folder / 'replay.npz', allow_pickle=False) as replay:
            centers = replay['cargo/cargo_0/centers'].copy()
            start = replay['task/cargo_0/start'].copy()
            goal = replay['task/cargo_0/goal_point'].copy()
        events = []
        for label, frame in list(result['phase_response']['entered'].items()) + [('FINAL_CONTROL_READOUT', len(steps)-1)]:
            if frame >= len(steps):
                continue
            s = steps[frame]
            nav = s['navigation_supervisor']
            record = dict(event=label, frame=frame, seconds=frame*cfg['dt'],
                phase=s['phase'], actual_goal_error_m=float(np.linalg.norm(goal-centers[frame])),
                actual_displacement=(centers[frame]-start).tolist(),
                legacy_scalar_estimated_progress=s['estimated_progress'],
                actual_phase_guards=s['phase_guards'], contacts=s['contacts'],
                raw_navigation_supervisor=nav)
            if nav is not None:
                record.update(estimated_goal_error_m=float(np.linalg.norm(nav['error'])),
                    navigation_displacement_error_m=float(np.linalg.norm(np.asarray(nav['displacement'])-(centers[frame]-start))),
                    error_vector=(np.asarray(nav['displacement'])-(centers[frame]-start)).tolist())
            events.append(record)
        hold_drift = None
        hold_frame = result['phase_response'].get('hold_frame')
        if hold_frame is not None and hold_frame < len(steps):
            h, f = steps[hold_frame], steps[-1]
            nh, nf = h['navigation_supervisor'], f['navigation_supervisor']
            if nh is not None and nf is not None:
                dh = {d['agent_id']: d for d in h['diagnostics']}
                df = {d['agent_id']: d for d in f['diagnostics']}
                common = sorted(set(nh['voters']) & set(nf['voters']))
                if common:
                    mh = np.median([dh[i]['reference_displacement'] for i in nh['voters']], axis=0)
                    mf = np.median([df[i]['reference_displacement'] for i in nf['voters']], axis=0)
                    assert np.max(np.abs(mh-nh['displacement'])) < 1e-12
                    assert np.max(np.abs(mf-nf['displacement'])) < 1e-12
                    ch = np.median([dh[i]['reference_displacement'] for i in common], axis=0)
                    cf = np.median([df[i]['reference_displacement'] for i in common], axis=0)
                    initial_roster, common_change, final_roster = ch-mh, cf-ch, mf-cf
                    assert np.max(np.abs(initial_roster+common_change+final_roster-(mf-mh))) < 1e-12
                    hold_drift = dict(seconds=(len(steps)-1-hold_frame)*cfg['dt'],
                        physical_translation_m=(centers[len(steps)-1]-centers[hold_frame]).tolist(),
                        observed_team_change_m=(mf-mh).tolist(),
                        remove_noncommon_initial_voters_m=initial_roster.tolist(),
                        common_voter_readout_change_m=common_change.tolist(),
                        add_noncommon_final_voters_m=final_roster.tolist(),
                        initial_voters=nh['voters'], final_voters=nf['voters'], common_voters=common,
                        reading='Exact endpoint accounting for coordinate medians; not an independent causal intervention or sum of vector norms.')
        audits.append(dict(case_path=row['case_path'], condition=row['condition'], method=row['method'], arm=row['arm'],
            endpoint_error_m=row['endpoint_error_m'], safe_completed=row['safe_completed'],
            g500=row['g500'], phase_response=result['phase_response'], events=events,
            post_hold_readout_accounting=hold_drift,
            first_contact_seconds=row['first_contact_seconds'],
            first_forward_motion_seconds=row['first_forward_motion_seconds'],
            input_receipt_sha256=study.sha(folder / 'case_result.json')))
    document = dict(cases=audits, selection='Post-hoc explanatory audit: all 24 CVT episodes plus every unsuccessful control; not independent hypothesis tests.',
        alignment='Control frame k uses pre-integration centers[k], not centers[k+1]. Truth is offline evaluation only.',
        source_script_sha256=study.sha(__file__),
        distinction='Legacy scalar estimated_progress is not the navigation-supervisor displacement or phase_guards.progress in vector-goal mode.')
    dump(DEST / 'stop_audit.json', document)
    return document


def save(fig, name):
    for suffix in ('.pdf', '.svg', '.png'):
        path = DEST / (name + suffix)
        if path.exists():
            raise RuntimeError('Refuse figure overwrite')
        fig.savefig(path, dpi=220, bbox_inches='tight')
    plt.close(fig)


def plots(regression, starts):
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 8,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none'})
    conditions = list(CONDITIONS)
    lookup = {(r['condition'], r['method'], r['arm']): r for r in starts['episodes']}
    columns = [(method, arm) for method in study.METHODS for arm in study.SETTINGS]
    colors = ['#eeeeee', '#b9d9ed', '#b2d8c2', '#f7c6a3', '#ed8d94']
    levels = np.zeros((len(conditions), len(columns)))
    fig, ax = plt.subplots(figsize=(7.16, 6.2))
    for i, condition in enumerate(conditions):
        for j, (method, arm) in enumerate(columns):
            row = lookup[condition, method, arm]
            status = (4 if row['execution_status'] != 'COMPLETED' else 3 if row.get('hard_safety')
                      else 2 if row['safe_completed'] and row['g500'] else 1 if row['safe_completed'] else 0)
            levels[i, j] = status
            flag = ('S' if row['safe_completed'] else 'F') + (' / G' if row['g500'] else ' / -')
            if row.get('hard_safety'):
                flag = 'H / -'
            error = row.get('endpoint_error_m')
            ax.text(j, i, flag + '\n' + ('error' if error is None else f'{error:.3f} m'),
                    ha='center', va='center', fontsize=6.9)
    ax.imshow(levels, cmap=ListedColormap(colors), norm=BoundaryNorm(np.arange(-.5, 5.5), 5), aspect='auto')
    labels = []
    for method, arm in columns:
        t = starts['by_method'][method][arm]
        labels.append(f"{LABEL[method]}\n{'.25' if arm == 'reference' else '.45'} prior\nS {t['safe_completed']}/12; G {t['g500']}/12")
    ax.set_xticks(range(6), labels)
    ax.xaxis.tick_top()
    ax.set_yticks(range(12), [CONDITIONS[c] for c in conditions])
    ax.tick_params(length=0, pad=6)
    ax.set_xticks(np.arange(-.5, 6), minor=True)
    ax.set_yticks(np.arange(-.5, 12), minor=True)
    ax.grid(which='minor', color='white', linewidth=1.8)
    ax.tick_params(which='minor', length=0)
    ax.axhline(7.5, color='#333333', linewidth=1)
    ax.set_title('Frozen new-start check: 12 condition blocks, 72 episodes', pad=55, fontsize=10)
    ax.set_xlabel('S: safe task completion; G: G500 pass; F: incomplete; H: hard-safety violation.\n'
                  'All episodes; final 2-D errors; 600 s limit. One new seed; no population confidence interval.',
                  fontsize=7, labelpad=10)
    ax.legend(handles=[Patch(facecolor=colors[i], label=name) for i, name in enumerate(
        ['Incomplete', 'Safe completion only', 'Safe completion + G500', 'Hard-safety violation'])],
        ncol=2, loc='upper center', bbox_to_anchor=(.45, -.12), frameon=False, fontsize=7)
    save(fig, 'new_start_outcomes')

    order = [c['condition_id'] for c in study.read(study.ROOT / 'experiments/ccta2027/cargo_near_v1.json')['cases']
             if c['variant'] == 'boundary_cvt']
    lookup = {(r['condition'].removeprefix('near_'), r['arm']): r for r in regression['episodes']}
    fig, ax = plt.subplots(figsize=(7.16, 8.7))
    y = np.arange(33)
    a = [lookup[c, 'reference']['endpoint_error_m'] for c in order]
    b = [lookup[c, 'candidate']['endpoint_error_m'] for c in order]
    for i in y:
        ax.plot([a[i], b[i]], [i, i], color='#aaaaaa', linewidth=.8, zorder=1)
    ax.scatter(a, y, marker='o', s=23, facecolors='none', edgecolors='#D55E00', label='Original prior .25', zorder=3)
    ax.scatter(b, y, marker='D', s=17, color='#0072B2', label='Frozen candidate .45', zorder=4)
    ax.axvline(.12, color='black', linestyle='--', linewidth=.8, label='Unchanged goal tolerance (.12 m)')
    ax.set_yticks(y, [c.replace('_', ' ') for c in order])
    ax.invert_yaxis()
    ax.set_xscale('log')
    ax.set_xlabel('Final 2-D goal error (m, log scale)')
    ax.set_title('Boundary CVT: all 33 previously used near-field conditions\n'
                 'Original: 31/33 safe, 27/33 G500; candidate: 33/33 safe, 31/33 G500', fontsize=10)
    ax.legend(loc='upper center', bbox_to_anchor=(.5, -.07), ncol=1, frameon=False, fontsize=7)
    ax.grid(axis='x', which='both', alpha=.2)
    save(fig, 'known_condition_pairs')


def main():
    study.guard()
    if (DEST / 'README.md').exists():
        raise RuntimeError('Published report exists')
    regression = study.read(study.ANALYSIS / 'regression/analysis.json')
    starts = study.read(study.ANALYSIS / 'new_starts/analysis.json')
    DEST.mkdir(parents=True, exist_ok=True)
    for stage in ('regression', 'new_starts'):
        for name in ('analysis.json', 'all_episodes.csv'):
            source, dest = study.ANALYSIS / stage / name, DEST / (stage + '_' + name)
            if dest.exists():
                assert study.sha(source) == study.sha(dest)
            else:
                shutil.copy2(source, dest)
                assert study.sha(source) == study.sha(dest)
    stop_audit = audit_stops(starts)
    plots(regression, starts)
    a, b = regression['by_method']['boundary_cvt']['reference'], regression['by_method']['boundary_cvt']['candidate']
    lines = ['# 运动估计先验：旧条件回归与冻结后新初态检查', '',
        '本轮固定0.25与0.45两个既定值，没有继续搜索或组合参数。只做无附着仿真，允许信息、物理、时间步长、600秒上限、二维到达/停稳、安全和G500判据均保持。全局默认值和论文题目未变。', '',
        '## 旧33条件回归', '',
        f"CVT安全完成 **{a['safe_completed']}/33 → {b['safe_completed']}/33**；G500 **{a['g500']}/33 → {b['g500']}/33**；硬安全违规两组均为0；约束缩放{a['barrier_scalings']}→{b['barrier_scalings']}次。",
        '', f"平均终点误差 {a['mean_endpoint_error_m']:.5f}→{b['mean_endpoint_error_m']:.5f} m，但中位数 {a['median_endpoint_error_m']:.5f}→{b['median_endpoint_error_m']:.5f} m，并非每例误差都下降。两项原任务失败被挽回；G500有6项转好、2项转差。",
        '', '只补跑缺少的25个候选回合；33个原参数和8个候选回合核验原始产品哈希及配置后复用，明确保留旧执行身份。原源码与现源码只差已审查的关闭trace扫描修复；所有研究入口都开启trace，上一轮8例全科学记录精确复现作为兼容证据。没有把这33个旧初态说成独立验证。', '',
        '![全部33条件配对终点误差](known_condition_pairs.png)', '',
        '[完整66条比较记录](regression_all_episodes.csv)；[身份、配对与判定](regression_analysis.json)。', '',
        '## 新初态：12条件×3方法×2参数', '',
        '运行前固定seed 20261102；近场8条件采用原scatter规则，远场4条件将机器人网格中心移至[3.75,1.05]并加入标准差0.005 m的位置扰动。每个条件六臂初态哈希相同，各条件均与旧初态不同；所有远场初始扫描为空、位置提示为空。圆、两种椭圆方向、矩形、L、C、U托盘及斜箱均在近场或远场检查中覆盖。', '',
        '这是候选冻结后的新初态检查；几何来自已有开发形状，条件是设计选取且共享随机数，不是任意形状总体的独立同分布抽样。一个新seed不支持一般90%可靠性结论。候选由CVT开发选出，本轮三种方法同参数比较也不是各方法独立充分调参后的排名。', '',
        '| 部署模块 | 安全完成 原→候选 | G500 原→候选 | 硬安全违规 原→候选 | 缩放事件 原→候选 |',
        '| --- | ---: | ---: | ---: | ---: |']
    for method in study.METHODS:
        values = starts['by_method'][method]
        a, b = values['reference'], values['candidate']
        lines.append(f"| {method} | {a['safe_completed']}/12→{b['safe_completed']}/12 | {a['g500']}/12→{b['g500']}/12 | {a['hard_safety_cases']}→{b['hard_safety_cases']} | {a['barrier_scalings']}→{b['barrier_scalings']} |")
    lines += ['', '| 范围 | 方法 | 原安全完成 | 候选安全完成 | 原G500 | 候选G500 |',
              '| --- | --- | ---: | ---: | ---: | ---: |']
    for stratum in ('near', 'far'):
        for method in study.METHODS:
            values = starts['by_stratum'][stratum][method]
            a, b = values['reference'], values['candidate']
            lines.append(f"| {stratum} | {method} | {a['safe_completed']}/{a['cases']} | {b['safe_completed']}/{b['cases']} | {a['g500']}/{a['cases']} | {b['g500']}/{b['cases']} |")
    lines += ['', '候选三方法均11/12安全完成，完成数优势未成立。CVT的G500为9/12、spacing为6/12、nearest为8/12；缩放事件分别5、123、26次。CVT相对nearest在11/12条件的就绪时未覆盖间隙更小（配对中位差-0.384 m），相对spacing在10/12条件更小（-0.157 m）。这些是本批次的部署与过程质量证据；就绪时刻因方法而异，不是共同物理时刻的覆盖优势，也不是formal caging证明。', '',
        '参数收益有代价：新初态CVT安全完成11/12→11/12、G500 9/12→9/12，均值终点误差0.10918→0.03465 m，但缩放2→5次；spacing安全完成10/12→11/12，缩放1→123次。spacing与nearest各挽回2个条件、又各丢失1个原成功条件，不能说所有成功都保持。', '',
        '![全部72个新初态回合](new_start_outcomes.png)', '',
        '[全表含全部失败、持续时长和缺失项](new_starts_all_episodes.csv)；[原始收据、配对及计数](new_starts_analysis.json)。失败时间为终止或观察截止时间，不充当成功完成时间。', '',
        f"事前旧条件门槛：{'PASS' if regression['frozen_stage_gate'] else 'FAIL'}；新初态CVT参数改进门槛：{'PASS' if starts['frozen_stage_gate'] else 'FAIL'}；候选CVT相对两个部署对照均净多完成至少2个条件的门槛：{'PASS' if starts['method_advantage_gate'] else 'FAIL'}。不得为通过而改动门槛。", '',
        '## 原G500保留的失败', '',
        'G500与安全到达CONTACT_READY、实际接触和最终二维完成分开。下表列出新初态中候选CVT的所有G500失败；其他方法全部原因见CSV。TRANSPORT缺失可能对应阶段机从CONTACT_READY直接制动的合法安全分支，不能补写一个不存在的阶段。', '',
        '| 条件 | 安全完成 | 终点误差/m | 原G500原因 |', '| --- | --- | ---: | --- |']
    for r in starts['episodes']:
        if r['method'] == 'boundary_cvt' and r['arm'] == 'candidate' and not r['g500']:
            lines.append(f"| {r['condition']} | {r['safe_completed']} | {r.get('endpoint_error_m', float('nan')):.5f} | {r.get('g500_reasons', 'EXECUTION_ERROR').replace('|', ';')} |")
    lines += ['', '旧条件候选的两项G500失败是T-load s0/r0的一次约束缩放（最小因子0.910），以及U-tray s1/r90峰值严格覆盖0.69375<0.70。它们仍完成了实际运输与停止；这没有取消合同失败。', '',
        '## 停止时实际用了什么估计', '',
        '[停止事件审计](stop_audit.json)检查全部24个CVT回合及所有未安全完成的对照，按控制前的t[k]对齐真值。逐项保留进入CONTACT_READY、TRANSPORT、BRAKE、HOLD和最后读数时的真实目标误差、实际导航估计与阶段守卫。日志中的旧标量estimated_progress不等于二维导航监督器实际使用的位移，不能拿它替代控制输入诊断。', '',
        '## 复现、信息权限与结论边界', '',
        '0.45调整的是地图/参考位移估计共用的运动速度先验，不能称为安全速度界放宽；安全对象速度界、原机器人速度与物理参数未改。两通道共同变化，本轮不能单独归因到某一个配准或融合模块。各方法共用自身/一跳观测和既有全队阶段监督，未新增物体真值反馈，不能声称完整闭环完全去中心化。', '',
        '本轮未改运行源码。旧“材料参考点不等于任务质心”的旋转语义问题仍需与配准误差分开研究；参数改变后的闭环成功不能代替该问题的一般性证明。全局默认保持0.25；0.45仅存在于冻结候选配置。', '',
        '- [回归协议](../../../experiments/ccta2027/transport_validation_v1_regression.json)',
        '- [新初态协议](../../../experiments/ccta2027/transport_validation_v1_new_starts.json)',
        '- [冻结、运行、环境与恢复保护入口](../../../scripts/validate_transport_prior.py)',
        '- [分析入口](../../../scripts/analyze_transport_validation.py)',
        '- [图表/报告生成入口](../../../scripts/publish_transport_validation.py)',
        '- [图表英文caption及数据哈希](figure_manifest.json)', '',
        f'新原始数据在 `runs/research/{study.MISSION}/`；完整本地设计/备份/命令/状态在 `artifacts/migrations/{study.MISSION}/` 与 `runs/maintenance/{study.MISSION}/`。勿重复冻结、覆盖或把已完成批次追加成新的调参实验。']
    skew = next(r for r in stop_audit['cases'] if r.get('condition') == 'far_skew_box_s0_r0_pos0'
                and r.get('method') == 'boundary_cvt' and r.get('arm') == 'candidate')
    hold = next(e for e in skew['events'] if e['event'] == 'HOLD')
    message = (f"远场斜箱候选在{hold['seconds']:.2f}秒进入HOLD时，导航估计目标误差仅"
               f"{hold['estimated_goal_error_m']:.5f} m，真实误差却为{hold['actual_goal_error_m']:.5f} m；"
               f"最终{skew['endpoint_error_m']:.5f} m仍超出0.12 m。它把原参数的0.93612 m大误差显著降低，"
               '却没有救回任务。之后物体已停止，局部参考读数与投票集合仍变化；精确向量记账见审计JSON。'
               '这支持优先研究停止估计的一致性/偏差及确认逻辑，而非继续延长物理预算。'
               '不能仅凭一致性证书宣称估计无偏，也不能把该偏差直接归因于某一个通道。')
    lines[lines.index('## 复现、信息权限与结论边界'):lines.index('## 复现、信息权限与结论边界')] = [message, '']
    (DEST / 'README.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    captions = dict(known_condition_pairs='Paired endpoint errors for all 33 previously used near-field conditions with boundary CVT. Historical executions are reused only after configuration and product verification and retain their original identities; 25 missing candidate episodes are newly executed. The logarithmic axis includes all endpoints and the unchanged 0.12 m task tolerance. These development conditions do not estimate out-of-distribution reliability.',
        new_start_outcomes='All 72 episodes in 12 predeclared new-start condition blocks. Each block pairs three deployment modules and two motion-estimation priors using identical physical initialization. S denotes safe task completion, G the unchanged G500 contract, F incomplete tasks, and H hard-safety violations; endpoint errors include failures. One fixed new seed and designed geometries are used, without population confidence intervals.')
    manifest = dict(captions=captions, inputs={stage: study.sha(study.ANALYSIS / stage / 'analysis.json') for stage in ('regression', 'new_starts')},
        script_sha256=study.sha(__file__), smoothing='None', downsampling='None: every episode',
        intervals='None: shared-seed designed condition blocks, not iid arbitrary-shape draws',
        artifacts={p.name: study.sha(p) for p in DEST.iterdir() if p.suffix in ('.pdf', '.svg', '.png')})
    dump(DEST / 'figure_manifest.json', manifest)
    study.event('publish', directory=DEST.relative_to(study.ROOT).as_posix(), readme_sha256=study.sha(DEST / 'README.md'))
    print(json.dumps(dict(report=DEST.relative_to(study.ROOT).as_posix())), flush=True)


if __name__ == '__main__':
    main()
