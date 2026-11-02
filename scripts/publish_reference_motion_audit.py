"""Report a completed offline diagnosis without converting oracles into successes."""
import argparse
import csv
import difflib
import json
import re
import shutil
import subprocess
from urllib.parse import unquote
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pypdf import PdfReader
import audit_reference_motion as s
from dbact_sim.experiments.study import cached_result, verify_source_snapshot


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(str(v).replace('|',' / ') for v in row)+' |' for row in rows])


def csv_save(path,rows):
    with path.open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)


def verified_paper_evidence():
    result={};snapshots=set()
    for name in ('regression','new_starts'):
        path=s.ROOT/f'docs/results/transport_validation_v1/{name}_analysis.json';a=s.read(path)
        for relative,h in a['input_receipts_sha256'].items():
            receipt_path=s.ROOT/relative;assert s.sha(receipt_path)==h
            receipt=s.read(receipt_path);cached_result(receipt_path.parent,receipt['identity'])
            parent=receipt_path.parent.parent
            if parent not in snapshots and (parent/'manifest.json').is_file():
                m=s.read(parent/'manifest.json');verify_source_snapshot(parent,m['source_files']);snapshots.add(parent)
        groups={}
        for method,arms in a['by_method'].items():
            groups[method]={}
            for arm,g in arms.items():
                rows=[r for r in a['episodes'] if r['method']==method and r['arm']==arm]
                assert len(rows)==g['cases']
                assert sum(r['safe_completed'] for r in rows)==g['safe_completed']
                assert sum(r['g500'] for r in rows)==g['g500']
                groups[method][arm]={k:g[k] for k in ('cases','safe_completed','g500','barrier_scalings')}
        result[name]=dict(analysis_sha256=s.sha(path),groups=groups,receipt_count=len(a['input_receipts_sha256']),
            scope='Historical execution identities preserved; these records are not independent new experiments of the current source.')
    result['source_snapshot_batches_verified']=len(snapshots)
    s.dump(s.PUBLIC/'paper_evidence_verification.json',result)


def publish():
    if (s.PUBLIC/'README.md').exists():raise RuntimeError('Preserve report')
    a=s.read(s.PUBLIC/'analysis.json');assert 'reporting_correction' in a
    verified_paper_evidence()
    rows=[]
    for c in a['cases']:
        h=c['first_hold']
        row=dict(case=c['case'],condition=c['condition'],arm=c['arm'],admitted_frames=c['admitted_frames'],
                 accepted_local_readouts=c['accepted_local_readouts'],true_goal_error_at_first_hold_m=h['true_goal_error_m'],
                 low_residual_bad_fit_readouts=c['low_residual_bad_fit_readouts'])
        for name in s.CF:
            row.update({name+'_MAE_m':c['statistics'][name]['same_input_MAE_m'],
                        name+'_estimated_goal_error_at_hold_m':h['goal_error_m'][name],
                        name+'_position_gate_at_hold':h['position_gate'][name]})
        rows.append(row)
    csv_save(s.PUBLIC/'all_24_same_input.csv',rows)
    probes=[dict(case=r['case'],**p) for r in a['replays'] for p in r['probes']]
    csv_save(s.PUBLIC/'all_reconstruction_probes.csv',probes)
    arm_names={'reference':'Original','early_reference':'Earlier','unscaled_bootstrap':'Unscaled'}
    cases=sorted([c for c in a['cases'] if c['condition'].startswith('far')],key=lambda c:(c['condition'],list(arm_names).index(c['arm'])))
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.facecolor':'white'})
    figures={}
    def save(fig,name,caption):
        size=fig.get_size_inches().tolist()
        for fmt in ('pdf','svg','png'):fig.savefig(s.PUBLIC/(name+'.'+fmt),dpi=220)
        plt.close(fig)
        figures[name]=dict(caption=caption,size_inches=size,analysis_sha256=s.sha(s.PUBLIC/'analysis.json'),
            files={fmt:s.sha(s.PUBLIC/(name+'.'+fmt)) for fmt in ('pdf','svg','png')})
    values=np.array([[c['statistics'][x]['same_input_MAE_m']*1000 for x in s.CF] for c in cases])
    labels=[('Far C' if 'c_open' in c['condition'] else 'Far skew')+' / '+arm_names[c['arm']] for c in cases]
    fig,ax=plt.subplots(figsize=(7.2,3.35),layout='constrained')
    ax.imshow(values,cmap='Blues',vmin=0,vmax=max(150,float(values.max())),aspect='auto')
    ax.set_yticks(range(6),labels)
    ax.set_xticks(range(5),['Recorded','Ideal rigid\nregistration','Remove\nbootstrap','Remove\npoint rotation','Remove\nboth'])
    for i in range(6):
        for j in range(5):ax.text(j,i,f'{values[i,j]:.2f}',ha='center',va='center',color='white' if values[i,j]>85 else 'black')
    ax.set_title('Same-input displacement MAE (mm): evaluator-only counterfactuals')
    save(fig,'same_input_errors','Mean displacement error on each original trajectory, evaluated at identical post-ready times and with the same original accepted voters. Each row uses one recorded episode; columns after Recorded use evaluation truth to replace registration or remove per-agent bootstrap/point-rotation error before taking the coordinate median. They are not deployable estimators or new task outcomes. Rows are two reused far-field conditions under three historical arms, not six independent reliability trials.')
    fig,ax=plt.subplots(figsize=(7.2,3.9),layout='constrained')
    markers={'reference':'o','early_reference':'^','unscaled_bootstrap':'s'}
    for c in cases:
        points=[p for p in probes if p['case']==c['case'] and p['actual_voter']]
        ax.scatter([p['fitted_rms_m']*1000 for p in points],[p['fit_error_m'] for p in points],
            s=20,alpha=.6,marker=markers[c['arm']],color='#0072B2' if 'c_open' in c['condition'] else '#D55E00',
            label=('C / ' if 'c_open' in c['condition'] else 'Skew / ')+arm_names[c['arm']])
    ax.set_xlabel('Point-to-plane residual RMS (mm)');ax.set_ylabel('Local rigid-readout error (m)')
    ax.set_ylim(bottom=0);ax.set_xlim(left=0);ax.grid(alpha=.2)
    ax.legend(ncol=3,fontsize=7,loc='lower center',bbox_to_anchor=(.5,1.02))
    fig.suptitle('Accepted voters at the predeclared event/extremum probes',fontsize=10)
    save(fig,'residual_vs_motion_error','Local registration readout error versus the accepted fit residual at predeclared first-post-ready, BRAKE, HOLD, final, and maximum-team-remainder probes. Only original actual voters are shown; all probe records are provided in CSV. Small residuals coexist with large rigid-motion errors. Points are correlated diagnostic observations, not independent experimental samples. The gate/trim set may differ at the evaluator true pose, so cross-pose residuals alone do not rank correspondence quality.')
    s.dump(s.PUBLIC/'figure_manifest.json',dict(script_sha256=s.sha(__file__),figures=figures,
        formats=['PDF embedded fonts','SVG text','PNG 220 dpi'],smoothing='None'))
    error_table=table(['条件/原臂','原读数MAE/mm','理想配准/mm','去初始化/mm','去旋转/mm','两项均去/mm'],
                      [[label,*[f'{v:.2f}' for v in line]] for label,line in zip(labels,values)])
    false_cases=[c for c in a['cases'] if c['first_hold']['true_goal_error_m']>.12]
    stop_table=table(['历史失败回合','真实目标误差/mm','原估计/mm','理想配准/mm','去初始化/mm','去旋转/mm','两项均去/mm'],
        [[c['case'],f"{c['first_hold']['true_goal_error_m']*1000:.2f}",
          *[f"{c['first_hold']['goal_error_m'][x]*1000:.2f}" for x in s.CF]] for c in false_cases])
    nread=sum(r['accepted_readouts_exact'] for r in a['replays']);nstates=sum(r['tracker_statuses_exact'] for r in a['replays'])
    nscans=sum(r['scan_counts_checked'] for r in a['replays'])
    text=f'''# 同输入参考运动核验与论文贡献判断

**本轮完成24条既有轨迹的离线消融，以及6条远场轨迹的原配准器精确重放；新增物理回合为0。** 没有新候选被宣称提高成功率。原24回合及其所有失败保持原样，默认控制和论文题目不变。

论文判断见[主张—证据与最少缺口](PAPER_READINESS.md)：现有材料足以形成边界明确的仿真初稿，尚不足以支撑任意形状90%可靠性、CVT完成率优势或完全去中心化闭环。新初态11/12这一设计条件计数不能当作总体成功概率；CVT、spacing、nearest在同批候选设置下均11/12。已额外核对[历史贡献证据的收据、产品与计数](paper_evidence_verification.json)，不把复用记录当作新增执行。

## 本轮实际做了什么

- 所有24条历史轨迹：原采样时间、原接受投票者、原物理轨迹不变，比较5种二维读数；只在CONTACT_READY后参考读数实际生效的帧评价。
- 六条远场轨迹：从保存的机器人/物体姿态和确定性传感噪声重建观测；按记录的初始化时刻与偏移重放原参考配准器。逐项精确复现{nread:,}次有效读数、{nstates:,}个配准状态；核对{nscans:,}份自身扫描点数，实际不可变参考模板逐元素一致。
- 这不是整个控制器或物理的再执行，也不是声称原始逐帧全扫描已经保存。重建后可观察的原配准输出全部精确一致，是这次同输入诊断的核验依据。
- 固定预算为24条分析、6条估计器重放、1800秒墙钟上限，单worker/内部线程1；没有后续调参、增加条件或替换失败。

## 同输入反事实

“理想配准”以真实刚体变换作用于实际记录的局部参考点，但仍保留原初始化偏移和材料参考点旋转。“去初始化”“去旋转”分别从每个机器人的原读数中减去相应真实误差，之后重新取原投票集合的坐标中位数；“两项均去”同时做两者。所有这些列均使用评价真值，**不能部署、不能作为新成功，也不能预测纠正后的闭环轨迹**。

{error_table}

原C形的理想配准MAE仍为106.20mm，取消初始化放大臂仍为145.09mm；说明单纯把配准做准也不会自动消除它们的参考原点和旋转语义误差。原斜箱由125.73降至68.53mm，说明配准误差在此更相关。只有评价用的两项同时去除时，六例团队读数MAE降至约1.66–4.91mm；这表明有值得研究的误差结构，**不是已经实现了毫米级可部署控制器**。

坐标中位数是非线性的。逐机器人去除某项误差会改变中位数对应的成员，因而结果不能由上一轮B/R/Q分量范数简单相加得到。上一轮配准“余项”也不是独立因果效应；局部大误差和团队中位数小误差可以同时存在。

[同输入误差图](same_input_errors.pdf) · [全部24条的数值与门槛](all_24_same_input.csv)。

## 低残差和满秩仍可能接受错误运动

原远场斜箱第1745控制帧，agent_04仍是实际投票者：局部配准读数误差1.34836m，拟合RMS仅4.532mm，原rcond=1e-3下的3列线性化设计矩阵秩为3。角度误差约−0.075度，并不能解释该大幅位移误差。它接受32个对应点；在评价真姿态处有112个对应点。

最后两组对应点经过不同的门控和trim，不能仅比较两组残差断言哪个优化目标更好。这个实际反例足以否定“低残差＋满秩意味着运动估计可靠”，但不能从一个点直接指定唯一错误对应、根因或新阈值。应继续检查局部表面匹配、支持区域和参考坐标的一致性，不能凭这次记录事后选一个匹配比例门槛。

[所有探针散点图](residual_vs_motion_error.pdf) · [全部重建探针](all_reconstruction_probes.csv)。探针是事件与极值选取的相关记录，不是独立统计样本；没有隐去高误差点。

## 原错误停止时刻的读数

{stop_table}

表中是同一历史时刻“距目标还有多少”的估计，控制位置门槛始终按原回合的20mm；独立任务容差仍120mm。三条失败的原读数均进入20mm带，评价用的理想配准读数则均不进入。但即使拒绝那一次停止，后续是否能够到达并停稳仍需要新闭环验证。本轮没有把“反事实拒绝错误停止”计成成功。

## 参考点与目标中心的必要区别

刚体上两个参考点的位移不同。若真实旋转为R、参考点相对目标中心的偏移为delta，位移差为(R−I)delta。在二维：

```text
norm((R(theta)-I) delta) = 2 abs(sin(theta/2)) norm(delta)
```

因此，只知道刚体变换、却不知道被评价中心相对观测参考点的位置时，无法一般性地把两种位移等同。纯平移时该项为零；有旋转时需要参考点关系或误差范围。这个条件性结论不意味着局部感知永远无法推断几何中心，也不否定圆形平移可观性。它是标准刚体运动学，不作为新定理。

代码核查：当前导航计算使用task.direction与task.distance，未用task.start/goal_point补偿该误差。TransportTask对象仍携带这些评价字段，未来可收窄接口防误用；本轮没有以它们修正控制。仿真真值只用于上述离线核验，未进入运行时。

## 继续工作的选择

本轮不再扫描增益或组合失败开关。下一项候选应先给出**允许信息下可执行的任务参考定义和更新方式**，并检验在转动、部分可见和机器人换位时的对应关系/误差范围。可能的路径是从足够的已观测几何建立共同参考，或控制姿态变化使参考点差异受到约束；两条都需要明确假设和新的局部实现，当前均未验证。

先在同一保存/重建输入上验证候选可用性及错误停止读数，冻结后才进入闭环；原成功安全判据、失败C形和斜箱仍保留。主比较和独立验证必须使用一套固定系统；不能按货物类别事后调用各臂的最好结果。论文近期主线保持完整搬运系统，进度/匹配结果承担机制与局限证据。

## 验证、修正与复现

823项核心测试通过、2项历史跳过；新增刚体旋转、初始化偏差、非线性中位数和支点角误差反例测试。随后新增配置覆盖回归，6项离线分析测试通过。

首次离线分析误用了DBACTParams默认位置带30mm，而历史配置明确为20mm。已按冻结执行配置修正position_gate及其计数；原冻结审计协议未改写，修正作为analysis_amendment.json单列。初版脚本、分析和逐帧派生文件均保存在本地审计区；所有数值读数、MAE、重放证据和物理结果保持一致，没有重跑估计器或仿真来修正该报告错误。

- [冻结离线协议](../../../experiments/ccta2027/reference_motion_audit_v1.json)
- [核验入口与单元数学](../../../scripts/audit_reference_motion.py)
- [分析测试](../../../tests/test_reference_motion_audit.py)
- [图表和证据发布入口](../../../scripts/publish_reference_motion_audit.py)
- [完整分析与修正来源](analysis.json)
- [英文caption、数据和脚本哈希](figure_manifest.json)

原始轨迹继续保存在runs/research/2026-11-02-reference-origin-v1/comparison/；本轮工程输出在runs/maintenance/{s.MISSION}/，保护、状态与修正记录在artifacts/migrations/{s.MISSION}/。仅写本仓库，没有实机、推送或论文题目变动。已有PyYAML6.0.2与锁6.0.3差异保留，没有变更环境。
'''
    (s.PUBLIC/'README.md').write_text(text,encoding='utf-8');s.event('publish',sha256=s.sha(s.PUBLIC/'README.md'))
    print(json.dumps(dict(report=str(s.PUBLIC/'README.md'),replayed_readouts=nread)))


def finalize():
    state=s.read(s.AUDIT/'mission_state.json');plan=s.read(s.PROTOCOL);a=s.read(s.PUBLIC/'analysis.json')
    if state['status']=='COMPLETED_OFFLINE_FIXED_INPUT':raise RuntimeError('Already finalized')
    assert s.source_hashes()==plan['source_files'] and s.environment_versions()==plan['environment']
    assert s.sha(s.PROTOCOL)==state['protocol_sha256']
    assert s.sha(s.ROOT/'.git/index')==state['index_sha256']
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==state['head']
    assert subprocess.check_output(['git','show-ref'])==(s.AUDIT/'backup/refs.txt').read_bytes()
    assert {str(p.relative_to(s.ROOT)):s.sha(p) for p in (s.ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()}==state['manuscript']
    assert all(s.sha(s.AUDIT/'backup'/n)==h for n,h in state['protected_originals'].items())
    assert s.sha(s.AUDIT/'backup/audit_reference_motion_before_gate_fix.py')==plan['script_sha256']
    assert a['reporting_correction']['corrected_script_sha256']==s.sha(s.ROOT/'scripts/audit_reference_motion.py')
    for label in ('checks','analysis','gate-fix-tests','correct-gate','publish','pip-check','diff-check'):
        assert s.read(s.OUT/(label+'-result.json'))['exit_code']==0,label
    source=s.read(s.ORIGIN);batch=s.ROOT/source['run_dir']
    verify_source_snapshot(batch,plan['source_files'])
    for case in source['cases']:
        folder=batch/case['case_id'];receipt=s.read(folder/'case_result.json')
        assert s.sha(folder/'case_result.json')==plan['receipt_sha256'][case['case_id']]
        cached_result(folder,receipt['identity'])
        rows=s.read(s.OUT/(case['case_id']+'-timeseries.json'))
        assert all(r['position_gate']=={k:v<=s.position_band(case) for k,v in r['goal_error_m'].items()} for r in rows)
    message='同输入参考运动核验完成：24条旧轨迹、6条远场原配准器精确重放，新增物理回合0；区分初始化、参考点旋转和匹配误差，并核查论文主张与验证缺口。'
    entries={'README.md':'docs/results/reference_motion_audit_v1/README.md','docs/DATA_INDEX.md':'results/reference_motion_audit_v1/README.md',
             'docs/results/README.md':'reference_motion_audit_v1/README.md','experiments/ccta2027/README.md':'../../docs/results/reference_motion_audit_v1/README.md'}
    patch=[]
    for name,link in entries.items():
        path=s.ROOT/name;before=path.read_bytes()
        if link.encode() not in before:
            assert s.sha(path)==state['protected_originals'][name]
            nl=b'\r\n' if b'\r\n' in before else b'\n';head,sep,tail=before.partition(nl)
            path.write_bytes(head+sep+nl+f'[最新参考运动与论文证据核验]({link})：{message}'.encode('utf-8')+nl+nl+tail)
        patch.extend(difflib.unified_diff((s.AUDIT/'backup'/name).read_text(encoding='utf-8').splitlines(True),path.read_text(encoding='utf-8').splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    (s.AUDIT/'documentation_changes.patch').write_text(''.join(patch),encoding='utf-8')
    count=0
    for path in [s.PUBLIC/'README.md',s.PUBLIC/'PAPER_READINESS.md',*[s.ROOT/n for n in entries]]:
        for target in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
            relative=unquote(target.split('#')[0].strip('<>'))
            if relative:assert (path.parent/relative).resolve().exists(),(path,target);count+=1
    figures=s.read(s.PUBLIC/'figure_manifest.json');pdfs={}
    assert figures['script_sha256']==s.sha(__file__)
    for name,f in figures['figures'].items():
        assert f['analysis_sha256']==s.sha(s.PUBLIC/'analysis.json')
        for fmt,h in f['files'].items():assert s.sha(s.PUBLIC/(name+'.'+fmt))==h
        pdf=PdfReader(s.PUBLIC/(name+'.pdf'));fonts=[]
        for page in pdf.pages:
            for font in page['/Resources']['/Font'].get_object().values():
                obj=font.get_object();children=obj.get('/DescendantFonts')
                desc=(children[0].get_object() if children else obj).get('/FontDescriptor')
                assert desc and any(k in desc.get_object() for k in ('/FontFile','/FontFile2','/FontFile3'))
                fonts.append(str(obj['/BaseFont']))
        pdfs[name]=dict(pages=len(pdf.pages),embedded_fonts=fonts)
    subprocess.run(['git','diff','--check'],check=True)
    integrity=dict(source_sha256=s.encode(s.source_hashes()),runtime='All source/config/lock bytes unchanged',git='HEAD/index/refs unchanged',
        manuscript='All bytes and title unchanged',original_trajectories_verified=24,new_physical_episodes=0,
        exact_replays=6,local_links_checked=count,pdfs=pdfs,figures_reviewed='Both PNGs visually inspected',
        sizes_bytes={n:sum(p.stat().st_size for p in root.rglob('*') if p.is_file()) for n,root in {'audit':s.AUDIT,'engineering':s.OUT,'curated':s.PUBLIC}.items()},
        disk_free_bytes=shutil.disk_usage(s.ROOT).free,analysis_amendment='Derived position gate corrected to original20mm; initial30mm analysis preserved',
        scripts={n:s.sha(s.ROOT/'scripts'/n) for n in ('audit_reference_motion.py','publish_reference_motion_audit.py')})
    s.dump(s.AUDIT/'final_integrity.json',integrity)
    (s.AUDIT/'validation_report.md').write_text(
        '# Validation\n\nPASS: 823 full core tests, 2 historical skips; six follow-up analysis/config regression tests. '
        'PASS: exact reconstruction of six original estimator trajectories; all24 original products and source snapshot verified. '
        'PASS: pip check, diff check, figure-data hashes, local links, PDF embedded fonts. '
        'CORRECTED_INTRODUCED: initial offline position gate incorrectly used default30mm; restored explicit executed20mm only in derived flags/counts. '
        'Original protocol/analysis/script/derived data preserved; no physics or estimator rerun. '
        'FAIL_PREEXISTING: PyYAML6.0.2 installed vs6.0.3 lock; no dependency changes. '
        'NOT_APPLICABLE: hardware/MAS; NOT_RUN: new physical simulations and manuscript build.\n',encoding='utf-8')
    (s.AUDIT/'FINAL_REPORT.md').write_text(
        '# Completed fixed-input motion audit\n\n24 historical trajectory analyses and6 exact estimator replays; new physical episodes0. '
        '[Scientific results](../../../docs/results/reference_motion_audit_v1/README.md) and '
        '[paper claim assessment](../../../docs/results/reference_motion_audit_v1/PAPER_READINESS.md).\n\n'
        'No new controller was promoted. All source/config/lock files, prior raw products, manuscript, Git HEAD/refs/index remain unchanged. '
        'Backup contains dirty-worktree/index/ref records and original navigation documents; review documentation_changes.patch before selective restoration. '
        'analysis_v1_default_band/ and backup/audit_reference_motion_before_gate_fix.py preserve the reporting correction provenance. '
        'No deletion of unique research data, commit or push. Same-disk protection is not a disk-disaster backup.\n',encoding='utf-8')
    state.update(status='COMPLETED_OFFLINE_FIXED_INPUT',stage='FINALIZED',integrity=integrity)
    s.dump(s.AUDIT/'mission_state.json',state)
    (s.AUDIT/'CONTINUE.md').write_text('Offline budget complete. No pending run. Read FINAL_REPORT.md and PAPER_READINESS.md. '
        'No new task success or deployable estimator has been established. Keep historical outcomes and original thresholds. '
        'Future work needs an allowed-information task reference and correspondence checks, followed by separately frozen closed-loop and independent validation.\n',encoding='utf-8')
    s.event('finalize',status=state['status']);print(json.dumps(integrity))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['publish','finalize']);a=p.parse_args()
    if a.action=='publish':publish()
    else:finalize()
