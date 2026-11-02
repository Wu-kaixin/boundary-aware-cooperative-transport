"""Publish the fixed support-screen result, not fictitious transport outcomes."""
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
import reference_support_study as s
from audit_reference_motion import local_terms


LABELS={'far_c_open_away_pos0':'Far C, opening away','far_skew_box_s0_r0_pos0':'Far skew box',
        'near_circle':'Near circle','near_ellipse_parallel':'Near ellipse, parallel',
        'near_ellipse_perpendicular':'Near ellipse, perpendicular','near_rectangle_parallel':'Near rectangle',
        'near_l_reference':'Near L','near_c_open_away':'Near C, opening away'}


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(map(str,row))+' |' for row in rows])


def hold_decomposition(case,result,series):
    """Post-hoc evaluator accounting, excluded from the frozen adoption gate."""
    old=s.read(s.OLD);folder=s.ROOT/old['run_dir']/case['case_id']
    meta=s.read(folder/'paired_metrics.json')['summary']['reference_initializations']
    with np.load(folder/'replay.npz',allow_pickle=False) as z:
        xy=z['cargo/cargo_0/centers'];angles=z['cargo/cargo_0/angles'];start=z['task/cargo_0/start']
    k=result['first_hold']['frame'];rows=[]
    for arm in (0,1):
        terms=[];actual=[]
        for i,aid in enumerate(sorted(meta)):
            if not np.isfinite(series['local_displacements'][arm,k,i]).all():continue
            m=meta[aid];birth=int(round(m['time']/case['config']['dt']))
            d=dict(reference_displacement=series['local_displacements'][arm,k,i],reference_angle=series['local_angles'][arm,k,i])
            terms.append(local_terms(m,d,angles[k],xy[k],xy[birth],angles[birth],start));actual.append(d['reference_displacement'])
        # Explicit accounting order: B, then R, then Q. This is not a causal split.
        b=np.array([t['initial'] for t in terms]);r=np.array([t['point_rotation'] for t in terms])
        B=np.median(b,axis=0);BR=np.median(b+r,axis=0)
        err=np.median(actual,axis=0)-(xy[k]-start)
        rows.append(dict(arm=('original','support_gate')[arm],voters=len(actual),bootstrap_component_m=B.tolist(),
            rotation_component_m=(BR-B).tolist(),registration_remainder_m=(err-BR).tolist(),total_error_m=err.tolist()))
    return rows


def publish():
    a=s.read(s.AUDIT/'offline_analysis.json');f=s.read(s.AUDIT/'offline_freeze.json')
    assert a['status']=='FAIL','This report describes the stopped offline candidate only'
    assert s.source_hashes()==f['source_files'] and s.sha(s.OLD)==f['original_protocol_sha256']
    if (s.PUBLIC/'README.md').exists():raise RuntimeError('Preserve completed report')
    s.PUBLIC.mkdir(parents=True,exist_ok=True)
    s.dump(s.PUBLIC/'analysis.json',a)
    rows=[];decompositions={};pairs={}
    for c in a['cases']:
        path=s.OUT/'offline'/(c['case']+'.npz');assert s.sha(path)==c['series_sha256']
        with np.load(path,allow_pickle=False) as z:pairs[c['condition']]={k:z[k].copy() for k in z.files}
        rows.append(dict(condition=c['condition'],original_MAE_mm=c['original_same_input_mae_m']*1000,
            candidate_MAE_mm=c['candidate_same_input_mae_m']*1000,relative_availability=c['availability_relative_to_original'],
            original_bad_local_readouts=c['original_local_bad_fit_count'],candidate_bad_local_readouts=c['candidate_local_bad_fit_count'],
            original_local_readouts=c['accepted_local_readouts'][0],candidate_local_readouts=c['accepted_local_readouts'][1],
            original_estimated_goal_error_at_old_hold_m=c['first_hold']['original_estimated_goal_error_m'],
            candidate_estimated_goal_error_at_old_hold_m=c['first_hold']['candidate_estimated_goal_error_m'],
            actual_goal_error_at_old_hold_m=c['first_hold']['true_goal_error_m']))
        if c['condition'].startswith('far'):
            case=next(x for x in f['cases'] if x['case_id']==c['case'])
            decompositions[c['condition']]=hold_decomposition(case,c,pairs[c['condition']])
    with (s.PUBLIC/'all_eight_conditions.csv').open('w',newline='',encoding='utf-8') as out:
        w=csv.DictWriter(out,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    s.dump(s.PUBLIC/'posthoc_hold_accounting.json',dict(scope='Exploratory fixed-order median accounting after the frozen screen; not used to revise gate or predict closed-loop success.',conditions=decompositions))
    plt.rcParams.update({'font.size':8,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.facecolor':'white'})
    figures={}
    def save(fig,name,caption):
        for ext in ('pdf','svg','png'):fig.savefig(s.PUBLIC/(name+'.'+ext),dpi=220)
        plt.close(fig);figures[name]=dict(caption=caption,analysis_sha256=s.sha(s.PUBLIC/'analysis.json'),
            files={ext:s.sha(s.PUBLIC/(name+'.'+ext)) for ext in ('pdf','svg','png')})
    fig,ax=plt.subplots(figsize=(7.2,3.4),layout='constrained')
    for i,r in enumerate(rows):
        ax.plot([r['original_MAE_mm'],r['candidate_MAE_mm']],[i,i],color='.5',lw=1)
    ax.scatter([r['original_MAE_mm'] for r in rows],range(8),color='#0072B2',marker='o',s=32,label='Original')
    ax.scatter([r['candidate_MAE_mm'] for r in rows],range(8),color='#D55E00',marker='x',s=32,label='Support gate')
    ax.set_yticks(range(8),[LABELS[r['condition']] for r in rows]);ax.invert_yaxis()
    ax.set_xlabel('Same-input displacement MAE (mm)');ax.set_xlim(left=0);ax.grid(axis='x',alpha=.2)
    ax.legend(loc='lower right');ax.set_title('Eight reused development conditions; no new physical episodes')
    save(fig,'paired_errors','Paired displacement errors on eight original development trajectories. Both estimators receive identical reconstructed own/one-hop scans and original initialization metadata, with candidate-specific admission and internal state. MAE uses times when both team estimates are available; the candidate is available at all originally available evaluation times in this batch. Frames and local readouts are not independent trials; these are not closed-loop improvements.')
    fig,axes=plt.subplots(2,1,figsize=(7.2,4.2),layout='constrained',sharex=False)
    for ax,condition in zip(axes,['far_c_open_away_pos0','far_skew_box_s0_r0_pos0']):
        z=pairs[condition];truth=z['truth'];est=z['estimates'];t=z['time'];target=np.array([1.2,0.])
        # Verify this convenience axis against the actual permitted task.
        case=next(c for c in f['cases'] if c['condition_id']==condition)
        env=s.SimulationEnvironment(case['config'],seed=case['seed']);task=env.controller.tasks['cargo_0']
        np.testing.assert_allclose(task.direction*task.distance,target,atol=1e-12)
        ax.plot(t,np.linalg.norm(truth-target,axis=1),'k-',lw=1,label='Evaluation truth')
        ax.plot(t,np.linalg.norm(est[0]-target,axis=1),color='#0072B2',lw=1,label='Original estimate')
        ax.plot(t,np.linalg.norm(est[1]-target,axis=1),color='#D55E00',ls='--',lw=1,label='Support-gated estimate')
        c=next(c for c in a['cases'] if c['condition']==condition)
        ax.axvline(c['first_hold']['time_s'],color='.45',ls=':',lw=1)
        ax.set_title(LABELS[condition]+'; dotted line = original HOLD');ax.set_ylabel('Goal distance (m)')
        ax.set_xlabel('Physical time (s)');ax.grid(alpha=.2)
    handles,labels=axes[0].get_legend_handles_labels()
    fig.legend(handles,labels,loc='outside upper center',ncol=3,fontsize=7)
    save(fig,'far_goal_readouts','Goal-distance estimates and evaluation truth on the original C and skew-box trajectories. All recorded physical times are shown without smoothing; NaNs are gaps. Dotted lines mark the original controller HOLD, not a candidate stopping event. The offline candidate rejects the original skew-box position gate at that time, but its subsequent closed-loop trajectory and completion were not executed.')
    s.dump(s.PUBLIC/'figure_manifest.json',dict(script_sha256=s.sha(__file__),figures=figures,formats=['PDF embedded fonts','SVG text','PNG 220dpi'],smoothing='none'))
    main_table=table(['条件','原读数MAE/mm','门控读数MAE/mm','原可用时刻仍可用','局部大误差：原→候选'],
        [[r['condition'],f"{r['original_MAE_mm']:.3f}",f"{r['candidate_MAE_mm']:.3f}",f"{r['relative_availability']:.0%}",
          f"{r['original_bad_local_readouts']} → {r['candidate_bad_local_readouts']}"] for r in rows])
    skew=next(r for r in rows if r['condition']=='far_skew_box_s0_r0_pos0')
    reduction=100*(1-skew['candidate_MAE_mm']/skew['original_MAE_mm'])
    text=f'''# 匹配几何支撑门控：固定同输入试验

**离线筛选FAIL，停止本候选的闭环扩展；新增物理回合0。** 八条已消费开发轨迹、两种估计器、固定参数一次执行。远场斜箱位移MAE改善{reduction:.2f}%，未达到事先25%的门槛；没有下调门槛或追加参数。此前原参考配置7/8的任务完成结果不变，不能把本轮写成8/8。

## 实际发现与最小改动

旧斜箱agent_04的参考点云次轴RMS约8.63mm，错误配准所用点约5–6mm，传感距离噪声标准差10mm。估计法向的波动仍让原矩阵通过秩检查。到首次HOLD，局部刚体读数误差1.348m，拟合残差仅4.53mm。近似线状点云、带噪法向与低残差共存；合成反例也直接复现了“满秩并不代表有可靠二维支撑”。这不是证明所有失败均由此产生。

新增默认关闭的`reference_support_gate`：原五步拟合与裁剪完成后，令X为保留匹配参考点去均值后的矩阵，检查：

```text
minor_rms = sqrt(max(lambda_min(X.T @ X / N), 0))
accept_support = minor_rms > 3 * declared_range_noise_std + 1e-12
```

不通过时不提交该次配准状态、不提供新导航投票，原因记为`support_nearly_linear`。不把旧读数伪装成新观测，不改变初始化时刻、偏移、不可变模板、径向支点、最小二乘、原运动界、投票聚合或停止规则。旧初始化可以保持低支撑模板，因此某些机器人会持续弃权；本轮没有自动重建参考来掩盖可用性损失。

系数3在实现和比较前固定，联系声明的传感噪声尺度；它是保守工程筛选，不是统计置信界、完整可观性证书或安全证明。几何坐标平移与旋转不会改变该次轴RMS。候选不接受物体位姿、形状ID、真接触或评价真值。

## 全部八个条件

{main_table}

局部“大误差”仅指配准读数相对真实刚体变换的误差大于原120mm任务容差，用于离线评价；没有把局部读数次数当成独立样本或新的成功数。全部条件均保留；椭圆平行仍有20条此类读数，说明门控不能识别一切错误。

六个近场案例的团队MAE没有变化。远场C的局部大误差649→0，但团队MAE增加4.31mm；远场斜箱5562→0，团队MAE仅降低9.41mm。这说明排除个别坏投票与消除共同偏差是不同问题；少了误差补偿，中位数甚至会略变差。

原斜箱首次HOLD时真实目标误差135.11mm，原估计18.16mm，新读数27.87mm。新读数在这个历史时刻不满足原20mm控制位置带；这只是同输入位置门控反事实，不是新闭环停止事件，更不能宣称之后一定完成。

## 固定预算与判定

- 离线：8条原轨迹，每条原方法与候选两臂；上限1800秒、一个worker/内部线程1。实际分析约{a['wall_seconds']:.1f}秒。
- 所有条件可用性须至少为原方法的90%；实际均100%，仅限团队在原可用时刻的读数，不代表每个机器人始终可用。
- 斜箱同输入MAE须至少降低25%，并在原错误HOLD处提供不进入20mm带的新读数。前者FAIL，后者PASS。
- 其他七个条件MAE恶化不得超过10mm；全部PASS。
- 只有全部通过才执行8条件×2臂、每回合600秒的16回合闭环。本轮按规则NOT_RUN。未执行的比较不能进入结果分母或当作新成功率。

[冻结离线设计](../../../experiments/ccta2027/reference_support_v1_offline.json) · [全部条件CSV](all_eight_conditions.csv) · [完整门槛与分析](analysis.json)。八个条件是既有开发块，包含圆、椭圆两朝向、矩形、L、近/远场C与斜箱；没有独立验证或部署方法优越性结论。

## 解释边界与后续方向

团队位移仍混合初始化偏差、材料参考点的旋转运动和配准误差。这次候选仅针对匹配点的空间支撑，没有估计物体中心或消除参考原点偏差。附加的[原HOLD时刻记账](posthoc_hold_accounting.json)是在筛选完成后生成的探索性分析，B→R→Q顺序固定；中位数分解次序相关，不是独立因果贡献，不改变冻结判定。

本候选保留为默认关闭的研究选项，不能推广为提高完成率的方法。下一项实质工作应明确如何用允许观测建立可追溯的共同任务参考，或给出参考点转动误差的可用范围；在此之前不再扫描门控倍数、增益或按形状挑选最优开关。新的闭环候选及独立验证需要另行冻结，当前没有后台任务继续运行。

系统仍仅仿真、无附着、自身/一跳观测，沿用既有全队阶段监督；不宣称完全去中心化。任务容差120mm、控制位置带20mm、停稳判据、物理与安全均不变；G500没有新闭环数据。论文题目和稿件本轮保持原字节。

## 验证与复现

原方法{sum(c['original_readouts_exact'] for c in a['cases']):,}条有效读数、{sum(c['original_statuses_exact'] for c in a['cases']):,}条估计器状态精确复现；{sum(c['scan_counts_verified'] for c in a['cases']):,}份自身扫描点数核验。原实际参考模板逐元素一致。扫描由保存姿态及确定性传感噪声重建；不是声称历史逐帧全扫描原件已保存。传感重建与独立评价使用仿真真值，候选入口只收到允许的BoundaryView、时间和原偏移。

基线专项29通过；实现前反例2项按预期失败，1项原本通过；实现后专项28通过；完整核心829通过、2项历史跳过。`pip check`、`git diff --check`通过。既有PyYAML6.0.2与锁6.0.3差异保留，未安装或升级依赖。

- [执行与冻结入口](../../../scripts/reference_support_study.py)：`freeze-offline`、`offline`、`freeze-closed`、`run`。已完成输出会拒绝覆盖；闭环冻结要求离线PASS，本轮禁止强行进入。
- [图表与发布入口](../../../scripts/report_reference_support.py)：`publish`、`finalize`；本次完成后拒绝覆盖。
- [回归测试](../../../tests/test_reference_support.py)。
- [误差比较图](paired_errors.pdf) · [远场时间序列](far_goal_readouts.pdf) · [英文caption与图—数据—源码哈希](figure_manifest.json)。没有平滑、补造轨迹或成功视频。

本轮工程数据在`runs/maintenance/{s.MISSION}/`；冻结、差异、备份和操作记录在`artifacts/migrations/{s.MISSION}/`。旧原件仍在原位置，同盘备份仅用于误操作回滚。没有提交、推送、历史改写或硬件操作。
'''
    (s.PUBLIC/'README.md').write_text(text,encoding='utf-8')
    s.event('publish',sha256=s.sha(s.PUBLIC/'README.md'));print(json.dumps(dict(status=a['status'],report=str(s.PUBLIC/'README.md'))))


def finalize():
    state=s.read(s.AUDIT/'mission_state.json');f=s.read(s.AUDIT/'offline_freeze.json');a=s.read(s.AUDIT/'offline_analysis.json')
    assert state['status']=='IN_PROGRESS' and a['status']=='FAIL'
    assert s.source_hashes()==f['source_files'] and s.environment_versions()==f['environment']
    changed={p for p,h in s.source_hashes().items() if state['source_before'].get(p)!=h}
    assert changed=={'src/dbact/control/controller.py','src/dbact/control/observed_reference.py'}
    assert s.sha(s.ROOT/'.git/index')==state['index_sha256']
    assert subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()==state['head']
    assert subprocess.check_output(['git','show-ref'])==(s.AUDIT/'backup/refs.txt').read_bytes()
    assert {str(p.relative_to(s.ROOT)):s.sha(p) for p in (s.ROOT/'manuscript/ccta2027').rglob('*') if p.is_file()}==state['manuscript']
    assert s.sha(s.AUDIT/'design.json')==f['design_sha256'] and s.sha(s.OLD)==f['original_protocol_sha256']
    assert s.sha(s.ROOT/'scripts/reference_support_study.py')==f['script_sha256']
    assert not (s.ROOT/f'runs/research/{s.MISSION}').exists()
    assert all(s.sha(s.AUDIT/'backup'/n)==h for n,h in state['protected'].items())
    for label in ('baseline','probes','probes-geometry','targeted','checks','offline','pip-check','diff-check','publish-recovered'):
        assert s.read(s.OUT/(label+'-result.json'))['exit_code']==0,label
    batch=s.ROOT/f['original_run_dir'];manifest=s.read(batch/'manifest.json');s.verify_source_snapshot(batch,manifest['source_files'])
    for case in f['cases']:
        folder=batch/case['case_id'];r=s.read(folder/'case_result.json')
        assert s.sha(folder/'case_result.json')==f['receipts'][case['case_id']];s.cached_result(folder,r['identity'])
    entries={'README.md':'docs/results/reference_support_v1/README.md','docs/DATA_INDEX.md':'results/reference_support_v1/README.md',
        'docs/results/README.md':'reference_support_v1/README.md','experiments/ccta2027/README.md':'../../docs/results/reference_support_v1/README.md'}
    for name,link in entries.items():
        path=s.ROOT/name;before=path.read_bytes();assert s.sha(path)==state['protected'][name]
        nl=b'\r\n' if b'\r\n' in before else b'\n';head,sep,tail=before.partition(nl)
        path.write_bytes(head+sep+nl+f'[匹配支撑固定筛选]({link})：8条同输入轨迹已完成；局部大误差减少但未通过团队误差门槛，新增物理回合0，候选默认关闭。'.encode('utf-8')+nl+nl+tail)
    patch=[]
    for name in state['protected']:
        patch.extend(difflib.unified_diff((s.AUDIT/'backup'/name).read_text(encoding='utf-8').splitlines(True),(s.ROOT/name).read_text(encoding='utf-8').splitlines(True),fromfile='before/'+name,tofile='after/'+name))
    (s.AUDIT/'task_changes.patch').write_text(''.join(patch),encoding='utf-8')
    count=0
    for path in [s.PUBLIC/'README.md',*[s.ROOT/n for n in entries]]:
        for target in re.findall(r'\]\(([^)]+)\)',path.read_text(encoding='utf-8')):
            if re.match(r'^[a-zA-Z][a-zA-Z0-9+.-]*:',target) or target.startswith('#'):continue
            relative=unquote(target.split('#')[0].strip('<>'))
            if relative:assert (path.parent/relative).resolve().exists(),(path,target);count+=1
    figs=s.read(s.PUBLIC/'figure_manifest.json');assert figs['script_sha256']==s.sha(__file__)
    for name,item in figs['figures'].items():
        assert item['analysis_sha256']==s.sha(s.PUBLIC/'analysis.json')
        for ext,h in item['files'].items():assert s.sha(s.PUBLIC/(name+'.'+ext))==h
        for page in PdfReader(s.PUBLIC/(name+'.pdf')).pages:
            for font in page['/Resources']['/Font'].get_object().values():
                obj=font.get_object();children=obj.get('/DescendantFonts');desc=(children[0].get_object() if children else obj).get('/FontDescriptor')
                assert desc and any(k in desc.get_object() for k in ('/FontFile','/FontFile2','/FontFile3'))
    subprocess.run(['git','diff','--check'],check=True)
    result=dict(status='COMPLETED_FIXED_SCREEN_CANDIDATE_NOT_ADOPTED',scientific_gate='FAIL',new_physical_episodes=0,
        core_tests='829 passed, 2 historical skips',original_readouts_exact=sum(c['original_readouts_exact'] for c in a['cases']),
        source_sha256=s.encode(s.source_hashes()),changed_runtime=sorted(changed),unchanged='All other runtime/config/lock files, original runs, manuscript, Git HEAD/index/refs.',
        original_cases_verified=8,local_links_checked=count,plots='Both PNGs visually inspected; fonts embedded in PDFs',
        new_bytes={n:sum(p.stat().st_size for p in folder.rglob('*') if p.is_file()) for n,folder in {'audit':s.AUDIT,'engineering':s.OUT,'curated':s.PUBLIC}.items()},
        disk_free_bytes=shutil.disk_usage(s.ROOT).free)
    s.dump(s.AUDIT/'final_integrity.json',result)
    (s.AUDIT/'validation_report.md').write_text('# Validation\n\nPASS: baseline29, targeted28, full core829 (2 historical skips), exact historical estimator replay, input products and source snapshot, pip/diff checks, links and PDF fonts.\nEXPECTED_FAIL: two pre-implementation noisy-line counterexamples.\nCORRECTED_INTRODUCED: first report attempt used bootstrap instead of initial for an existing helper dictionary key; original producer and error log retained, report regenerated from unchanged analysis, no estimator/physics rerun.\nSCIENTIFIC_FAIL: frozen 25% far-skew MAE reduction gate; no threshold changed.\nNOT_RUN_BY_DESIGN:16 conditional closed-loop episodes; no new success/safety/G500 claims.\nFAIL_PREEXISTING: PyYAML6.0.2 vs lock6.0.3; unchanged.\nNOT_APPLICABLE: MAS/hardware.\n',encoding='utf-8')
    (s.AUDIT/'FINAL_REPORT.md').write_text('# Fixed support screen completed\n\n[Results](../../../docs/results/reference_support_v1/README.md). Candidate failed the predeclared adoption screen; no new physical episodes or promoted default. Eight inputs and 54,280 original readings verified. Two runtime files contain one opt-in feature and a diagnostic; defaults remain off. All other runtime/config/lock files, original data, manuscript and Git HEAD/index/refs unchanged. Review task_changes.patch; backup preserves preexisting modifications, including untracked source. No commit/push/deletion. Same-disk protection is not disk-disaster backup.\n',encoding='utf-8')
    (s.AUDIT/'CONTINUE.md').write_text('Fixed screen complete. No running job or pending physical episode. Do not lower the frozen threshold or automatically run16 cases: offline FAIL. Read public README and posthoc accounting; future work must address task-reference origin/rotation and use a new frozen hypothesis. Failed candidate remains opt-in and default-off.\n',encoding='utf-8')
    state.update(status=result['status'],stage='FINALIZED',integrity=result);s.dump(s.AUDIT/'mission_state.json',state)
    s.event('finalize',status=result['status']);print(json.dumps(result))


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['publish','finalize']);args=p.parse_args()
    if args.action=='publish':publish()
    else:finalize()
