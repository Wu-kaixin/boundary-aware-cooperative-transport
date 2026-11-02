"""Publish verified compact evidence without changing the manuscript or old results."""
from pathlib import Path
import json,shutil
from analyze_dynamic_contact_study import read,sha
from freeze_contact_reinforcement import ROOT,MISSION,ARMS,CONDITIONS


def publish():
    source=ROOT/'artifacts/research'/MISSION/'analysis'
    r=read(source/'analysis.json');plan=read(ROOT/'experiments/ccta2027/contact_reinforcement_v1.json')
    diagnostic=read(source.parent/'outcome_diagnosis.json')
    assert r['repeat_compatibility'] and len(r['episodes'])==9
    assert r['provenance']['protocol_sha256']==sha(ROOT/'experiments/ccta2027/contact_reinforcement_v1.json')
    audit=ROOT/'artifacts/migrations'/MISSION
    protected=read(audit/'backup_manifest.json')
    for name in ('README.md','docs/DATA_INDEX.md','docs/results/README.md','experiments/ccta2027/README.md'):
        if sha(ROOT/name)!=protected[name]:raise RuntimeError('Navigation file changed since protection: '+name)
    target=ROOT/'docs/results/contact_reinforcement_v1';target.mkdir(exist_ok=False)
    copies=['analysis.json','all_9_episodes.csv','paired_comparisons.csv']
    copies += [f'{stem}.{ext}' for stem in ('complete_trajectories','task_and_contact_timeseries') for ext in ('pdf','svg','png')]
    mapping={}
    for name in copies:
        shutil.copy2(source/name,target/name);assert sha(source/name)==sha(target/name)
        mapping[name]=dict(source=(source/name).relative_to(ROOT).as_posix(),sha256=sha(target/name))
    shutil.copy2(source.parent/'outcome_diagnosis.json',target/'outcome_diagnosis.json')
    assert sha(source.parent/'outcome_diagnosis.json')==sha(target/'outcome_diagnosis.json')
    mapping['outcome_diagnosis.json']=dict(source=(source.parent/'outcome_diagnosis.json').relative_to(ROOT).as_posix(),sha256=sha(target/'outcome_diagnosis.json'))
    lookup={(e['condition'],e['arm']):e for e in r['episodes']}
    lines=['| 条件 | 原参考 | 仅修恢复 | 修复＋保留推手及增援 |','|---|---|---|---|']
    for n in CONDITIONS:
        cells=[]
        for a in ARMS:
            e=lookup[n,a]
            cells.append(f"{e.get('outcome',e['execution_status'])}；{e.get('endpoint_error_m',float('nan')):.4f} m；G500={e['g500']}；安全硬违规={e.get('hard_safety') or '无'}")
        lines.append('| '+n+' | '+' | '.join(cells)+' |')
    table='\n'.join(lines)+'\n';(target/'capability_table.md').write_text(table,encoding='utf8')
    mechlines=['| 增援臂条件 | 启动增援 | 到位事件 | 群体响应确认事件 | 方向不再有利而退出 | 回退事件 |',
               '|---|---:|---:|---:|---:|---:|']
    for n in CONDITIONS:
        e=lookup[n,'reinforcement'];m=r['mechanisms'].get(n+'_reinforcement',{}).get('events',{})
        mechlines.append(f"| {n} | {e.get('recruitment_attempts',0)} | {m.get('arrived_not_confirmed',0)} | {e.get('response_confirmations',0)} | {e.get('direction_role_exits',0)} | {e.get('rollback_events',0)} |")
    mechanism_table='\n'.join(mechlines)
    estimates=['| 增援臂条件 | 全程所选位移MAE（m） | 峰值位移误差（m） | 首次BRAKE时真实目标误差（m） |',
               '|---|---:|---:|---:|']
    for e in diagnostic['episodes']:
        if e['arm']!='reinforcement':continue
        brake='未进入' if e['first_BRAKE_physical_goal_error_m'] is None else f"{e['first_BRAKE_physical_goal_error_m']:.4f}"
        estimates.append(f"| {e['condition']} | {e['selected_displacement_MAE_m']:.4f} | {e['selected_displacement_max_error_m']:.4f} | {brake} |")
    estimate_table='\n'.join(estimates)
    counts='、'.join(f"{label} {r['counts'][a]['completed']}/3" for a,label in zip(ARMS,('原参考','仅修恢复','修复加增援')))
    rescued='、'.join(r['rescued_conditions']) or '无'
    decision=('达到本轮开发门槛，但只支持继续独立验证，不能推广默认方法。' if r['development_gate'] else
              '未达到本轮开发门槛。停止这一冻结候选的继续调参，保留实验开关，默认关闭。')
    text=f'''# 恢复修复与增援优先：九回合固定开发比较

本轮完成数：{counts}。增援安全完成且原参考未完成的条件：{rescued}。{decision}

圆形、椭圆平行和 C 背向，seed 2，三臂共九回合；每例最多600秒物理时间。
这是事先选定的失败集，不是三次独立同分布试验，也不是完整九条件成功率。
没有新增种子、延长预算或根据结果调参。原始失败、超时和安全结果均保留。

原参考是上一轮 **both-reference vector CVT**，不是历史 scalar CVT 或 spacing。
三个原参考重跑的25个 replay 数组及全部历史逐帧字段精确复现。
恢复臂只修复暂停期间的循环计数依赖；增援臂还保留原推手接触点、抑制该处
切向 CVT 移动并招募替补。因此不能把差异全归因于“机器人数量增加”。

{table}

`COMPLETED` 使用原到达＋停稳判据；`SETTLED_OFF_TARGET` 是目标外停稳；
`CENSORED_600S` 是观察截止仍未完成，不证明永远不能完成。G500单独报告。
安全硬违规独立核验：间距、进入物体、穿透预算、求解器失败；屏障缩放次数仍
保留在表格和G500结果中，不被硬约束汇总隐藏。所有启动回合都在分母。

## 从日志可以说明什么

旧日志全帧重建显示圆形、椭圆平行没有暂停；C背向暂停时最多两个几何合格推手，
少于原四人恢复门槛。修恢复本身不能凭空增加接触。最后10秒的力审查只是该
窗口的物理证据，不能当作全回合接触力证明。

增援只使用自身观测、局部估计和一跳消息；公共阶段监督器依旧存在。
没有附着、负法向力或真值控制输入，物理和安全/成功阈值未改。
几何就绪、推压指令、实际接触及群体运动响应分别记录。群体运动响应不是
某一个机器人确实产生推力或完成接管的证书。

{mechanism_table}

上述事件包含同一机器人反复进入/退出同一角色，不能当作独立成功次数。
“方向不再有利”表示本候选读取的局部法向与局部目标方向不满足保留条件，
不等于已证明真实接触失效。角色切换频繁属于次要描述性诊断，不能单凭
计数断言其造成了任务失败，也不能据此在本轮继续增加滞回参数重跑。

未测试候选对原六个成功条件的影响，不能与旧成功拼出新总体成功率。
论文题目和稿件未改。本轮只做仿真，没有MAS实机操作。

## 数据和复现

- [全部9回合](all_9_episodes.csv)；[配对差值](paired_comparisons.csv)；[完整分析与校验](analysis.json)。
- [研究判断与尚未解决的问题](DECISION.md)。
- [事后位移误差与首次制动诊断](outcome_diagnosis.json)。
- [完整轨迹PDF](complete_trajectories.pdf)；[时间序列PDF](task_and_contact_timeseries.pdf)。
- [冻结协议](../../../experiments/ccta2027/contact_reinforcement_v1.json)；[运行前设计](../../CONTACT_REINFORCEMENT_RESEARCH_2026-11-01.md)。
- 原始日志、轨迹、源码快照和收据：`runs/research/{MISSION}/run/`。
- 全分析：`artifacts/research/{MISSION}/analysis/`；旧日志诊断：同级`diagnosis/`。
- 恢复和审计：`artifacts/migrations/{MISSION}/`；测试：`runs/maintenance/{MISSION}/`。

从仓库根使用根环境，只读干运行：

```powershell
.\\.venv\\Scripts\\python.exe -m dbact_sim study --protocol experiments/ccta2027/contact_reinforcement_v1.json --split development --dry-run
```

重新分析时给出尚不存在的任务内输出目录，不覆盖已发布分析：

```powershell
.\\.venv\\Scripts\\python.exe scripts/analyze_contact_reinforcement.py --out artifacts/research/{MISSION}/analysis-verification
```

## Figure captions and scope

**Complete trajectories.** Cargo-center trajectories for three preselected failure
conditions and three paired controller arms (one reused development seed each).
Circles mark task completion; crosses mark termination without completion. The
reference and recovery-only curves coincide exactly in all three conditions.
The
dotted target region uses the original 0.12 m tolerance. All sampled trajectories
are shown without smoothing. The comparison evaluates a bundled retention and
recruitment intervention and does not establish generalization.

**Task and contact time series.** Physical goal error, actual selected displacement
error, and physical contact count on a common physical-time axis. Selected
displacement is compared with the pre-control cargo state; contact samples refer
to the corresponding physics control hold. Missing estimates remain gaps. Curves
end at their real termination time; no stopped method is extended as if running.
These are within-episode diagnostics, not independent statistical samples.

No analytical smoothing or downsampling is applied. All saved values are passed
to Matplotlib 3.11.2. Its default display-path simplification is enabled with
threshold 0.111111111111; numerical extrema and safety checks use the original
arrays and logs, not the rendered curves.

Vector PDF/SVG and PNG previews are generated by
`scripts/analyze_contact_reinforcement.py`; [source mapping](figure_provenance.json)
records data, protocol and analysis hashes. No synthetic trajectories or missing
successes are filled in.
'''
    (target/'README.md').write_text(text,encoding='utf8')
    decision_text=f'''# 本轮研究判断

完成数：{counts}。{decision}

## 能下的结论

1. 恢复计数的循环依赖是可复现的实现问题；修复保留原门槛并通过回归测试。
   但这三例的恢复臂与原参考25个轨迹数组均完全一致。圆形、椭圆未暂停，
   C背向暂停时只有至多两个合格推手，因此逻辑修复没有创造缺少的接触。
2. 增援方案确实执行了保留、招募、移动、到位及回退流程。它测试的是
   保留接触点、抑制该处切向部署和招募的组合，不能从这三臂分出各子机制贡献。
3. 任务完成、G500、硬安全和终点误差分别评价。误差降低不能替代完成，
   群体运动响应不能证明某个替补独立完成了力传递。

{table}

## 次要诊断及不能推出的结论

{mechanism_table}

{estimate_table}

这些MAE沿各臂自身轨迹计算，不能当作同输入的估计器比较。
圆形仍存在明显位移误差；其284.9秒首次进入BRAKE时，真实目标误差约0.743米。
当时控制剩余量为0.4064米；原配置的制动提前量为
`max((1−0.65)×1.2, 0.1833)=0.42 m`，所以记录符合原制动门控。
这连接了估计偏差与该次阶段触发，但不能单凭一次事件解释整条失败轨迹。
BRAKE是制动阶段，不代表已停稳或完成。椭圆平行和C背向的全程平均所选
位移误差约1.4、1.3厘米，仍运输失败，表明这些失败还需要解释实际接触
和持续推进能力，不能统一归因为进度估计。详细首次制动输入见
[事后诊断](outcome_diagnosis.json)，这不是冻结的主要评价指标。

圆形与椭圆平行分别记录1086、765次“方向不再有利”的角色退出。
这是策略输入上的反复切换，不能单凭计数认定真实接触不可靠、
估计器是唯一根因、或直接断言给角色加滞回就能成功。原压力约束、
局部安全几何和可达接触配置仍可能共同限制有效合力。

旧末10秒样本说明物理推进余量不足是需要面对的事实：圆形和C背向的
合力峰值低于该模型的静摩擦启动阈值，椭圆平行仅接近阈值。
这不是全回合力审查，也不是局部控制器已知的力反馈。

## 保留和停止

全部九回合、失败轨迹和安全记录保留。候选保持默认关闭，不继续调整
次数、方向阈值或预算来追逐这三个开发条件的通过。不把旧六个成功
条件与本轮结果拼成未经执行的总体成功率。论文题目不变。

下一项有区分力的工作是**分别核对圆形的位移/速度读出与制动触发，以及
角色退出时局部选中法向/目标方向变化与实际接触力变化是否一致**，
并分别报告无可达观测路径、到位但
无正向接触、接触后失去持续推进三类事件。这只是一项后续建议，
本轮没有执行新候选、额外仿真或按结果调参。
'''
    (target/'DECISION.md').write_text(decision_text,encoding='utf8')
    (target/'figure_provenance.json').write_text(json.dumps(dict(mapping=mapping,protocol_sha256=r['provenance']['protocol_sha256'],
        analysis_sha256=r['provenance']['analysis_sha256'],statistics=r['statistics'],smoothing='none',
        plots='all recorded samples passed to Matplotlib; no frames treated as independent',
        renderer_path_simplification=dict(enabled=True,threshold=0.111111111111,matplotlib='3.11.2'),
        captions='README.md'),indent=2),encoding='utf8')
    main=ROOT/'README.md';s=main.read_text(encoding='utf-8-sig')
    marker='## 当前研究：从局部边界部署到实际搬运和停止\n'
    paragraph=f'\n[最新恢复与增援比较](docs/results/contact_reinforcement_v1/README.md)按固定预算完成三条件×三臂九回合，每例最多600秒：{counts}。{decision}原参考三例全轨迹和历史字段精确复现；全部失败、G500与安全指标见[九回合能力表](docs/results/contact_reinforcement_v1/capability_table.md)。无附着、局部信息和原判据保持，论文题目未改；不将这三个失败条件与历史成功拼成新总体成功率。\n'
    assert marker in s;s=s.replace(marker,marker+paragraph,1).replace('[最新位置/速度分量比较]','[上一轮位置/速度分量比较]',1)
    main.write_text(s,encoding='utf8')
    path=ROOT/'docs/DATA_INDEX.md';s=path.read_text(encoding='utf-8-sig');first,rest=s.split('\n',1)
    path.write_text(first+f'\n\n2026-11-01 恢复与增援固定九回合：原始产品及执行源码在 `runs/research/{MISSION}/run/`，分析和旧日志诊断在 `artifacts/research/{MISSION}/`，恢复备份/审计在同身份 `artifacts/migrations/`，工程测试在 `runs/maintenance/`。完整九行结果和矢量图见[结果入口](results/contact_reinforcement_v1/README.md)。全部失败保留，默认候选关闭，稿件题目不变。\n'+rest,encoding='utf8')
    path=ROOT/'docs/results/README.md';s=path.read_text(encoding='utf-8-sig');marker='| --- | --- |\n'
    assert marker in s;s=s.replace(marker,marker+f'| [contact_reinforcement_v1](contact_reinforcement_v1/README.md) | 三个失败条件的恢复/增援消融，9回合、600秒；{counts}；全部负结果、安全与参考精确复现 |\n',1);path.write_text(s,encoding='utf8')
    path=ROOT/'experiments/ccta2027/README.md';s=path.read_text(encoding='utf-8-sig');first,rest=s.split('\n',1)
    path.write_text(first+f'\n\n## 恢复与增援固定比较\n\n[contact_reinforcement_v1.json](contact_reinforcement_v1.json)已冻结并执行九回合：圆形、椭圆平行、C背向，各比较原双参考CVT、仅修恢复、恢复加保留推手与增援。每例600秒，seed 2，最多8案例进程、内部单线程。{counts}。结果和全部失败见[报告](../../docs/results/contact_reinforcement_v1/README.md)。不重新冻结或追加调参，原成功和安全判据、物理、估计器及稿件题目不变。\n'+rest,encoding='utf8')
    with (audit/'operations.jsonl').open('a',encoding='utf8') as f:
        f.write(json.dumps(dict(operation='publish verified compact evidence',target=target.relative_to(ROOT).as_posix(),files=mapping))+'\n')
    print(json.dumps(dict(published=str(target),counts=r['counts'],gate=r['development_gate']),ensure_ascii=False))


if __name__=='__main__':publish()
