"""Publish verified compact evidence without touching frozen history or Git refs."""
import argparse
import json
from pathlib import Path
import shutil
import subprocess
from statistics import median
from prepare_cargo_discovery import ROOT,MISSION,AUDIT,sha
from dbact_sim.experiments.deployment import dump
from freeze_cargo_discovery import METHODS

DEST=ROOT/'docs/results/cargo_discovery_v1'
LABELS={'boundary_cvt':'边界 CVT','local_spacing':'spacing','nearest_observed_target':'nearest',
        'detection_density_cvt':'检测点密度 CVT'}
def read(p):return json.loads(p.read_text(encoding='utf-8'))
def write_new(path,text):
    data=text.encode('utf-8');path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        if path.read_bytes()!=data:raise RuntimeError('Conflicting publication '+str(path))
        return
    path.write_bytes(data)
def copy_new(source,target):
    if target.exists():assert sha(source)==sha(target)
    else:shutil.copy2(source,target)
    assert sha(source)==sha(target)

def publish(dry_run=False):
    research=ROOT/'artifacts/research'/MISSION
    near=read(research/'near-analysis/analysis.json');far=read(research/'far-analysis/analysis.json')
    shadow=read(research/'shadow/analysis.json')
    far_safety=read(research/'far_safety_failure.json')
    state=read(AUDIT/'mission_state.json')
    assert len(near['episodes'])==132 and len(far['episodes'])==12
    refs=subprocess.check_output(['git','--no-optional-locks','show-ref'],cwd=ROOT)
    assert refs==(AUDIT/'backup/refs.txt').read_bytes()
    assert sha(ROOT/'.git/index')==state['index_sha256']
    assert all(sha(ROOT/p)==h for p,h in state['manuscript_sha256'].items())
    checks=[c for c in near['verification'].values() if c]
    assert len(checks)==9 and all(c['historical_replay_exact'] and c['historical_fields_exact'] for c in checks)
    for stage in ('near','far'):
        assert (ROOT/'runs/maintenance'/MISSION/f'{stage}.exit.txt').read_text(encoding='utf-8-sig').strip()=='0'
        receipt=read(AUDIT/'freeze_receipt.json')[stage]
        assert sha(ROOT/receipt['protocol'])==receipt['sha256']
    totals=near['totals']; t=far['totals']['all']
    g500_values='、'.join(f"{totals['all'][m]['g500']}/33" for m in METHODS)
    barrier_cases='、'.join(str(sum(r['barrier_scalings']>0 for r in near['episodes'] if r['arm']==m)) for m in METHODS[:3])
    barrier_events='、'.join(str(sum(r['barrier_scalings'] for r in near['episodes'] if r['arm']==m)) for m in METHODS[:3])
    # Descriptive secondary outcomes: preserve the predeclared primary gate.
    lookup={(r['condition'],r['arm']):r for r in near['episodes']}
    secondary={}
    for cohort in ('all','new_cargo_24'):
        selected=[r for r in near['episodes'] if r['arm']=='boundary_cvt'
                  and (cohort=='all' or r['cohort']==cohort)]
        pairs={}
        for arm in ('local_spacing','nearest_observed_target'):
            differences=[r['ready_gap_upper_m']-lookup[r['condition'],arm]['ready_gap_upper_m'] for r in selected]
            ready=[r['first_contact_ready_seconds']-lookup[r['condition'],arm]['first_contact_ready_seconds'] for r in selected]
            pairs[arm]=dict(n=len(selected),smaller=sum(d<0 for d in differences),
                larger=sum(d>0 for d in differences),equal=sum(d==0 for d in differences),
                median_paired_gap_difference_m=median(differences),median_paired_readiness_difference_s=median(ready))
        secondary[cohort]=pairs
    secondary_lines=['| 新24条件，CVT相对 | 未覆盖弧更小 / 更大 | 配对弧长差中位数 | 配对部署时间差中位数 |',
                     '| --- | ---: | ---: | ---: |']
    for arm,p in secondary['new_cargo_24'].items():
        secondary_lines.append(f"| {LABELS[arm]} | {p['smaller']}/24 / {p['larger']}/24 | {p['median_paired_gap_difference_m']:+.3f} m | {p['median_paired_readiness_difference_s']:+.3f} s |")
    lines=['| 部署模块 | 原九条件安全完成 | 新24条件安全完成 | 全部33条件安全完成 | 任务完成 | G500 | 硬安全失败 |',
           '| --- | ---: | ---: | ---: | ---: | ---: | ---: |']
    for method in METHODS:
        v=totals['all'][method]
        lines.append(f"| {LABELS[method]} | {totals['historical_nine'][method]['safe_completed']}/9 | {totals['new_cargo_24'][method]['safe_completed']}/24 | {v['safe_completed']}/33 | {v['completed']}/33 | {v['g500']}/33 | {v['hard_safety_cases']}/33 |")
    paired=[]
    for p in near['paired']:
        paired.append(f"- 对 {LABELS[p['comparison_arm']]}：边界 CVT 独有安全完成 {len(p['reference_wins'])}，对照独有 {len(p['reference_losses'])}，净差 {p['net']:+d}/33。CVT 失去的完成条件："+('、'.join(p['reference_losses']) or '无')+'。')
    verdict=('达到预先声明的开发优势筛选门槛；仍需要独立初态和未见参数验证，不能据此声称总体成功概率。'
             if near['development_advantage_gate'] else
             '未达到相对全部对照的预先开发优势门槛，不能宣称整体算法优越；应按具体条件与指标报告得失。')
    note='；'.join(f"{LABELS[m]} {totals['all'][m]['safe_completed']}/33" for m in METHODS)
    far_note=f"未知位置 {t['unknown_location']['completed']}/6 完成、{t['unknown_location']['detected']}/6 检测；初始位置提示 {t['initial_location_hint']['completed']}/6 完成、{t['initial_location_hint']['detected']}/6 检测"
    far_note+=f"；硬安全失败：未知位置 {t['unknown_location']['hard_safety_cases']}/6、提示 {t['initial_location_hint']['hard_safety_cases']}/6"
    text=f'''# BACT：不规则货物与远场位置权限比较

本轮完成近场 132 回合及远场 12 回合固定开发试验，seed 2、最多 600 秒/例。近场安全完成：{note}。远场：{far_note}。这些是设计条件上的开发结果，不是任意形状成功概率或独立可靠性验证。论文题目未改，只做仿真、无附着。

[预先设计](../../CARGO_DISCOVERY_RESEARCH_2026-11-02.md)；[近场冻结协议](../../../experiments/ccta2027/cargo_near_v1.json)；[远场冻结协议](../../../experiments/ccta2027/cargo_far_v1.json)。全部失败、异常身份、执行源码和原始轨迹保留。

## 边界 CVT 究竟多解决了什么

{chr(10).join(lines)}

{chr(10).join(paired)}

{verdict}

更具体的次要指标是部署分布，而不是额外完成条件：

{chr(10).join(secondary_lines)}

差值均为 CVT 减对照，按条件先配对再取中位数，不是两个组中位数之差。弧长来自各方法首次 CONTACT_READY 后真实状态的360点等弧长诊断、0.42米外侧机器人覆盖半径；它不等于 formal caging，也不是同一物理时刻的比较。部署时间代价单列。完整总体及新条件的描述性统计见 [次要配对指标](secondary_metrics.json)。当前证据支持在这些条件下改善边界分布，不能把它改写为完成率优势或动态安全保证。

G500按上表方法顺序分别为{g500_values}，仍是独立的既有合同指标。前三种方法触发 barrier scaling 的条件数分别为{barrier_cases}，总事件数分别为{barrier_events}；所以“发生事件的条件更少”和“事件总数更少”并不等价，也不能笼统宣称 CVT 对所有安全代价都更优。各回合具体G500失败原因保留在CSV。

比较只改变首次 CONTACT_READY 之前的部署目标，后续运输、位移估计、压力、物理、安全和成功规则共同。原九个 CVT 参考回合的 25 个 replay 数组及原逐帧科学字段全部精确复现；共同安全修复不能计作 CVT 的独有创新。

边界密度的支撑是观测边界的任务方向偏置点，并按弧长、置信度和间隙加权；spacing/nearest 使用同一组边界偏置点但改变目标选择。检测密度 CVT 把支撑改为自身/一跳新鲜检测机器人的位置，保持核宽、背景和总核质量。它受 [Song 等人原式 (5)-(6)](https://arxiv.org/html/2602.19070v2) 启发，但保留本系统公共边界质量与门控，是表示机制对照，不能宣称完整复现或战胜该论文的方法。

本轮没有分别优化各方法的参数，所以比较回答的是固定共同参数下的部署表示作用，不是各方法最佳调参版本的排名。特别是检测密度需要与其原始招募/控制流程一起评价，不能从这一替代接口的失败推断原论文算法必然失败。

对检测密度的完成差同时出现在 CVT、spacing 和 nearest 三种边界目标方法中，因此更直接支持边界目标信息在本流程中的作用，不能单独归功于 CVT 求解器。CVT 相对另外两种边界目标方法目前更明确的证据，是上述部署间隙的条件配对差异；完成数依然相同。

全部条件和指标见[近场 CSV](near_episodes.csv)、[近场可追溯分析](near_analysis.json)、[结果矩阵](near_all_outcomes.pdf)和[配对指标](near_paired_metrics.pdf)。部署图和完成结果必须合看；几何覆盖、物理接触、收定、G500和任务完成不互相替代。

## 未知外形与未知位置分开回答

近场包含原五形状九条件，以及六种货物族、两个几何实例、两个朝向的新 24 条件。均等面积/质量，公开周长、惯量、尺寸；新旧初态层次分开。形状是预先设计的刚体多边形，不是任意形状总体随机样本。[几何图](cargo_geometry_atlas.pdf)。

远场采用圆、斜边箱、C 背向三种几何、两个位置。所有机器人初始扫描为空，最近初始边界距离超过 3.12 米。两臂物理初态相同，只在提示臂提供一次初始位置 token，12 秒 TTL不变；不提供轮廓或后续位姿真值。{far_note}。完整配对见[远场 CSV](far_episodes.csv)、[位置权限图](far_location_pairs.pdf)、[实际过程图](far_full_process.pdf)。发现成功不代表运输完成。

两臂各六例全部检测、达到部署阶段并发生真实接触，但各只有三例完成，G500各为2/6。C背向在位置0由提示帮助完成，在位置1反而是未知位置臂完成而提示臂失败；这排除了“该形状标签必然失败”的说法，但没有单独识别接触配置、观测历史或估计误差中的因果因素。

**硬安全反例必须保留：** `c_open_away_pos1_initial_location_hint` 的终点误差为 {next(r['endpoint_error_m'] for r in far['episodes'] if r['condition']=='c_open_away_pos1' and r['arm']=='initial_location_hint'):.4f} 米。已保存帧中的首次机器人中心入物体发生在 {far_safety['first_saved_center_inside']['time_s']:.2f} 秒、BRAKE阶段；全记录最大穿透 {far_safety['original_contract']['max_penetration']:.4f} 米，超过原预算 {far_safety['original_contract']['penetration_budget']:.4f} 米。原求解器没有fallback或infeasible记录，仍不能据此推导物理安全。见[离线时序核验](far_safety_failure.json)，生成入口 `scripts/audit_cargo_far_safety.py`。这是一项未解决的研究限制，不是已经修好的案例；没有放宽阈值或重跑到成功。

活动控制仍采用自身/一跳观测及原全队阶段监督器；已知工作空间边界、任务方向/距离及机器人资源。没有先验外形不等于没有假设，远场实验也不证明任意复杂环境的完整搜索保证。

本轮只有一个货物，传感量测带对象标识，机器人自身位置和邻居位置使用共同坐标系。未知对象数量、多物体数据关联、自定位误差和实机链路不在本轮证据范围。新增权限回归检查确认六个盲起点场景的首帧搜索命令不随未观测货物的位置或形状改变。

## C 背向的一个否定结果

大误差主要在初始化之后形成。本轮仅测试一次先拟合运动再裁剪创新的候选。同输入局部参考读数对照精确复现 {shadow['exact_historical_accepted_readings']} 次；配对位移 MAE 从 {shadow['original_MAE_m']:.6f} 米变为 {shadow['candidate_MAE_m']:.6f} 米，降低 {100*shadow['fractional_reduction']:.2f}%，终点估计误差仍为 {shadow['final_error_m']:.6f} 米。未通过冻结门槛，没有启动候选闭环矩阵或继续调参；该选项默认关闭，主比较也关闭。见[原始分析](reference_negative_analysis.json)和[带符号误差图](reference_negative_result.pdf)。

## 研究贡献与边界

系统贡献是具有明确信息权限的感知、地图、部署、接触推进、估计和停止流程，以及从真实轨迹得到的完整任务证据。边界 CVT 是待独立归因的部署模块；CVT、QP/CBF、ICP、PI 和搜索扫描等标准组成不能单独包装为新算法。当前结果不会通过增加容易形状、剔除失败或延长预算变成“90% 可靠”。

原完成规则仍为 HOLD、真实平移速度 <=0.005 m/s 持续40帧、二维终点误差 <=0.12米；没有新增角速度判据。硬安全采用原间距、穿透预算、求解器检查；G500单列。600秒观察截止不是成功耗时；全部启动任务进入分母。统计单位是条件配对，不是逐帧数据或匹配数。

[图与视频索引](MEDIA.md)记录生成入口、英文图注及身份。完整原始数据在 `runs/research/{MISSION}/`，完整审计在 `artifacts/migrations/{MISSION}/`。代码、结果与论文题目未自动提交或推送。
'''
    if dry_run:
        print(json.dumps(dict(near=totals,far=t,advantage_gate=near['development_advantage_gate'])));return
    DEST.mkdir(exist_ok=True)
    write_new(DEST/'secondary_metrics.json',json.dumps(dict(
        analysis_sha256=sha(research/'near-analysis/analysis.json'),
        interpretation='Descriptive paired deployment diagnostics, primary completion gate unchanged',
        results=secondary),ensure_ascii=False,indent=2))
    for stage in ('near','far'):
        copy_new(research/(stage+'-analysis/all_episodes.csv'),DEST/(stage+'_episodes.csv'))
        copy_new(research/(stage+'-analysis/analysis.json'),DEST/(stage+'_analysis.json'))
    copy_new(research/'shadow/analysis.json',DEST/'reference_negative_analysis.json')
    copy_new(research/'far_safety_failure.json',DEST/'far_safety_failure.json')
    fig_names=['cargo_geometry_atlas','reference_negative_result','near_all_outcomes','near_paired_metrics','far_location_pairs','far_full_process']
    media=['# 图表与真实轨迹视频','',
           '所有图来自冻结配置或已核验的真实记录。下采样、时间对齐、统计单位和主张范围见图注。','',
           '生成入口：`scripts/plot_cargo_discovery.py preflight|near|far`；视频入口：`scripts/render_cargo_discovery.py near|far`。已有产物拒绝覆盖。','',
           '| 科学问题 | 图或视频 | 必需字段 | 统计单位 | 支持与边界 | 建议位置 |',
           '| --- | --- | --- | --- | --- | --- |',
           '| 边界部署表示是否改变最终任务能力？ | near_all_outcomes、near_paired_metrics | 完成、安全、终点误差、物理时间、首次部署边界间隙 | 33 个设计条件的四臂配对 | 固定开发条件下的得失；不是可靠性概率或完整文献基线排名 | 配对指标正文，完整矩阵补充 |',
           '| 未知位置是否额外限制完整任务？ | far_location_pairs、far_full_process、far 视频 | 首次检测、接触、阶段、真实位姿、实际终止 | 6 个物理初态的两臂配对 | 区分发现和搬运；没有任意环境搜索保证 | 正文及视频补充 |',
           '| C 背向的单次估计改动能否解决大误差？ | reference_negative_result | 同输入原读数、候选读数、评价真值和准入 | 1 条历史轨迹，帧是重复测量 | 候选门槛失败；不能从离线读数推出闭环改善 | 局限或补充 |',
           '| 新增货物覆盖了哪些几何？ | cargo_geometry_atlas | 冻结顶点、面积、质量、姿态 | 12 个几何实例、24 个方向条件 | 展示设计范围；轮廓图没有运输成功含义 | 补充 |',
           '| 方法怎样从部署走到实际终止？ | near 同步视频 | 真实 replay、估计读数、阶段和终止状态 | 1 个预先选定条件的四臂 | 说明完整过程及失败，不能代替全部结果表 | 视频补充 |','']
    for name in fig_names:
        info=read(research/'figures'/(name+'.json'))
        for suffix in ('pdf','svg','png','json'):copy_new(research/'figures'/(name+'.'+suffix),DEST/(name+'.'+suffix))
        media.extend([f'## {name}',f'[PDF]({name}.pdf) · [SVG]({name}.svg) · [来源与哈希]({name}.json)','',info['caption'],''])
    for stage in ('near','far'):
        record=research/'media'/(stage+'_media.json')
        if record.exists():
            copy_new(record,DEST/(stage+'_media.json'))
            media.extend([f'## {stage} 同步视频',
                f'[MP4](../../../artifacts/research/{MISSION}/media/{stage}_synchronized.mp4) · [GIF](../../../artifacts/research/{MISSION}/media/{stage}_preview.gif) · [身份]({stage}_media.json)',
                '完整时间范围按共同物理时钟显示，每物理秒取一真实状态、10 fps（约10倍速）；不插值、不补齐轨迹。已结束方法冻结并明确标注终止时刻及结果。估计标注为上一控制步读数；货物轮廓/轨迹是评价真值。选例与结果无关。大视频留本地，不自动加入 Git。',''])
    write_new(DEST/'README.md',text);write_new(DEST/'MEDIA.md','\n'.join(media))
    note_text=f"[不规则货物与远场比较](docs/results/cargo_discovery_v1/README.md)：固定近场132回合（33条件×4方法），安全完成为{note}；远场12回合中{far_note}。两组分别冻结，完整负结果保留；{verdict}\n\n"
    updates={
        'README.md':('## 当前研究：从局部边界部署到实际搬运和停止\n\n',note_text),
        'docs/results/README.md':('# 结果与证据索引\n\n',note_text.replace('docs/results/cargo_discovery_v1/','cargo_discovery_v1/')),
        'docs/DATA_INDEX.md':(None,f'\n## {MISSION}\n\n- `runs/research/{MISSION}/near/`：132个固定近场回合。\n- `runs/research/{MISSION}/far/`：12个未知位置/初始位置提示配对回合。\n- `artifacts/research/{MISSION}/`：C背向离线否定结果、核验分析、矢量图及同步视频。\n- `artifacts/migrations/{MISSION}/`：保护记录、状态、命令、哈希、验证与最终报告。\n- [精简证据](results/cargo_discovery_v1/README.md)：新旧条件分层，失败与G500单列；不代表独立90%可靠性。\n'),
        'experiments/ccta2027/README.md':(None,'\n## 不规则货物与位置权限的分离开发比较\n\n[cargo_near_v1.json](cargo_near_v1.json)冻结33条件×四方法=132回合；[cargo_far_v1.json](cargo_far_v1.json)冻结六物理初态×两位置权限臂=12回合。每例最多600秒、seed 2、同物理/评价；[结果和边界](../../docs/results/cargo_discovery_v1/README.md)保留失败。协议冻结后不得按当前新代码直接重标历史身份；复现需对应执行快照。\n')}
    backup=read(AUDIT/'backup_manifest.json')
    for name,(anchor,addition) in updates.items():
        path=ROOT/name;old=path.read_text(encoding='utf-8')
        if addition in old:continue
        assert sha(path)==backup[name], 'User or external changes must be preserved: '+name
        new=old+addition if anchor is None else old.replace(anchor,anchor+addition,1)
        assert new!=old
        path.write_text(new,encoding='utf-8')
    dump(AUDIT/'publication_receipt.json',dict(outputs={p.relative_to(ROOT).as_posix():sha(p) for p in DEST.iterdir() if p.is_file()},
        source=sha(Path(__file__)),near=totals,far=t,git_and_manuscript_preserved=True))
    print(json.dumps(dict(destination=str(DEST),near=totals['all'],far=t,gate=near['development_advantage_gate'])))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--dry-run',action='store_true');publish(p.parse_args().dry_run)
