"""Publish small verified derivatives of the completed frozen handoff batches.

Never edits raw receipts, overwrites a conflicting publication, or uploads files.
"""
from __future__ import annotations

from collections import Counter
import json
from pathlib import Path

from dbact_sim.experiments.deployment import dump
from freeze_contact_handoff_study import ROOT, MISSION, read, sha


def main():
    audit=ROOT/f'artifacts/migrations/{MISSION}'
    artifacts=ROOT/f'artifacts/research/{MISSION}'
    public=ROOT/'docs/results/contact_handoff_v1'
    if not public.resolve().is_relative_to(ROOT.resolve()):
        raise ValueError('Publication path escapes repository')
    reports={stage:read(artifacts/stage/'analysis.json') for stage in ('ablation','main','extended')}
    for stage,r in reports.items():
        if r['source_integrity']!='PASS' or r['historical_integrity']!='PASS' or not r['compatibility_pass']:
            raise RuntimeError(f'{stage} verification failed; do not publish as verified')
    public.mkdir(parents=True,exist_ok=True)
    copies=[]
    names=['all_episodes.csv','capability_table.md','analysis.json','provenance.json',
           'endpoint_comparison.pdf','endpoint_comparison.svg','endpoint_comparison.png',
           'complete_trajectories.pdf','complete_trajectories.svg','complete_trajectories.png']
    for stage in reports:
        for name in names:
            source=artifacts/stage/name
            target=public/stage/name
            digest=sha(source)
            if target.exists() and sha(target)!=digest: raise FileExistsError(target)
            target.parent.mkdir(parents=True,exist_ok=True)
            if not target.exists(): target.write_bytes(source.read_bytes())
            assert sha(target)==digest
            copies.append(dict(source=source.relative_to(ROOT).as_posix(),target=target.relative_to(ROOT).as_posix(),sha256=digest))
    # Arrival may be admitted in the same control step as acceptance because a
    # volunteer is already at the proposed patch. Do not report this as travel.
    arrival_states=Counter()
    for case in (ROOT/f'runs/research/{MISSION}/main').glob('*_handoff/paired_metrics.json'):
        steps=read(case)['steps'];previous={}
        for step in steps:
            for d in step['diagnostics']:
                if d['handoff_event']=='arrived_not_confirmed':
                    arrival_states[previous.get(d['agent_id'],'initial')]+=1
                previous[d['agent_id']]=d['handoff_state']
    stages=reports['main']['events']
    events=Counter()
    for name,r in stages.items():
        if name.endswith('_handoff'): events.update(r['handoff_events'])
    main_counts=reports['main']['counts'];ext_counts=reports['extended']['counts']
    lines=[
        '# 确认与回退接触交接 v1：固定预算结果', '',
        '三个批次分别冻结：12 回合压力/换位消融、27 回合 150 秒主比较，以及用户指定的 18 回合最多 600 秒观察。',
        '同一开发 seed 2；原始初态、感知、物理、安全约束、阶段/停稳和二维完成判据保留。全部失败和执行身份保留。',
        '这不是独立泛化验证，也不是硬件实验。', '',
        '| 比较 | 原 CVT | spacing | 交接候选 |', '|---|---:|---:|---:|',
        f"| 150 秒任务完成 | {main_counts['legacy']['completed']}/9 | {main_counts['spacing']['completed']}/9 | {main_counts['handoff']['completed']}/9 |",
        f"| 150 秒 G500 | {main_counts['legacy']['g500']}/9 | {main_counts['spacing']['g500']}/9 | {main_counts['handoff']['g500']}/9 |",
        f"| 600 秒观察任务完成 | {ext_counts['legacy']['completed']}/9 | 未在本批执行 | {ext_counts['handoff']['completed']}/9 |", '',
        '600 秒只是独立观察预算：已经按原规则进入 HOLD 并停稳的回合在原时刻结束；不会用真值重启。',
        '尚未完成且到达 600 秒的回合记为右删失，不能推断永远不可能完成。', '',
        '## 本轮结论', '',
        '压力分配单独启用即可重现矩形成功丢失：反馈修复/仅换位分别完成 1/3，归一化分配/联合版本分别 0/3。',
        '这支持保留原有推压能力，不支持把上一版退化全部归因于换位。消融中的反馈修复开关与主比较的默认控制明确分开。', '',
        f"主比较记录 {events['proposed']} 次提议、{events['accepted_keep_pressing']} 次接纳、{events['arrived_not_confirmed']} 次位置到达判定，",
        f"但持续局部确认 {events['locally_confirmed']} 次，试探卸载 {events['trial_unload']} 次，原推手退出 {events['release_complete']} 次。",
        f"到达判定的前一状态计数为 `{dict(arrival_states)}`：这些都是已在目标邻域的接纳，不能当作完成侧向换位。",
        '因此个别终点误差下降不能归因于成功交接。候选没有通过事前完成能力门槛，默认开关仍关闭。', '',
        '局部间隙、推压命令和运动响应仅构成操作性代理判据，既非力传感器确认，也不能辨识替补的独立贡献。',
        '当前候选还要求单个替补不产生反向纵向分量，这比无附着物理更严格；其拒绝不证明对应形状不可搬运。', '',
        '## 可追溯结果', '',
        '- [消融全表](ablation/capability_table.md) · [CSV](ablation/all_episodes.csv)。',
        '- [150 秒九条件全表](main/capability_table.md) · [CSV](main/all_episodes.csv)。',
        '- [600 秒观察全表](extended/capability_table.md) · [CSV](extended/all_episodes.csv)。',
        '- [主比较终点图 PDF](main/endpoint_comparison.pdf) · [全轨迹图 PDF](main/complete_trajectories.pdf)。',
        '- [研究判断、文献对照与标题选项](../../CONTACT_HANDOFF_RESEARCH_2026-11-01.md)。',
        '- [图表含义、英文图注与本地视频](MEDIA.md)。', '',
        '六个消融参考、十八个主比较对照的回放/原日志字段精确一致；十八个延长观察的 150 秒前缀检查通过。',
        '记录中的硬安全异常与 barrier scaling 分别报告，未用测试通过代替动态安全证明。', '',
        '## 复现与身份', '',
        f'完整原始数据位于本地 `runs/research/{MISSION}/ablation|main|extended/`，每回合有独立收据和产品哈希。',
        f'规范派生结果位于 `artifacts/research/{MISSION}/`。本目录是经 SHA-256 核对的小型公开证据副本。',
        '完整视频、逐帧数据和环境不随 Git 克隆提供；不自动提交或上传。', '',
        '```powershell',
        '.\\.venv\\Scripts\\python.exe scripts/analyze_contact_handoff_study.py ablation',
        '.\\.venv\\Scripts\\python.exe scripts/analyze_contact_handoff_study.py main',
        '.\\.venv\\Scripts\\python.exe scripts/analyze_contact_handoff_study.py extended',
        '```', '',
        '重新执行必须使用相应冻结源码/环境和新输出目录；同身份中断使用原目录 `--resume`。',
        '不能把当前 checkout 的提交号当作脏工作树实验源码身份，也不能修改旧协议后继续同一收据。',
    ]
    def write_same_or_new(path,text):
        if path.exists() and path.read_text(encoding='utf-8')!=text: raise FileExistsError(path)
        if not path.exists(): path.write_text(text,encoding='utf-8')
    write_same_or_new(public/'README.md','\n'.join(lines)+'\n')
    media=f'''# 图表、视频与主张边界

| 科学问题 | 产品与字段 | 统计单位 | 可支持与不可支持 |
|---|---|---|---|
| 压力分配还是换位导致退化？ | 消融全表、终点图；四个机制开关、完成、误差 | 3 个配对条件，一个 seed | 矩形反例与机制隔离；不支持跨种子可靠性 |
| 是否救回完整任务失败？ | 主比较终点图、全轨迹；全部 27 回合 | 9 个配对条件，一个 seed | 描述完成与配对变化；不以帧数作样本，不做显著性/等价结论 |
| 更多时间是否足够？ | 延长观察全表及轨迹；18 回合，600 s | 9 个配对条件 | 有限观察结果；删失不等于永远失败，HOLD 不重启 |
| 是否真的发生接管？ | `main/analysis.json` 的事件及独立接触核验 | 机器人事件，非独立实验 | 本轮没有通过确认/卸载；不能把位置到达当作接触能力 |
| 完整过程如何发生？ | 三方法同步视频、真实物体轨迹和已保存地图 | 两个事后说明案例 | 展示成功与失败，不替代全表，不声称这些是独立验证 |

English caption — endpoint comparison:
“Final two-dimensional endpoint errors for every frozen development condition.
Filled markers denote completion under the original settling and 0.12 m endpoint
criterion; open markers denote noncompletion at the observed end. Conditions are
paired within one seed-2 block. No confidence intervals or independent-sample
inference are asserted. G500 is reported separately.”

English caption — complete trajectories:
“Unsmoothed evaluation trajectories from start to actual termination for all
conditions. Crosses mark observed endpoints; dashed circles show the unchanged
target tolerance. Axes use metric coordinates with equal aspect within each panel;
panel ranges differ. These are object-truth evaluation paths, not information
available to local controllers.”

English caption — extended observation:
“Matched original and candidate episodes under a separate 600 s physical-time
observation budget. Original termination rules are retained. Unfinished episodes
at the budget are right-censored; previously terminated off-target HOLD episodes
are not restarted or imputed as later successes.”

English caption — synchronized videos:
“Post-hoc circle-failure and rectangle-parallel retained-completion illustrations,
shown on the same physical timeline, camera and scale. Ended episodes retain their
actual final frame with a terminal label. Orange geometry is evaluation truth;
teal points show agent 00's saved fused map, not raw laser rays. Geometric readiness
rings are not contact certificates. The candidate did not achieve a confirmed
handoff in these experiments.”

本地媒体目录：`artifacts/research/{MISSION}/media/`。

- `circle.mp4`、`circle.gif`：三方法圆形失败，完整起止过程。
- `rectangle_parallel.mp4`、`rectangle_parallel.gif`：三方法矩形保留完成，完整起止过程。
- `receipts.json`、`manifest.json`：输入与输出哈希、实际渲染吞吐和生成版本。

控制频率 20 Hz；视频采样间隔 0.25 s、播放 10 fps，名义 2.5 倍速，末尾补实际最后一帧。
不插值、不按数组比例强行同步；视频 HUD 区分物理时间和前一控制时刻的团队估计。
GIF 为 4 fps、720 像素宽预览。所有定量曲线不平滑、保留全部样本；视频下采样不替代原始数据。
PDF/SVG 为矢量，PNG 仅作预览。图—数据—分析版本映射见各阶段 `provenance.json` 与本目录清单。
'''
    write_same_or_new(public/'MEDIA.md',media)
    publication=dict(mission=MISSION,verified_copies=copies,
        execution_source_sha256={stage:read(ROOT/f'experiments/ccta2027/contact_handoff_v1_{stage}.json')['frozen_source_sha256'] for stage in reports},
        generator_sha256=sha(Path(__file__)), arrival_previous_states=dict(arrival_states))
    if (public/'manifest.json').exists():
        previous=read(public/'manifest.json')
        if 'figure_style_revision' in previous:
            for item in previous['figure_style_revision']['revisions']:
                assert sha(ROOT/item['path'])==item['after_sha256']
            publication['figure_style_revision']=previous['figure_style_revision']
    dump(public/'manifest.json',publication)
    with (audit/'operations.jsonl').open('a',encoding='utf-8') as f:
        for item in copies: f.write(json.dumps({'operation':'verified_public_derivative',**item})+'\n')
    print(json.dumps({'public':str(public.relative_to(ROOT)),'copies_verified':len(copies),'main':main_counts,'extended':ext_counts},indent=2))


if __name__=='__main__':
    main()
