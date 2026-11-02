"""Publish verified compact evidence; preserve raw data and prior documents."""
import argparse,json,os,shutil,sys
from pathlib import Path
from audit_contact_consistency import ROOT,MISSION,read,sha

AUDIT=ROOT/'artifacts/migrations'/MISSION
DEST=ROOT/'docs/results/contact_consistency_v1'
NAMES={'circle':'圆形','ellipse_parallel':'椭圆平行','ellipse_perpendicular':'椭圆垂直',
       'rectangle_parallel':'矩形平行','rectangle_perpendicular':'矩形垂直',
       'l_reference':'L 参考姿态','l_rotated':'L 旋转 180 度',
       'c_open_toward':'C 开口朝向','c_open_away':'C 开口背向'}

def write_new(path,text):
    data=text.encode('utf-8')
    if path.exists():
        if path.read_bytes()!=data:raise RuntimeError('Conflicting publication: '+str(path))
        return
    with path.open('xb') as f:f.write(data)

def publish(analysis,dry_run):
    assert Path.cwd().resolve()==ROOT and Path(sys.prefix).resolve()==ROOT/'.venv'
    analysis=analysis.resolve()
    assert analysis.is_relative_to(ROOT/'artifacts/research'/MISSION)
    a=read(analysis/'analysis.json');old=a['totals']['legacy_aggregate'];new=a['totals']['consistent_aggregate']
    assert len(a['episodes'])==18 and not old['execution_errors'] and not new['execution_errors']
    assert (ROOT/'runs/maintenance'/MISSION/'study.exit.txt').read_text(encoding='utf-8-sig').strip()=='0'
    assert read(AUDIT/'figure_validation.json')['status']=='PASS'
    assert sha(ROOT/'experiments/ccta2027/contact_consistency_v1.json')==a['protocol_sha256']
    repeat=[v for v in a['verification'].values() if 'historical_replay_exact' in v]
    assert len(repeat)==3 and all(v['historical_replay_exact'] and v['historical_fields_exact'] for v in repeat)
    note=(f"固定 18 回合，九个开发条件、同一 seed 2、每例最多 600 秒。"
          f"共同参考配置下，旧聚合公式完成 {old['completed']}/9，坐标一致修正完成 {new['completed']}/9；"
          f"G500 分别 {old['g500']}/9、{new['g500']}/9，修正臂原硬安全检查违规 {new['hard_safety_cases']}/9。"
          "C 背向仍失败且出现越过目标后的错误停止；8/9 不是总体成功概率。"
          "本轮是公共安全计算修复的配对比较，不是原 CVT 对 spacing 的优越性检验。")
    table=['| 条件 | 旧公式完成 | 修正完成 | 旧终点误差 m | 修正终点误差 m | 旧/新 G500 |',
           '| --- | --- | --- | ---: | ---: | --- |']
    for r in a['paired']:
        table.append(f"| {NAMES[r['condition']]} | {'是' if r['legacy_complete'] else '否'} | "
                     f"{'是' if r['corrected_complete'] else '否'} | {r['legacy_endpoint_m']:.6f} | "
                     f"{r['corrected_endpoint_m']:.6f} | {int(r['legacy_g500'])}/{int(r['corrected_g500'])} |")
    regress=[NAMES[r['condition']] for r in a['paired'] if r['endpoint_change_m']>0]
    summary=f"""# 坐标一致的安全平面：固定开发比较

{note}

身份为 `{MISSION}`。[预先设计](../../CONTACT_CONSISTENCY_RESEARCH_2026-11-02.md)与[冻结协议](../../../experiments/ccta2027/contact_consistency_v1.json)保持原字节。全部启动回合进入分母；无追加版本、换种子、延时或阈值调整。

## 全部条件

{chr(10).join(table)}

完整字段见[全部 18 回合](all_18_episodes.csv)、[条件配对差](paired_comparison.csv)和[机器可读分析](analysis.json)。端点误差退化的条件为：{'、'.join(regress)}，即使任务仍完成也保留。救回条件为：{'、'.join(NAMES[x] for x in a['rescued'])}；失去的完成为：{'、'.join(NAMES[x] for x in a['lost']) or '无'}。预先任务门槛：{'PASS' if a['predeclared_task_gate'] else 'FAIL'}。

## 比较究竟改变了什么

两臂均使用 boundary_cvt、已有固定观测参考的位置/速度双通道、径向参考点与自身扫描接触距离拟合；仅 `aggregate_frame_consistent` 开关不同。两臂的初态、观测权限、压力控制、物理、安全阈值与成功判据相同。旧臂不能改称原始 CVT 或 spacing，也不能把此前成功拼接为本轮结果。

旧公式将平均法向单位化，却用另一分母平均平面偏置；同一局部几何平移后安全余量会变化。新公式用 `sum(w*n@(p-b))/norm(sum(w*n))-r_safe`，恢复坐标平移一致性。具体推导见预先设计。它修复确定的代数错误，不证明聚合平面等于真实曲面，也不构成动态接触安全定理。

本轮仍无附着、无控制真值，保留自身/一跳观测和既有全队监督；不能称为完全去中心化闭环。新开关默认关闭以保留冻结执行兼容，本协议显式开启修正臂。若后续比较部署算法，应给各方法同样的公共修复。

## 成功、安全与失败的边界

原任务完成要求 HOLD 后速度不超过 0.005 m/s 持续 40 帧，且最终二维误差不超过 0.12 m；dt=0.05 s。没有新增角速度收定判据。600 秒未结束为观察截止，不作为成功耗时。

硬检查沿用间距、物体内部侵入、原离散穿透预算和求解失败检查；零违例不是形式安全保证。C 朝向修正臂最大穿透约 0.06524 m，略高于名义连续量 0.065 m，但在原离散预算内，不隐去此差别。屏障缩放另列，未当作零风险。

修正矩形平行任务完成，但 G500 因整段没有记录到 TRANSPORT 激活而失败。C 背向终点误差约 0.795 m、三次屏障缩放，任务与 G500 都失败。几何就绪、阶段标签、实际接触、收定、二维完成和 G500 分别保留。

## 有输入身份的机制证据

两例旧日志分别精确核对 12000 个地图快照与末 200 帧共 3200 条机器人命令。详细同输入平面检查仅覆盖末 10 秒的名义推手；C 背向 211 条、圆形 1272 条相关的机器人/帧记录不是独立试验。平面距离平均绝对误差分别从 148.3 降至 16.9 mm、63.0 降至 10.4 mm；修正后最大误差仍约 108.5、85.5 mm。反事实命令不是实际物理轨迹；见[same_input_screen.json](same_input_screen.json)。

三个历史 combined 对照的全部 25 个 replay 数组及旧逐帧科学字段精确复现。其余六条件是本轮共同配置下的新对照。源码快照与每例产品哈希全部核验，未把工程重复当作独立样本。

## 剩余 C 背向问题

修正后的物体实际前进约 1.995 m，目标 1.2 m；44.5 s 首次 HOLD 时控制估计仍约 1.194 m。三位被接受的局部读出约为 1.008、1.194、2.022 m，匹配残差均很小。当前团队监督器缺少对这些分歧的停止资格检查。完整时间对齐见[c_away_stop_audit.json](c_away_stop_audit.json)。这直接表明错误停止与分歧并存，但不能凭此断定一个投票阈值就能修好闭环。

峰值真速度约 0.380 m/s，84 帧超过配准配置的 0.25 m/s 边界；这是后验诊断，不是因果证明。本轮不提高速度上限、不扩大终点容忍量、不改变停止定义。[研究决定](DECISION.md)说明下一项最小核验。

## 图表、复现与原件

[完整轨迹](complete_trajectories.pdf)、[全部任务比较](paired_task_results.pdf)、[同输入平面误差](same_input_plane_error.pdf)，同时提供 SVG/PNG。[图表问题、英文图注与边界](FIGURES.md)。这些图来自真实保存数据，没有合成轨迹、平滑或失败剔除。

原始批次在 `runs/research/{MISSION}/run/`，含执行源码、协议、逐帧记录和独立评价；离线诊断/分析在 `artifacts/research/{MISSION}/`；恢复与本地审计在 `artifacts/migrations/{MISSION}/`；测试在 `runs/maintenance/{MISSION}/`。原始数据与同盘备份不等于 GitHub 备份。论文题目和稿件本轮未改动。

只读协议核验：`.\\.venv\\Scripts\\python.exe -m dbact_sim study --protocol experiments/ccta2027/contact_consistency_v1.json --split development --dry-run`。离线再生成使用 `scripts/analyze_contact_consistency.py --out artifacts/research/{MISSION}/analysis-02`（须为不存在的新目录），不启动仿真。执行入口 `scripts/run_contact_consistency.py` 遇已有批次会拒绝覆盖；本轮不再次执行。

GitHub 16 个标签的对象身份均已核对；其中选读的 10 个标签安全聚合实现含同类混合归一化。范围与历史报告限制见[TAG_REVIEW.md](TAG_REVIEW.md)。
"""
    decision="""# 本轮研究决定

确定问题：安全聚合平面的法向与偏置采用不同归一化，导致坐标原点依赖。修复通过代数/命令回归与冻结对照；本轮预定任务门槛已通过。它是公共实现正确性修复，不能包装成 CVT 相对 spacing 的新算法优势。

保留：原始全部失败、旧公式兼容分支、固定观测参考与其失败边界、完整执行身份。停止：当前预算内继续加种子或调参、继续把圆/椭圆停滞全归于接触布局、用旧标签的静态保证支撑动态运输。

下一项主线应是局部参考估计的一致性与停止资格。C 背向不再仅表现为接触不足，而是越过目标后错误估计触发 HOLD。先用本次已有日志做一次同输入审查：核对时间对齐、参考锚点、匹配切换、被拒绝样本与投票者变化。将局部小残差但不同全局位移的时段单列；检查小转角、速度边界和观测覆盖的替代解释。离线剔除坏票即使有效，也不能单独证明闭环有效。

若找到确定实现错误，单独修复并冻结新基线；若主要是数据关联歧义，候选应输出可信/不确定状态，并让不确定停止进入可回退的减速与重新观测，而非把“离目标近”直接锁存为完成。置信不足时一味刹停也可能永久停滞，所以必须同时评价可恢复性。禁止从真值选票、用真实距离设门、无限等待或放宽成功/安全阈值。

候选必须先明确新增信息、算力、动作与时间预算，再设计同初态配对；当前不写入新控制器、不启动第二轮。下一次比较要保留本轮三个救回条件与 C 反例，并检查已成功条件的倒退。无需再同时叠加压力、增援、换位和估计器。

备用主线是必要的接触重配置，但只有在估计可信、当前接触无法产生所需纠偏运动的证据成立后才启动；保留有效推手、确认替补接管的原则仍可复用。仅靠调整压入量不能改变单边接触的可达力方向；本轮没有证明一种新的重部署算法。

“任意形状”需要限制尺寸、质量、摩擦、可接近边界和机器人资源；无限制的所有形状/载荷不是当前有限资源推物系统可承诺的集合。“成功概率超过 90%”必须先固定场景分布，再冻结方法、用未消费的独立样本验证。9 个反复使用的开发条件即使 9/9 也不能直接支持该概率；跨形状同一 seed 的相关性应保留。不要把相关帧、配准次数或多轮调参作为独立样本。

论文可以报告完整任务能力及失败边界，但目前不能声称任意形状完成、总体超过 90%、完全去中心化或相对已有方法普遍优越。论文题目保持原样。修复共用基础设施后，再以同权限的 CVT/spacing 比较边界部署的具体贡献。
"""
    figures="""# 图、数据与主张

| 科学问题 | 图 | 字段与统计单位 | 支持与边界 | 位置 |
| --- | --- | --- | --- | --- |
| 修复是否改变实际物体路径？ | complete_trajectories | 保存的 cargo center、start、goal；9 条件配对 | 展示全部 18 条路径与失败；不是全形状可靠性或机器人部署视频 | 正文候选 |
| 是否救回任务、是否有退化？ | paired_task_results | 完成/误差、运行时长、屏障缩放；9 条件配对 | 全部启动任务；失败运行时长不当作完成时长 | 正文候选 |
| 给定同一输入，平面计算误差是否减少？ | same_input_plane_error | 独立几何距离与旧/新聚合平面；两例末10秒的相关机器人/帧 | 同输入反事实计算，不能替代真实闭环或独立统计验证 | 补充/机制图 |

所有图源数据为 analysis.json、all_18_episodes.csv、paired_comparison.csv、same_input_screen.json 和本地 plot_data.json；逐项输入 SHA-256 与分析代码 SHA-256 在 analysis.json。无平滑、插值或外推，无置信区间（没有适合可靠性推断的独立样本）。蓝色虚线/方形为旧公式，橙色实线/圆形为修正；灰色圈/线为原目标容差。图宽 7.15 inch，矢量 PDF/SVG 与 220 dpi PNG，已核验嵌入字体和渲染。

**Trajectory caption.** Complete recorded object-center paths for nine paired development conditions (seed 2). Both arms share sensing, geometry readouts, physical parameters and evaluation; only the safety-plane normalization differs. Crosses mark terminal positions and the circle denotes the unchanged 0.12 m goal tolerance. Failed trials are retained. These reused conditions do not estimate population reliability.

**Task caption.** Paired endpoint error, observed duration and barrier-scaling events for all eighteen started episodes. F denotes task failure; a cross in the duration panel denotes an unfinished or off-target trial, whose duration is not a completion time. G500 and hard safety checks are reported separately in the accompanying table; they are not interchangeable with task completion.

**Mechanism caption.** Mean absolute error of the aggregate plane gap relative to the independent geometric gap under identical saved inputs. The audit covers nominal pushers during the final ten seconds of two historical failures (211 and 1272 repeated agent/frame records). These correlated records are not independent experiments. Corrected maximum errors remain 108.5 and 85.5 mm; the calculation is not a geometric safety certificate.

生成入口为 `scripts/analyze_contact_consistency.py`。图表不声称展示接触力时序、逐机器人边界观测或完整过程视频；原始轨迹仍可供后续按统一物理时间离线渲染。
"""
    tags="""# 冻结标签审查范围

通过 GitHub matching-refs/tags API 取得 16 个标签，与本地标签对象逐一比较，全部一致；没有移动标签、更新分支、push 或重写历史。tag_identity.json 记录远端对象与本地 peel 后的 commit。tag_code_review.json 记录所选安全源码的路径、哈希和公式位置；其中 10 个标签有同类混合归一化。没有据此声称每个标签的所有文件或所有实验均已重新验证。

直接检查的历史证据：

| 标签 / commit | 原始文件与发现 | 可用范围 |
| --- | --- | --- |
| archive/2026-10-23/DBACT-research-v3 / a3d5e693d40d541b90d69606624c607223d2e491 | docs/results/v2_shape_matrix/REPORT.txt：180 回合、旧定义成功 54/180 | 12 形状、5 seeds、3 距离；不可合并当前二维完成定义 |
| archive/pre-integration-2026-09-01/A-boundary-aware-closed-loop-v3 / c038829487ad7b6a15046dc79b9dfc95b3bfe83c | docs/results/ARBITRARY_SHAPE_MONTE_CARLO_FINAL_SE2.md：60 未筛选回合、14/60 达到当时合同 | 0.10 m 激活后距离、1500 帧；不是当前 1.2 m 任务 |
| archive/2026-10-23/research-simulation-evidence-closure / 15d62f2e30063c3258ae49ea00b5dedbc0228d8a | README.md 区分静态部署、定理和独立仿真证据 | 静态结论不能当作动态接触运输保证 |

上述已读报告没有建立当前“任意形状或总体成功概率超过 90%”的证据。本地尚未推送的研究按源码快照与收据识别，不能因为 GitHub 缺少它们而视为未完成。
"""
    files={name:analysis/name for name in ['analysis.json','all_18_episodes.csv','paired_comparison.csv','c_away_stop_audit.json']}
    files['same_input_screen.json']=ROOT/'artifacts/research'/MISSION/'screen.json'
    for name in ('tag_identity.json','tag_code_review.json'):files[name]=AUDIT/name
    for stem in ('complete_trajectories','paired_task_results','same_input_plane_error'):
        for ext in ('pdf','svg','png'):files[f'{stem}.{ext}']=analysis/f'{stem}.{ext}'
    texts={'README.md':summary,'DECISION.md':decision,'FIGURES.md':figures,'TAG_REVIEW.md':tags}
    additions={
        'README.md':('## 当前研究：从局部边界部署到实际搬运和停止\n\n',f'[最新坐标一致修复](docs/results/contact_consistency_v1/README.md)：{note}\n\n'),
        'docs/results/README.md':('# 结果与证据索引\n\n',f'[坐标一致修复固定比较](contact_consistency_v1/README.md)：{note}\n\n'),
        'docs/DATA_INDEX.md':('# 数据与原始文件索引\n\n',f'坐标一致修复批次原始 18 回合在 `runs/research/{MISSION}/run/`，执行源码与原始失败完整保留；分析/审计/工程测试分别在同身份 `artifacts/research/`、`artifacts/migrations/`、`runs/maintenance/`。见[结果与身份](results/contact_consistency_v1/README.md)。{note}\n\n'),
        'experiments/ccta2027/README.md':('# CCTA 2027 实验入口\n\n',f'## 坐标一致修复固定比较\n\n[contact_consistency_v1.json](contact_consistency_v1.json) 已冻结执行。{note} [完整结果](../../docs/results/contact_consistency_v1/README.md)。旧公式兼容保留，新开关默认关闭，本协议显式开启；不重新冻结或追加实验。\n\n')}
    previous=read(AUDIT/'backup_manifest.json');edits={}
    for name,(anchor,addition) in additions.items():
        path=ROOT/name;before=path.read_text(encoding='utf-8')
        if addition in before:continue
        assert sha(path)==previous[name],'Document changed since backup: '+name
        assert before.count(anchor)==1,name
        edits[name]=before.replace(anchor,anchor+addition,1)
    if dry_run:
        print(json.dumps(dict(copy=list(files),new_docs=list(texts),indices=list(edits)),ensure_ascii=False));return
    DEST.mkdir(parents=True,exist_ok=True)
    for name,source in files.items():
        dest=DEST/name;expected=sha(source)
        if dest.exists():assert sha(dest)==expected,name
        else:
            with source.open('rb') as source_stream,dest.open('xb') as target:shutil.copyfileobj(source_stream,target)
        assert sha(dest)==expected and sha(source)==expected,name
    for name,body in texts.items():write_new(DEST/name,body)
    for name,body in edits.items():
        path=ROOT/name;assert sha(path)==previous[name],name
        temp=path.with_name(path.name+'.contact-consistency.tmp')
        with temp.open('x',encoding='utf-8',newline='\n') as f:f.write(body)
        os.replace(temp,path)
    manifest=dict(mission=MISSION,analysis=analysis.relative_to(ROOT).as_posix(),
        analysis_script_sha256=sha(ROOT/'scripts/analyze_contact_consistency.py'),
        publication_script_sha256=sha(Path(__file__)),
        files={name:sha(DEST/name) for name in sorted(set(files)|set(texts))},
        updated_indices={name:sha(ROOT/name) for name in additions},
        raw_data_unchanged=True,manuscript_updated=False)
    write_new(DEST/'manifest.json',json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    with (AUDIT/'operations.jsonl').open('a',encoding='utf-8') as f:
        f.write(json.dumps(dict(operation='publish_verified_evidence',manifest=manifest),ensure_ascii=False)+'\n')
    print(json.dumps(dict(published=str(DEST),files=len(manifest['files'])),ensure_ascii=False))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--analysis',required=True,type=Path)
    p.add_argument('--dry-run',action='store_true');args=p.parse_args();publish(args.analysis,args.dry_run)
