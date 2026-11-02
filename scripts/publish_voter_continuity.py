"""Figures and an evidence-bounded report from the frozen four-arm comparison."""
import csv
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import voter_continuity_study as study
from dbact_sim.experiments.deployment import dump

LABELS={'reference':'Original','continuity':'Continuity','confirmation':'Confirm 1 s','combined':'Combined'}
COLORS={'reference':'#222222','continuity':'#0072B2','confirmation':'#D55E00','combined':'#009E73'}
LINES={'reference':'-','continuity':'--','confirmation':'-.','combined':':'}
NAMES=['Circle','Ellipse parallel','Ellipse perpendicular','Rectangle parallel','L reference',
       'C away (near)','C away (far)','Skew box (far)']
CAPTIONS={}


def save(fig,name,caption,inputs):
    size=fig.get_size_inches().tolist()
    fig.savefig(study.PUBLIC/(name+'.pdf'))
    fig.savefig(study.PUBLIC/(name+'.svg'))
    fig.savefig(study.PUBLIC/(name+'.png'),dpi=220)
    plt.close(fig)
    CAPTIONS[name]=dict(caption=caption,inputs=inputs,size_inches=size,
        files={suffix:study.sha(study.PUBLIC/(name+'.'+suffix)) for suffix in ('pdf','svg','png')})


def table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','| '+' | '.join(['---']*len(headers))+' |']+
                     ['| '+' | '.join(str(v).replace('|',' / ') for v in row)+' |' for row in rows])


def main():
    output=study.PUBLIC/'README.md'
    if output.exists(): raise RuntimeError('Preserve published report')
    a=study.read(study.PUBLIC/'analysis.json'); rows=a['episodes']; arms=list(study.ARMS)
    plt.rcParams.update({'font.size':8,'axes.labelsize':8,'axes.titlesize':9,'pdf.fonttype':42,
                         'ps.fonttype':42,'svg.fonttype':'none','savefig.facecolor':'white'})
    matrix=np.zeros((8,4));texts=[]
    for i,c in enumerate(study.CONDITIONS):
        line=[]
        for j,arm in enumerate(arms):
            r=next(r for r in rows if r['condition']==c and r['arm']==arm)
            matrix[i,j]=1 if r['safe_completed'] else 0
            label='PASS' if r['safe_completed'] else 'FAIL' if r.get('settled') else 'CUTOFF'
            line.append(label+' / '+format(r.get('endpoint_error_m',float('nan')),'.3f')+' m\n'+
                        format(r.get('seconds',0),'.1f')+' s / G'+str(int(r['g500'])))
        texts.append(line)
    fig,ax=plt.subplots(figsize=(7.2,4.5),layout='constrained')
    ax.imshow(matrix,cmap=ListedColormap(['#F4D9C6','#D4E8E2']),vmin=0,vmax=1,aspect='auto')
    ax.set_xticks(range(4),[LABELS[k] for k in arms]);ax.set_yticks(range(8),NAMES)
    for i in range(8):
        for j in range(4): ax.text(j,i,texts[i][j],ha='center',va='center',fontsize=8)
    ax.set_title('All 32 development episodes: safe completion / endpoint error\nElapsed physical time / original G500 (1 = pass)')
    save(fig,'all_episodes','All eight previously consumed development conditions and four paired arms. Cells report safe task completion, Euclidean endpoint error, elapsed physical duration and the unchanged G500 verdict. A cutoff is not a completion time; the maximum observation is 600 s. These are selected conditions, not independent reliability trials.',{'analysis.json':study.sha(study.PUBLIC/'analysis.json')})

    fig,axes=plt.subplots(2,2,figsize=(7.2,4.7),layout='constrained')
    time_inputs={}
    for col,condition in enumerate(('far_skew_box_s0_r0_pos0','far_c_open_away_pos0')):
        for arm in arms:
            name=condition+'_'+arm
            p=study.OUT/'timeseries'/(name+'.csv');time_inputs[str(p.relative_to(study.ROOT))]=study.sha(p)
            with p.open(encoding='utf-8') as f: series=list(csv.DictReader(f))
            t=np.array([float(r['time_s']) for r in series])
            for row,key in enumerate(('true_goal_error_m','signed_error_x')):
                y=np.array([float(r[key]) for r in series])
                axes[row,col].plot(t,y,color=COLORS[arm],ls=LINES[arm],lw=1,label=LABELS[arm])
                hold=a['diagnostics'][name]['first_hold']
                if hold:
                    k=hold['control_frame'];axes[row,col].plot(t[k],y[k],'o',color=COLORS[arm],ms=4)
            axes[0,col].set_title('Far skew box' if col==0 else 'Far C, opening away')
        axes[0,col].axhline(.12,color='#777777',ls=':',lw=.8)
        axes[1,col].axhline(0,color='#777777',lw=.6)
        for row in range(2):
            axes[row,col].set_xlabel('Physical time (s)');axes[row,col].grid(alpha=.2)
    axes[0,0].set_ylabel('True goal error (m)')
    axes[1,0].set_ylabel('Signed x estimate error (m)')
    axes[0,0].legend(fontsize=7)
    save(fig,'stop_and_bias','Control-time aligned trajectories of the two preselected far-field cases, with all recorded samples and no smoothing. Top: true goal error (the dotted line is the original 0.12 m task tolerance). Bottom: signed controller displacement error along x. Circles indicate first HOLD; traces end at their actual termination. Different arms follow different trajectories, so these curves are not a same-input estimator comparison.',time_inputs)

    prior=study.read(study.AUDIT/'prior_log_audit.json')
    fig,axes=plt.subplots(2,1,figsize=(7.2,4.7),layout='constrained')
    for dim,ax in enumerate(axes):
        for j,(key,color,label) in enumerate([('initial_error_m','#777777','Initial bias'),('roster_sum_m','#0072B2','Roster contribution'),('common_error_sum_m','#D55E00','Common-readout error')]):
            vals=[]
            for c in prior['cases']:
                assert len(c['segments'])==1
                vals.append(c['segments'][0][key][dim]*1000)
            ax.bar(np.arange(8)+(j-1)*.22,vals,width=.21,color=color,label=label)
        total=[c['segments'][0]['final_error_m'][dim]*1000 for c in prior['cases']]
        ax.scatter(range(8),total,color='black',marker='D',s=16,label='Error at HOLD',zorder=3)
        ax.axhline(0,color='#666666',lw=.7);ax.set_ylabel(('x' if dim==0 else 'y')+' error (mm)')
        ax.set_xticks(range(8),NAMES,rotation=25,ha='right');ax.grid(axis='y',alpha=.2)
    axes[0].legend(ncol=4,fontsize=7)
    save(fig,'prior_error_accounting','Exact signed error accounting from the first usable reference readout to first HOLD on the eight original recorded trajectories: initial error plus roster changes plus common-voter increment error equals total displacement error. Every case has one uninterrupted common-voter segment. The components are correlated bookkeeping terms, not independently identified causal effects. No extra simulation is represented.',{'prior_log_audit.json':study.sha(study.AUDIT/'prior_log_audit.json')})
    dump(study.PUBLIC/'figure_manifest.json',dict(figures=CAPTIONS,script_sha256=study.sha(__file__),
        formats=['PDF embedded TrueType','SVG text','PNG 220dpi'],
        sampling='Full recorded control-time rows; no smoothing or synthetic points. Statistical unit is a condition block, not a frame.'))
    totals=table(['消融臂','安全完成','错误HOLD','G500','硬安全失败','约束缩放','截尾'],[
        [LABELS[arm],str(a['aggregates'][arm]['safe_completed'])+'/8',a['aggregates'][arm]['false_hold_cases'],
         str(a['aggregates'][arm]['g500'])+'/8',a['aggregates'][arm]['hard_safety_cases'],a['aggregates'][arm]['barrier_scalings'],
         sum(r.get('outcome','').startswith('CENSORED') for r in rows if r['arm']==arm)] for arm in arms])
    cases=table(['条件','臂','终点误差/m','物理秒','状态','G500原因'],[
        [r['condition'],r['arm'],format(r.get('endpoint_error_m',float('nan')),'.6f'),r.get('seconds'),
         r.get('outcome','EXECUTION_ERROR'),r.get('g500_reasons','unknown')] for r in rows if not r['safe_completed'] or not r['g500']])
    text=f"""# 投票集合连续性与停止确认：固定预算开发消融

状态：32回合固定比较完成；预注册组合候选采用门槛 **{a['adoption_gate']}**。所有结果见[完整逐回合表](episodes.csv)和[分析JSON](analysis.json)。默认仍关闭，论文题目未修改。

{totals}

这里的“错误HOLD”严格指首次进入HOLD时，独立评价的真实目标误差超过原任务容差0.12m；它不由控制器读取。没有进入HOLD的截尾回合不能算可靠停止。G500、硬安全、实际完成分别报告。

## 比较身份

8个已经消费过的开发条件 × 4臂，固定seed 20261102，每回合至多600秒，16机器人、dt=0.05秒。参考是上一轮0.45运动估计先验的开发候选，**不是全局默认0.25**。四臂共用该设置、相同初态、同一感知/物理/推手控制、安全及任务判据。8条原方法重跑的全部replay数组及原有逐帧科学字段均精确复现上一轮；新增诊断字段不参加旧字段比较。32条均为新执行，未把旧数据重复充作新增样本。

身份核验明确区分配置哈希与数值：显式增加False/1两个默认开关会改变config_sha256和config_hash。已核对配置仅有这两项增加，分别用原SHA-256及BLAKE2算法重算两个哈希；这两个身份字段单列，所有其余旧字段仍要求精确相等。首版分析把身份哈希也当作科学数值比较而触发断言，原分析及日志已保留在本地审计；只修正离线核验，没有重跑仿真、修改结果或放宽数值容差。

连续化仅在相邻时刻共有的有效投票者上计算位移变化，累加抵消换票贡献；每个机器人维护自己的本地实例，只接收自身和一跳消息。全队实例只供原有公共阶段监督器。无共有成员时不假装连续：该帧拒绝读数并以当前原始值建立新段，下一帧有共有成员才恢复。切换地图/参考估计来源时分段。

连续化作用于已有局部导航和团队导航的整个读数链，因此也可能改变接触前的转向和部署轨迹，不能把该臂的所有终点差异归因于最后一次停止跳变。相反，“仅停止确认”在原方法首次HOLD以前的轨迹及科学字段逐帧对比见analysis.json的stop_only_prefix_equivalence；它是区分停止许可与早期闭环改变的对照。

停止确认仅将原到达、低速、可用估计条件要求持续20帧（1秒），对照为原1帧。窗口事先借用已有contact_dwell长度，没有试多个窗口。制动、局部可撤销卸载及独立物理停稳规则保持原样；没有修改0.02m控制到达带、0.12m任务容差或安全阈值。

## 区分瞬时跳变与历史偏差

上一轮远场斜箱首次HOLD处，物体没有移动，移除agent_03贡献−49.25mm，共有成员变化−3.83mm，总跳变−53.08mm。这让估计误差进入20mm到达带，但真实误差仍135.11mm。

从同轮首次有效参考读数累计到HOLD，斜箱x误差分解为：初值+51.02mm、换票累计−22.06mm、共有成员相对真位移的误差+92.08mm，合计+121.04mm。远场C形则为+152.22−103.37+26.63=+75.48mm。因而部分换票恰好抵消历史偏差；消除跳变并不等于估计无偏，也不能把全部误差归因于换票。

公式为 e_k = e_0 + Σ(roster_delta) + Σ(common_delta − true_delta)，仅在连续有效段内成立。共有成员误差仍混合配准、旋转参考点、历史估计和成员选择效应，不能直接指定某一模块为根因。数值恒等式最大残差≤1.12e−16m，不是精度证明。

[全部回合图](all_episodes.pdf) · [停止与偏差时间序列](stop_and_bias.pdf) · [旧日志累计误差分解](prior_error_accounting.pdf) · [首次HOLD与同输入误差](stop_audit.json)。

## 本轮研究判断

**不采用这两个设置替换原方法；停止扩展本轮“全程抵消换票偏移＋固定1秒确认”的组合。** 六个近场条件四臂都安全完成；原本完成的远场C形在三个修改臂均未完成，斜箱也没有救回。所有臂G500仍6/8；硬安全失败均为0，但约束缩放另计，不能把它们抹去。这里只否定本轮具体方案，并不否定继续研究可撤销的停止确认。

斜箱连续化臂的同输入原始/使用读数MAE为44.30/305.83mm；组合臂为41.29/331.89mm。组合进入HOLD时，原始x估计误差约+30.50mm，累积连续化偏移约−389.71mm，实际使用的x估计误差变成−359.21mm；即使连续20帧满足原停止条件，仍在真实误差360.16mm处停止。这是候选自己引入并保留大幅聚合偏置的直接数值证据，不能只归咎于原有估计。连续桥接恒等式残差很小，也不代表读数正确。

仅确认臂在原HOLD之前逐帧一致，随后阻止了原斜箱错误HOLD，但到600秒仍在BRAKE、终点误差741.60mm；C形误差140.85mm，且未完成停稳终止。这说明“没有锁定错误HOLD”和“可靠完成任务”是两个指标。连续化C形与组合C形的终点误差均778.34mm；该轨迹没有一次满足全部原停止条件，不能通过解释为确认窗太长来掩盖反馈本身的问题。

下一项有价值的核验应围绕一致的位移参考与偏差来源：分别量化参考初始化误差、共有读数的后续偏差、成员切换的短暂影响，避免把所有成员变化永久积成校正量。可以保留可撤销的到达候选状态作为后续设计，但本轮不再缩短窗口、调增益或换seed追求成功。

## 全部任务或G500不通过项

{cases}

## 结论边界与复现

这些是事后选定的8个开发条件，不支持任意形状、90%可靠性或对其他部署算法的优势结论。同一臂内部的raw/used MAE使用完全相同的输入与物理时刻；跨臂MAE依赖不同轨迹及终止时刻，不能当作同输入精度排名。确认持续到达只能检验估计的时间持续性，无法证明绝对位移正确。全部失败、截尾及负向变化保留；本轮停止，不追加调参。

实际环境记录于analysis.json，PyYAML仍为本轮开始前已有的6.0.2，与锁文件6.0.3不一致；未安装或升级依赖。本轮所有臂同环境。

- [冻结协议](../../../experiments/ccta2027/voter_continuity_v1.json)
- [执行入口](../../../scripts/voter_continuity_study.py)
- [旧日志审计](../../../scripts/audit_voter_continuity.py)
- [结果分析](../../../scripts/analyze_voter_continuity.py)
- [图表与报告生成](../../../scripts/publish_voter_continuity.py)
- [图—数据—脚本哈希及英文caption](figure_manifest.json)

原始结果、源代码快照和收据：runs/research/{study.MISSION}/ablation/。保护、状态及恢复记录：artifacts/migrations/{study.MISSION}/。工程验证及逐帧分析CSV：runs/maintenance/{study.MISSION}/。大体积原始轨迹和备份留在本仓库内，不自动加入Git。
"""
    output.write_text(text,encoding='utf-8')
    study.event('publish',report_sha256=study.sha(output))
    print(json.dumps(dict(report=str(output),figures=list(CAPTIONS))))


if __name__=='__main__': main()
