import json,re
from pathlib import Path
root=Path('docs/literature_review_2026-10-22')
inv=json.loads((root/'inventory.json').read_text(encoding='utf-8'))['files']
groups={
'理论基础与覆盖驻点': ['L31','L35','L50','L69'],
'局部覆盖、可见性与计算权限':['L03','L24','L42','L48','L64'],
'未知场、探索与估计':['L11','L22','L37','L40','L41','L46','L54','L56'],
'时变配置与覆盖任务定义':['L08','L25','L27','L28','L33','L44','L45','L55'],
'未知边界与目标包围':['L04','L09','L30','L39','L51','L53','L58','L60','L70','L71'],
'自身前期框架与贡献增量':['L10'],
'配置、接触闭合与任务作用':['L02','L06','L18','L19','L21','L26','L34','L36','L38','L47'],
'附着、反馈与协作运输':['L01','L07','L13','L15','L16','L17','L29'],
'安全保证、扰动与数字执行':['L05','L14','L20','L23','L32','L43','L49','L52','L59','L61','L62','L65','L66','L67','L68'],
'法向估计与误差前提':['L57','L63']}
route={i:g for g,ids in groups.items() for i in ids}
draft=(root/'manuscript_working.md').read_text(encoding='utf-8')
rows=[]
for f in inv:
 ident=f['reading_id']; p=root/'notes'/f'{ident}.md'; s=p.read_text(encoding='utf-8')
 if '\n## 原文入口\n' not in s:
  p.write_text(s.rstrip()+f"\n\n## 原文入口\n\n[用户文件夹原文件]({f['url']})；文件身份及阅读状态见 [索引](../reading_index.md)。\n",encoding='utf-8')
 if ident=='L12': use='内部历史审计，不作为学术引文'; status='不纳入学术正文'
 else:
  assert ident in route,ident
  use=route[ident]; status='已用于 Markdown 论证草稿' if f'notes/{ident}.md' in draft else '背景库，未写入正文'
 rows.append({'id':ident,'title':f['title'],'use':use,'status':status,'source':f['url'],'note':f'notes/{ident}.md'})
assert len(route)==70,len(route)
lines=['# 逐篇正文整合用途','', '70 篇论文均已在 Markdown 相关工作/问题论证中有对应位置，另1份审计只用于内部追溯。当前草稿有意保留完整证据链，正式投稿时再按篇幅合并同类引文；引用多不等于贡献强。全部首轮阅读不等于独立证明、最新出版版本都已验证。','', '| ID | 原文件 | 正文用途 | 当前处理 | 阅读依据 |','| --- | --- | --- | --- | --- |']
for r in rows:
 title=r['title'].removesuffix('.pdf').replace('|','/')
 lines.append(f"| {r['id']} | [{title}]({r['source']}) | {r['use']} | {r['status']} | [含页码笔记]({r['note']}) |")
(root/'citation_integration_map.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
(root/'citation_integration_map.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'papers_routed':len(route),'papers_in_draft':sum(r['status']=='已用于 Markdown 论证草稿' for r in rows),'notes':len(list((root/'notes').glob('L*.md')))},ensure_ascii=False))
