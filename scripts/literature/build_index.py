import json
from pathlib import Path
from collections import Counter, defaultdict

root=Path('docs/literature_review_2026-10-22')
inv=json.loads((root/'inventory.json').read_text(encoding='utf-8'))
manifest={x['reading_id']:x for x in json.loads((root/'source_manifest.json').read_text(encoding='utf-8'))}
full={'L06','L07','L10','L15','L21','L24','L48','L61','L66','L68'}
read={p.stem for p in (root/'notes').glob('L*.md')}
rows=[]
labels={'full_extracted_text_read':'全文提取文本已读；公式/图待复核','body_prose_read':'正文文字通读；显示公式/图待复核','audit_text_read':'审计全文文字已读；非学术论文','original_pages_read':'原始六页图像已读；证明未独立验证','pending':'待阅读'}
for f in inv['files']:
    ident=f['reading_id']; m=manifest[ident]
    status='audit_text_read' if ident=='L12' else 'original_pages_read' if ident=='L45' else 'full_extracted_text_read' if ident in full else 'body_prose_read' if ident in read else 'pending'
    rows.append({'id':ident,'title':f['title'],'source':f['url'],'pages':m.get('pages'),'sha256':m['sha256'],'reading_status':status,'note':f'notes/{ident}.md' if ident in read else None,'bibliography_status':'see_note_and_verification_log' if ident in read else 'pending','original_page_visual_checks':{'L68':[15],'L45':[1,2,3,4,5,6],'L31':[26],'L34':[7,8],'L36':[3,4],'L56':[7]}.get(ident,[]),'independent_proof_verification':'pending' if ident!='L12' else 'not_applicable'})
counts=Counter(r['reading_status'] for r in rows)
hashes=defaultdict(list)
for r in rows: hashes[r['sha256']].append(r['id'])
duplicates=[v for v in hashes.values() if len(v)>1]
report={'snapshot_date':'2026-10-22','scope':'first-pass body reading complete; proof, figure and publication-version verification remain separate','counts':dict(counts),'pdfs':70,'audit_documents':1,'total_pdf_pages':sum(r['pages'] or 0 for r in rows),'byte_identical_groups':duplicates,'rows':rows}
(root/'reading_ledger.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
lines=['# 文献索引与阅读进度（首轮正文阅读完成）','',f"70 个 PDF，共 {report['total_pdf_pages']} 页；另 1 份历史审计 DOCX。70 篇 PDF 已完成首轮正文阅读（10 篇完整提取文本、59 篇正文文字、1 篇原始六页图像）；另 1 份历史审计文字已读。**关键公式、原图和证明独立验证尚未全部完成。**",'',f'原文件全部可获取、均可提取文本；字节相同的文件组：{duplicates or "无"}。版本/会议扩展/内容重复仍需人工核对，不能由哈希去重代替。逐页缺图、乱码和公式提取质量需在精读中复核。','', '作者、年份与版本信息记录在各篇笔记；未核实项不从文件修改日期或 PDF 模板推断。正文完成不代表证明正确或图表已经复核。','', '| ID | 文件与原文入口 | PDF 页数 | 阅读状态 | 阅读记录 |','| --- | --- | ---: | --- | --- |']
for r in rows:
    title=r['title'].replace('|','/').removesuffix('.pdf')
    note=f"[笔记]({r['note']})" if r['note'] else '待建立'
    lines.append(f"| {r['id']} | [{title}]({r['source']}) | {r['pages'] or '—'} | {labels[r['reading_status']]} | {note} |")
(root/'reading_index.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
print(json.dumps({k:report[k] for k in ['counts','total_pdf_pages','byte_identical_groups']},ensure_ascii=False))
