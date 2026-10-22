import json,re,sys
from pathlib import Path
root=Path('data/literature/2026-10-22')
inv=json.loads(Path('docs/literature_review_2026-10-22/inventory.json').read_text(encoding='utf-8'))['files']
chunks=[]
for f in inv:
    ident=f['reading_id']
    if f['mime_type']!='application/pdf': continue
    pages=json.loads((root/'text'/f'{ident}.pages.json').read_text(encoding='utf-8'))
    in_refs=False
    alltext=''
    for i,page in enumerate(pages):
        lines=[]; seen=set()
        for s in page.splitlines():
            if i >= len(pages)*0.55 and re.match(r'^\s*(REFERENCES|References|Bibliography|EFERENCES)\s*$',s): in_refs=True;continue
            if re.match(r'^\s*(APPENDIX|Appendix)',s): in_refs=False
            if in_refs: continue
            if len(re.findall(r'[A-Za-z]',s))<6: continue
            if s.strip() in seen: continue
            seen.add(s.strip());lines.append(s.strip())
        if lines: alltext+=f'\n=== {ident} PDF p.{i+1} PROSE; display math/figures deferred ===\n'+ ' '.join(lines)+'\n'
    parts=[alltext[i:i+13500] for i in range(0,len(alltext),13500)]
    for j,t in enumerate(parts):
        path=root/'chunks'/f'{ident}-{j+1:02}.txt'
        path.parent.mkdir(exist_ok=True)
        path.write_text(t,encoding='utf-8')
        chunks.append({'id':ident,'part':j+1,'parts':len(parts),'path':str(path),'chars':len(t)})
(root/'chunks.json').write_text(json.dumps(chunks,indent=2),encoding='utf-8')
print(json.dumps({'chunks':len(chunks),'chars':sum(c['chars'] for c in chunks),'per_paper':{f['reading_id']:sum(c['id']==f['reading_id'] for c in chunks) for f in inv}},indent=2))
