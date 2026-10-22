import json,re,sys
from pathlib import Path
sys.stdout.reconfigure(encoding='utf-8')
ident=sys.argv[1]
pages=json.loads(Path(f'data/literature/2026-10-22/text/{ident}.pages.json').read_text(encoding='utf-8'))
start=int(sys.argv[2]) if len(sys.argv)>2 else 1
end=int(sys.argv[3]) if len(sys.argv)>3 else len(pages)
in_refs=False
for i in range(start-1,min(end,len(pages))):
    # Prose reading view: all prose lines retained, display math is deferred.
    lines=[]
    seen=set()
    for s in pages[i].splitlines():
        if re.match(r'^\s*(REFERENCES|References|Bibliography)\s*$',s):
            in_refs=True
            continue
        if re.match(r'^\s*(APPENDIX|Appendix)',s):
            in_refs=False
        if in_refs:
            continue
        if len(re.findall(r'[A-Za-z]',s))<6:
            continue
        if s.strip() in seen:
            continue
        seen.add(s.strip())
        lines.append(s.strip())
    print(f'=== {ident} PDF PAGE {i+1}/{len(pages)} PROSE (equations/figures deferred) ===\n'+ ' '.join(lines))
