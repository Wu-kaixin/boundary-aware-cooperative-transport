import argparse, json, re, sys
sys.stdout.reconfigure(encoding='utf-8')
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('id')
p.add_argument('start',type=int,nargs='?',default=1)
p.add_argument('end',type=int,nargs='?',default=999)
a=p.parse_args()
pages=json.loads(Path(f'data/literature/2026-10-22/text/{a.id}.pages.json').read_text(encoding='utf-8'))
for i in range(a.start-1,min(a.end,len(pages))):
    t=pages[i]
    t=re.sub(r'[ \t]+',' ',t)
    t=re.sub(r'\n{2,}','\n',t)
    print(f'=== {a.id} PDF PAGE {i+1}/{len(pages)} ===\n{t}')
