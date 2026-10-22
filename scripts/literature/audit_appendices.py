import json,re
from pathlib import Path
for p in sorted(Path('data/literature/2026-10-22/text').glob('L*.pages.json')):
 pages=json.loads(p.read_text(encoding='utf-8')); refs=False; hits=[]
 for i,t in enumerate(pages):
  for line in t.splitlines():
   if i>=len(pages)*.55 and re.match(r'^\s*(REFERENCES|References|Bibliography|EFERENCES)\s*$',line): refs=True
   if refs and re.search(r'(?i)\bappendi[xc]',line) and len(line.strip())<100:
    if not re.match(r'^\s*(APPENDIX|Appendix)',line): hits.append((i+1,line.strip()))
    else: refs=False
 if hits: print(p.stem, hits)
