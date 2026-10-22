import hashlib, json, re, zipfile
from pathlib import Path
from xml.etree import ElementTree as ET
from pypdf import PdfReader

root = Path('data/literature/2026-10-22')
inventory = json.loads(Path('docs/literature_review_2026-10-22/inventory.json').read_text(encoding='utf-8'))
records = []
for f in inventory['files']:
    ident = f['reading_id']
    suffix = '.pdf' if f['mime_type'] == 'application/pdf' else '.docx'
    source = root / 'pdfs' / (ident + suffix)
    record = dict(reading_id=ident, title=f['title'], bytes=source.stat().st_size, sha256=hashlib.sha256(source.read_bytes()).hexdigest())
    if suffix == '.pdf':
        reader = PdfReader(source)
        pages = [p.extract_text() or '' for p in reader.pages]
        record.update(pages=len(pages), chars_per_page=[len(p) for p in pages], metadata={str(k):str(v) for k,v in (reader.metadata or {}).items()})
        text = '\n\n'.join(f'=== PDF PAGE {i+1} ===\n{p}' for i,p in enumerate(pages))
        (root / 'text' / (ident + '.pages.json')).write_text(json.dumps(pages,ensure_ascii=False),encoding='utf-8')
    else:
        with zipfile.ZipFile(source) as z:
            doc = ET.fromstring(z.read('word/document.xml'))
        ns={'w':'http://schemas.openxmlformats.org/wordprocessingml/2006/main'}
        text = '\n'.join(''.join(p.itertext()) for p in doc.findall('.//w:p',ns))
        record.update(chars=len(text))
    (root / 'text' / (ident + '.txt')).write_text(text,encoding='utf-8')
    records.append(record)
Path('docs/literature_review_2026-10-22/source_manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps([{'id':r['reading_id'],'pages':r.get('pages'), 'low_text_pages':[i+1 for i,n in enumerate(r.get('chars_per_page',[])) if n<100]} for r in records]))
