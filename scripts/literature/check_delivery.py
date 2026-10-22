import json,re,hashlib
from pathlib import Path
from fractions import Fraction
root=Path('docs/literature_review_2026-10-22'); inv=json.loads((root/'inventory.json').read_text(encoding='utf-8'))['files']; ledger=json.loads((root/'reading_ledger.json').read_text(encoding='utf-8'))
missing=[]
for p in root.rglob('*.md'):
 for target in re.findall(r'\]\(([^)]+)\)',p.read_text(encoding='utf-8')):
  if re.match(r'^(?:https?://|#)',target): continue
  target=target.split('#')[0]
  if target and not (p.parent/target).exists(): missing.append((str(p),target))
mat=(root/'capability_matrix.md').read_text(encoding='utf-8'); md=(root/'manuscript_working.md').read_text(encoding='utf-8')
expected={f['reading_id'] for f in inv if f['mime_type']=='application/pdf'}
mat_ids=set(re.findall(r'notes/(L\d+)\.md',mat)); draft_ids=set(re.findall(r'notes/(L\d+)\.md',md))
assert not missing,missing
assert expected==mat_ids,(expected-mat_ids,mat_ids-expected)
assert expected==draft_ids,(expected-draft_ids,draft_ids-expected)
assert not [r for r in ledger['rows'] if r['reading_status']=='pending']
tex=Path('manuscript/ccta2027/main.tex'); digest=hashlib.sha256(tex.read_bytes()).hexdigest().upper(); assert digest=='F5AB09BFD369037C65A1F977DE4EB24DE607A6EE9453F3328A317FF2935D7A5C'
assert Fraction(5,12)+Fraction(1,18)-Fraction(1,36)==Fraction(4,9)
report={'source_files':len(inv),'pdf_notes':len(expected),'all_notes':len(list((root/'notes').glob('L*.md'))),'pdf_pages':ledger['total_pdf_pages'],'counts':ledger['counts'],'matrix_pdf_coverage':len(mat_ids),'draft_pdf_coverage':len(draft_ids),'broken_relative_links':missing,'tex_sha256_unchanged':digest,'appendix_parser_check':'L35 numbered appendix separately read pp12-13; no other detected missed appendix headings','independent_proof_review':'not complete; narrow original-page checks only','algorithm_changes_this_stage':False,'new_large_experiments':False}
(root/'delivery_checks.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False,indent=2))
