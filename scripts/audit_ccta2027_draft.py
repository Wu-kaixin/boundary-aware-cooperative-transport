"""Verify the final local research-draft package without rerunning physical experiments."""
from __future__ import annotations
import argparse
import ast
import json
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
from pypdf import PdfReader
import resource_noise_validation as study
from dbact_sim.experiments.deployment import dump

PAPER = study.ROOT/'manuscript/ccta2027'
REFRESH = study.ROOT/'runs/research/2026-11-04-ccta2027-draft-refresh-v1'


def pdf_audit(path):
    reader=PdfReader(path)
    fonts={}; visited=set()
    def resources(res):
        if res is None:
            return
        res=res.get_object()
        for font in res.get('/Font',{}).get_object().values() if hasattr(res.get('/Font',{}),'get_object') else res.get('/Font',{}).values():
            obj=font.get_object()
            key=getattr(font,'idnum',str(obj.get('/BaseFont')))
            children=obj.get('/DescendantFonts')
            base=children[0].get_object() if children else obj
            desc=base.get('/FontDescriptor')
            desc=desc.get_object() if desc else {}
            entry=dict(name=str(obj.get('/BaseFont')),subtype=str(obj.get('/Subtype')),
                descendant_subtype=str(base.get('/Subtype')),
                embedded=any(k in desc for k in ('/FontFile','/FontFile2','/FontFile3')))
            assert entry['subtype']!='/Type3' and entry['descendant_subtype']!='/Type3',entry
            assert entry['embedded'],entry
            fonts[str(key)]=entry
        xo=res.get('/XObject',{})
        xo=xo.get_object() if hasattr(xo,'get_object') else xo
        for item in xo.values():
            key=getattr(item,'idnum',id(item))
            if key in visited:
                continue
            visited.add(key)
            obj=item.get_object()
            if obj.get('/Subtype')=='/Form':
                resources(obj.get('/Resources'))
    text_lengths=[]
    for page in reader.pages:
        assert [float(v) for v in page.mediabox[2:]]==[612.,792.]
        resources(page.get('/Resources'))
        text=page.extract_text() or ''
        assert '\ufffd' not in text
        text_lengths.append(len(text.strip()))
        assert len(text.strip())>100, 'Nearly empty page in '+str(path)
    return dict(pages=len(reader.pages),paper='US Letter',fonts=list(fonts.values()),
        all_fonts_embedded=True,type3_fonts=0,page_text_characters=text_lengths,sha256=study.sha(path))


def test_receipt(path):
    suites=list(ET.parse(path).iter('testsuite'))
    counts={k:sum(int(s.attrib.get(k,0)) for s in suites) for k in ['tests','failures','errors','skipped']}
    counts['passed']=counts['tests']-counts['failures']-counts['errors']-counts['skipped']
    assert counts['errors']==counts['failures']==0
    return dict(**counts,receipt_sha256=study.sha(path))


def main(out):
    study.guard(); study.verify()
    out=out.resolve()
    if out.exists() or not out.is_relative_to(REFRESH):
        raise ValueError('Fresh audit directory required')
    data=study.read(study.BASE/'analysis_v1/analysis.json')
    summary=study.read(study.BASE/'run/summary.json')
    assert summary['source_integrity']=='PASS'
    assert summary['completed_denominator']==summary['planned_denominator']==60
    assert summary['execution_errors']==0
    rows=data['episodes']
    assert len({r['case_id'] for r in rows})==60
    assert sum(r['completed'] for r in rows)==58 and sum(r['safe_completed'] for r in rows)==56
    assert sum(bool(r['hard_safety']) for r in rows)==2
    assert all(r['ready_s'] is not None and r['first_contact_s'] is not None for r in rows)
    assert study.sha(study.BASE/'analysis_v1/all_60_episodes.csv')==study.sha(study.ROOT/'docs/results/resource_noise_validation_v1/all_60_episodes.csv')
    media={}
    for folder,name in [('media_v1','media_manifest.json'),('diagnostics_v1','diagnostics_manifest.json')]:
        receipt=study.read(study.BASE/folder/name)
        for file,expected in receipt['artifacts'].items():
            assert study.sha(study.BASE/folder/file)==expected,file
        media[folder]=dict(manifest_sha256=study.sha(study.BASE/folder/name),verified_artifacts=len(receipt['artifacts']))
    core=test_receipt(REFRESH/'core_tests_v2.xml')
    mas=test_receipt(REFRESH/'mas_tests.xml')
    assert core['passed']==1113 and core['skipped']==2 and mas['passed']==106
    pdfs={name:pdf_audit(PAPER/(name+'.pdf')) for name in ('main','supplementary')}
    assert pdfs['main']['pages']==8
    assert pdfs['supplementary']['pages']<=8
    for name in pdfs:
        log=(PAPER/(name+'.log')).read_text(encoding='utf-8',errors='replace')
        assert not re.search(r'Overfull|Missing character|Undefined control sequence|undefined references|Citation .+ undefined',log)
    tex=(PAPER/'main.tex').read_text()
    cites={k for s in re.findall(r'\\cite\{([^}]+)\}',tex) for k in s.split(',')}
    bib=set(re.findall(r'\\bibitem\{([^}]+)\}',(PAPER/'references.tex').read_text()))
    assert cites==bib and len(bib)==20,(cites-bib,bib-cites)
    helpers=['resource_noise_validation.py','analyze_resource_noise_validation.py','render_resource_noise_media.py',
        'diagnose_resource_noise_results.py','publish_resource_noise_evidence.py','build_ccta2027_latest_evidence.py','audit_ccta2027_draft.py']
    for helper in helpers:
        ast.parse((study.ROOT/'scripts'/helper).read_text(encoding='utf-8'))
    diff=subprocess.run(['git','diff','--check'],capture_output=True,text=True,check=True)
    out.mkdir(parents=True)
    audit=dict(status='PASS',scope='Local draft delivery, not algorithm/safety/submission acceptance',
        source_sha256=data['source_sha256'],protocol_sha256=data['protocol_sha256'],
        runtime_unchanged=True,scientific_episodes=60,execution_errors=0,spatial_completed=58,
        completed_without_original_hard_safety_failure=56,core_tests=core,mas_tests=mas,
        pdfs=pdfs,bibliography_items=20,media=media,python_helpers_parsed=helpers,
        git_diff_check='PASS',visual_review='Human-style agent visual inspection recorded separately in FINAL_AUDIT.md',
        note='Original raw-product verification is the completed analysis_v1; this audit rechecks identities, compact tables and media hashes.')
    dump(out/'audit.json',audit)
    report=f'''# Final local draft audit — 2026-11-04

**Result: local research-draft deliverables verified.** This is not acceptance of a new transport algorithm, dynamic safety guarantee, or formal conference submission.

| Required deliverable | Evidence and result |
|---|---|
| Trustworthy and reproducible baseline | Current source/config/environment identity rechecked; protected YAML and runtime unchanged. Core regression: {core['passed']} passed, {core['skipped']} skipped; MAS: {mas['passed']} passed. |
| Scientific question and contribution decision | [Research summary](RESEARCH_SUMMARY.md); deployment-to-transport gap and directional-capability analysis, with novelty and information limits. |
| Multishape full-task comparison | Complete historical populations remain separate; current frozen protocol has 60/60 terminal receipts, zero execution errors, 58 spatial completions, 56 without original hard safety failure. |
| Mathematics and assumptions | Contact equations, QP set/projection distinction and dual bound in main; restricted identifiability/accounting in supplement; source mapping in [index](RESULTS_INDEX.md). |
| Success, failure and limitation report | Both incomplete current cases and both penetration failures retained; rejected candidate branches and interrupted full constraint audit explicitly identified. |
| Figures, media and reproduction | Matplotlib figures, four new fully decoded MP4s, verified 81-frame GIF, Inkscape vector export, raw-replay hashes and fresh-output commands in [media guide](MEDIA_AND_CLAIMS.md). |
| English source, references and PDF | Main {pdfs['main']['pages']} pages, supplement {pdfs['supplementary']['pages']} pages, both Letter. Twenty main references resolve; fonts embedded, zero Type 3; no overfull, missing-glyph or undefined-reference warning. |

## Verification details

- Main final pages 1–8 were rendered and inspected. Final edits changed only pages 4 and 6 relative to the earlier fully reviewed eight-page render; all other page PNG hashes are identical. The changed pages were inspected again.
- Supplement was re-rendered after fixing float-only heading pages. Every final page was inspected for readable mathematics, labels, captions and cropping. This supplement is a local reading companion, not a claim of conference attachment compliance.
- New media final-state previews were inspected; start and intermediate frames were additionally checked. MP4 decoder checks cover all frames, while visual sampling does not claim inspection of every animation frame.
- Main font objects: {len(pdfs['main']['fonts'])}; supplement: {len(pdfs['supplementary']['fonts'])}. Recursive PDF resource/font-descriptor checks use the root environment's `pypdf`; `pdffonts` is unavailable in the local Poppler subset. Poppler rendered the pages.
- `git diff --check` and syntax parsing of the seven new research/presentation helpers pass. Scientific helpers also completed their built-in source, input/product identity, denominator and numerical consistency assertions during actual execution.
- The first core test attempt failed during temporary-directory setup under the sandbox. Its logs remain separate; the approved rerun in a fresh repository-local test directory completed with the counts above. No software failure is concealed by discarding that receipt.
- The first supplementary build lacked cached `article.cls`; it was changed to the existing official `ieeeconf` class in one-column mode. The successful build still emits a Fontconfig configuration message, but recursive font inspection confirms embedded fonts and visual rendering shows the intended glyphs. Main underfull spacing warnings are retained and inspected; no overfull warning remains.
- Inkscape 1.4.4 was downloaded from its official release link as a local portable tool. Its noninteractive SVG-to-PDF export succeeded and was visually checked. GTK display/recent-file warnings are retained as environment messages; no system-wide installation or solver change occurred.

Machine-readable results: `runs/research/2026-11-04-ccta2027-draft-refresh-v1/{out.name}/audit.json`. The same directory's `release_manifest.json` identifies final local deliverables. Raw current validation remains in `runs/research/2026-11-04-resource-noise-validation-v1/`; no historical run or source snapshot was rewritten.

## Scope left deliberately unclaimed

No complete finite-support constraint PASS; no resumed rejected algorithm; no fully decentralized supervision; no arbitrary-shape/noise reliability or continuous-time contact-safety theorem; no hardware validation, remote push, public upload or formal submission. Author identities remain pending. These are explicit scientific and publication boundaries, not missing local-draft deliverables.
'''
    (PAPER/'FINAL_AUDIT.md').write_text(report,encoding='utf-8')
    docs=[PAPER/p for p in ['README.md','RESULTS_INDEX.md','RESEARCH_SUMMARY.md','MEDIA_AND_CLAIMS.md','sources.md','FINAL_AUDIT.md']]
    docs.append(study.ROOT/'docs/results/resource_noise_validation_v1/README.md')
    checked_links=0
    for doc in docs:
        for target in re.findall(r'\]\(([^)]+)\)',doc.read_text(encoding='utf-8-sig')):
            if target.startswith(('http:','https:','#','mailto:')):
                continue
            target=target.split('#')[0].strip('<>')
            assert (doc.parent/target).exists(),(doc,target)
            checked_links+=1
    files=[p for p in PAPER.rglob('*') if p.is_file() and p.suffix.lower() in ('.tex','.pdf','.md','.json','.svg','.png')]
    files += [study.ROOT/'scripts'/p for p in helpers]
    files += [study.PROTOCOL,study.DESIGN,out/'audit.json']
    dump(out/'release_manifest.json',dict(local_links_checked=checked_links,
        files={str(p.relative_to(study.ROOT)).replace('\\','/'):study.sha(p) for p in files},
        source_sha256=data['source_sha256'],audit_generator_sha256=study.sha(Path(__file__))))
    print(json.dumps(dict(status='PASS',pdf_pages={k:v['pages'] for k,v in pdfs.items()},
        font_counts={k:len(v['fonts']) for k,v in pdfs.items()},local_links_checked=checked_links,
        core_tests=core,mas_tests=mas,manifest_files=len(files))),flush=True)


if __name__=='__main__':
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out',type=Path,required=True)
    main(ap.parse_args().out)
