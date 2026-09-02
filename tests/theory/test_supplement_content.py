"""The LaTeX and Word supplements must keep saying the same thing.

Both renderers consume ``theorem1_document.document()``.  These tests pin that
contract, the claim boundary, and the LaTeX->OMML conversion rules that a
silent-corruption bug would otherwise slip through.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SUPPLEMENT = ROOT / "theory" / "supplement"
sys.path.insert(0, str(SUPPLEMENT))

import theorem1_document as content  # noqa: E402


CERT = json.loads(
    (ROOT / "theory" / "certificates" / "analytic_constants.json").read_text(encoding="utf-8")
)
BLOCKS = content.document(CERT)


def texts() -> list[str]:
    """Every human-readable string the document emits."""
    out: list[str] = []
    for block in BLOCKS:
        kind, args = block[0], block[1:]
        if kind in ("h1", "h2", "h3", "para", "lead"):
            out.append(args[0])
        elif kind == "bullets":
            out.extend(args[0])
        elif kind == "callout":
            out.extend(args)
        elif kind == "table":
            out.extend(cell for row in args[1] for cell in row)
        elif kind == "chain":
            out.extend(why for _, why in args[0])
    return out


def equations() -> list[str]:
    """Every LaTeX fragment, display and inline."""
    out: list[str] = []
    for block in BLOCKS:
        kind, args = block[0], block[1:]
        if kind == "eq":
            out.append(args[0])
        elif kind == "chain":
            out.extend(latex for latex, _ in args[0])
    for text in texts():
        parts = text.split("$")
        out.extend(parts[1::2])
    return out


# --------------------------------------------------------------------- #
# content
# --------------------------------------------------------------------- #

def test_every_numbered_section_is_present():
    headings = [b[1] for b in BLOCKS if b[0] == "h1"]
    for expected in (
        "1. Scope and frozen claim",
        "2. Notation",
        "3. Assumptions A1-A10",
        "4. Lemma 1 - boundary and map to density consistency",
        "5. Lemma 2 - mass, quadrature and centroid perturbation",
        "6. Proposition 1 - ideal Local-CVT gradient and descent",
        "7. Proposition 2 - aggregate implementation disturbance",
        "8. Proposition 3 - reference rate and local regularity",
        "9. Theorem 1 - local practical stability",
        "10. Corollaries",
        "11. Limits and evidence status",
        "Appendix A. Constant ledger",
        "Appendix B. Wolfram verification manifest",
    ):
        assert expected in headings


def test_each_main_result_carries_a_step_by_step_proof():
    """A statement without a worked derivation is what this rewrite removed."""
    steps = [b[1] for b in BLOCKS if b[0] == "h3"]
    assert sum(s.startswith("Step ") for s in steps) >= 15
    for result in ("4.2 Proof", "5.2 Proof", "6.2 Proof",
                   "7.2 Proof", "9.2 Proof - every inequality named"):
        assert any(b[0] == "h2" and b[1] == result for b in BLOCKS)


def test_the_lyapunov_chain_justifies_every_inequality():
    chains = [b[1] for b in BLOCKS if b[0] == "chain"]
    theorem_chain = max(chains, key=len)
    assert len(theorem_chain) >= 7
    for _, why in theorem_chain:
        assert why.strip(), "an inequality with no stated justification"
    joined = " ".join(why for _, why in theorem_chain)
    for cited in ("Young", "PL", "P2", "P1-b", "Cauchy-Schwarz"):
        assert cited in joined


def test_ledger_rows_come_from_the_generated_certificate():
    rows = [cell for b in BLOCKS if b[0] == "table" and b[1] == "ledger"
            for row in b[2] for cell in row]
    for name in content.LEDGER_CONSTANTS:
        item = CERT["constants"][name]
        assert name in rows
        assert f"{item['value']:.6g}" in rows
        assert item["status"] in rows


def test_conditional_results_are_never_reported_as_proved():
    rows = [cell for b in BLOCKS if b[0] == "table" and b[1] == "limits"
            for row in b[2] for cell in row]
    assert rows.count("CONDITIONAL") == 2
    assert "Proposition 3" in rows and "Theorem 1" in rows


def test_claim_boundary_is_stated_verbatim():
    joined = " ".join(texts()) + " " + content.CALLOUT_BODY
    assert "No global PL." in joined
    assert "No formal caging." in joined
    assert "No unconditional safety or cooperative-transport success." in joined
    assert "a uniform reduced-Hessian bound for an unspecified local branch." in joined
    assert "This supplement does not prove any of the following." in joined


def test_no_numeric_sanity_result_is_promoted_to_proof():
    joined = " ".join(texts())
    assert "NUMERIC-SANITY: debugging evidence only." in joined
    assert "no such entries" in joined


# --------------------------------------------------------------------- #
# maths conversion
# --------------------------------------------------------------------- #

docx_builder = pytest.importorskip("build_supplement_docx")


def test_every_equation_survives_conversion_to_word():
    """The LaTeX->MathML->OMML chain drops symbols silently; the builder's own
    guard must accept every formula the document actually contains."""
    for latex in equations():
        docx_builder.latex_to_omml(latex)


@pytest.mark.parametrize("latex,must_keep", [
    (r"\|d\|^2", "d"),           # a lone symbol inside a norm
    (r"\|\nabla H^*\|", "∇"),    # a gradient inside a fence
    (r"\pi^{3/2}", "/"),         # a slash in a superscript
    (r"\left(a + b + c\right)", "+"),   # three terms inside a fence
])
def test_known_conversion_traps_are_neutralised(latex, must_keep):
    from lxml import etree
    root = etree.fromstring(docx_builder.latex_to_omml(latex))
    payload = docx_builder._omml_payload(root)
    assert must_keep in payload, f"{must_keep!r} lost from {latex!r}"


@pytest.mark.parametrize("raw,expect_lost", [
    (r"a/b", "/"),                       # the slash MML2OMML drops
    (r"\|d\|^2", "d"),                   # a scripted norm loses its content
    (r"\left(a + b + c\right)", "+"),    # the operators inside a fence
])
def test_the_conversion_guard_actually_detects_loss(raw, expect_lost):
    """Without normalisation these inputs lose content.

    If this stopped detecting loss, every other conversion test above would
    pass vacuously, so the detector is checked against the raw constructs
    rather than the normalised ones.
    """
    import latex2mathml.converter as l2m
    from lxml import etree

    mathml = l2m.convert(raw)
    root = docx_builder._load_xslt()(etree.fromstring(mathml.encode())).getroot()
    lost = docx_builder._significant_diff(
        docx_builder._mathml_payload(mathml), docx_builder._omml_payload(root)
    )
    assert expect_lost in lost, f"guard no longer sees {expect_lost!r} lost from {raw!r}"


def test_renderers_cover_every_block_kind():
    import render_latex
    kinds = {b[0] for b in BLOCKS}
    known = {"h1", "h2", "h3", "para", "lead", "eq", "chain",
             "bullets", "callout", "table", "pagebreak"}
    assert kinds <= known, f"unhandled block kind: {kinds - known}"
    for kind in kinds:
        if kind in ("eq", "pagebreak"):
            continue
        assert hasattr(docx_builder.DocxRenderer, kind), f"docx renderer lacks {kind}"
    render_latex.render(BLOCKS)   # raises on an unhandled kind
