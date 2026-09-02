"""The PDF and the Word supplement must keep saying the same thing.

Both renderers drive ``supplement_content.compose``.  These tests pin that
contract and the claim boundary, so a future edit cannot quietly promote a
conditional result in one format only.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SUPPLEMENT = ROOT / "theory" / "supplement"
sys.path.insert(0, str(SUPPLEMENT))

import supplement_content as content  # noqa: E402


CERT = json.loads(
    (ROOT / "theory" / "certificates" / "analytic_constants.json").read_text(encoding="utf-8")
)


class RecordingEmitter:
    """Collects every block ``compose`` emits, in order."""

    def __init__(self) -> None:
        self.blocks: list[tuple[str, str]] = []

    def title_block(self) -> None:
        for text in (content.KICKER, content.TITLE, content.SUBTITLE):
            self.blocks.append(("title", text))

    def section(self, title: str) -> None:
        self.blocks.append(("section", title))

    def subsection(self, title: str) -> None:
        self.blocks.append(("subsection", title))

    def lead(self, text: str) -> None:
        self.blocks.append(("lead", text))

    def body(self, text: str) -> None:
        self.blocks.append(("body", text))

    def bullets(self, items: list[str]) -> None:
        for item in items:
            self.blocks.append(("bullet", item))

    def equation(self, text: str) -> None:
        self.blocks.append(("equation", text))

    def toc(self, items: list[str]) -> None:
        for item in items:
            self.blocks.append(("toc", item))

    def callout(self, head: str, text: str) -> None:
        self.blocks.append(("callout", head))
        self.blocks.append(("callout", text))

    def table(self, kind: str, rows: list[list[str]]) -> None:
        for row in rows:
            for cell in row:
                self.blocks.append((f"table:{kind}", cell))

    def spacer(self, points: float) -> None:
        self.blocks.append(("spacer", str(points)))

    def pagebreak(self) -> None:
        self.blocks.append(("pagebreak", ""))


def compose_blocks() -> list[tuple[str, str]]:
    emitter = RecordingEmitter()
    content.compose(emitter, CERT)
    return emitter.blocks


def test_compose_covers_every_numbered_section():
    headings = [text for kind, text in compose_blocks() if kind == "section"]
    for expected in (
        "1. Scope and frozen claim",
        "2. Notation and assumptions",
        "3. Lemma 1 - boundary/map to density",
        "4. Lemma 2 - mass and centroid perturbation",
        "5. Proposition 1 - ideal gradient and descent",
        "6. Proposition 2 - aggregate disturbance",
        "7. Proposition 3 - reference rate and regularity",
        "8. Theorem 1 - local practical stability",
        "9. Corollaries",
        "10. Limits and evidence status",
        "Appendix A. Constant ledger",
        "Appendix B. Wolfram verification manifest",
    ):
        assert expected in headings


def test_ledger_rows_come_from_the_generated_certificate():
    rows = [text for kind, text in compose_blocks() if kind == "table:ledger"]
    for name in content.LEDGER_CONSTANTS:
        assert name in rows
        item = CERT["constants"][name]
        assert f"{item['value']:.6g}" in rows
        assert item["status"] in rows


def test_conditional_results_are_never_reported_as_proved():
    rows = [text for kind, text in compose_blocks() if kind == "table:limits"]
    # Proposition 3 and Theorem 1 inherit A8; both must stay conditional.
    assert rows.count("CONDITIONAL") == 2
    assert "Proposition 3" in rows and "Theorem 1" in rows


def test_claim_boundary_is_stated_verbatim():
    text = " ".join(t for _, t in compose_blocks())
    assert "No global PL." in text
    assert "No formal caging." in text
    assert "No unconditional safety or cooperative-transport success." in text
    assert "no uniform Hessian bound for an unspecified local branch." in text


def test_no_numeric_sanity_result_is_promoted_to_proof():
    text = " ".join(t for _, t in compose_blocks())
    assert "NUMERIC-SANITY: debugging evidence only" in text
    assert "There are no numeric-sanity entries in this package." in text


@pytest.mark.parametrize("module", ["build_supplement", "build_supplement_docx"])
def test_both_renderers_implement_the_emitter_protocol(module):
    pytest.importorskip("reportlab" if module == "build_supplement" else "docx")
    imported = __import__(module)
    emitter_cls = getattr(imported, "PdfEmitter", None) or imported.DocxEmitter
    required = [
        name for name in dir(content.Emitter)
        if not name.startswith("_")
    ]
    for name in required:
        assert callable(getattr(emitter_cls, name, None)), f"{module} lacks {name}"
