"""Build the Theorem 1 proof supplement as a Word document.

Same frozen text as the PDF: both renderers drive
:func:`supplement_content.compose`, so the two files cannot state different
things about what is proved and what is only assumed.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Pt, RGBColor, Inches

sys.path.insert(0, str(Path(__file__).resolve().parent))

import supplement_content as content  # noqa: E402


HERE = Path(__file__).resolve().parent
OUT = HERE / "DBACT_Theorem1_Proofs_and_Supporting_Results.docx"
CERT = HERE.parent / "certificates" / "analytic_constants.json"

NAVY = RGBColor(0x17, 0x32, 0x4D)
BLUE = RGBColor(0x2A, 0x6F, 0x97)
INK = RGBColor(0x17, 0x21, 0x2B)
MUTED = RGBColor(0x5C, 0x6B, 0x78)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

PALE_HEX = "EAF2F8"
EQ_HEX = "F7FAFC"
NAVY_HEX = "17324D"
ZEBRA_HEX = "F6F8FA"

SERIF = "DejaVu Serif"
SANS = "DejaVu Sans"
MONO = "DejaVu Sans Mono"


def _shade(element, hex_color: str) -> None:
    """Paint a solid background behind a paragraph or table cell."""
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_color)
    element.append(shd)


def _box(paragraph, hex_color: str = "C7D8E5") -> None:
    """Draw a thin rule on all four sides of a paragraph."""
    borders = OxmlElement("w:pBdr")
    for edge in ("top", "left", "bottom", "right"):
        el = OxmlElement(f"w:{edge}")
        el.set(qn("w:val"), "single")
        el.set(qn("w:sz"), "6")
        el.set(qn("w:space"), "6")
        el.set(qn("w:color"), hex_color)
        borders.append(el)
    paragraph._p.get_or_add_pPr().append(borders)


def _run(paragraph, text, *, font=SERIF, size=9.35, color=INK, bold=False, italic=False):
    run = paragraph.add_run(text)
    run.font.name = font
    run.font.size = Pt(size)
    run.font.color.rgb = color
    run.bold = bold
    run.italic = italic
    # Word resolves east-asian and complex-script faces separately.
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    for attr in ("w:ascii", "w:hAnsi", "w:cs"):
        rfonts.set(qn(attr), font)
    return run


class DocxEmitter:
    """Renders :func:`supplement_content.compose` into a Word document."""

    def __init__(self, document: Document):
        self.doc = document

    # -- helpers ------------------------------------------------------- #

    def _para(self, *, align=WD_ALIGN_PARAGRAPH.LEFT, before=0, after=6, left=0, hanging=None):
        p = self.doc.add_paragraph()
        p.alignment = align
        fmt = p.paragraph_format
        fmt.space_before = Pt(before)
        fmt.space_after = Pt(after)
        if left:
            fmt.left_indent = Pt(left)
        if hanging is not None:
            fmt.first_line_indent = Pt(-hanging)
        return p

    # -- emitter protocol ---------------------------------------------- #

    def title_block(self):
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, before=48, after=16)
        _run(p, content.KICKER, font=SANS, size=12, color=BLUE, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=18)
        _run(p, content.TITLE, font=SERIF, size=27, color=NAVY, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=28)
        _run(p, content.SUBTITLE, font=SERIF, size=12.5, color=MUTED)

    def section(self, title):
        p = self._para(before=4, after=10)
        _run(p, title, font=SANS, size=16, color=NAVY, bold=True)

    def subsection(self, title):
        p = self._para(before=9, after=5)
        _run(p, title, font=SANS, size=11.5, color=BLUE, bold=True)

    def lead(self, text):
        p = self._para(after=12)
        _run(p, text, font=SERIF, size=11.3, color=NAVY)

    def body(self, text):
        p = self._para(align=WD_ALIGN_PARAGRAPH.JUSTIFY, after=6)
        _run(p, text, font=SERIF, size=9.35, color=INK)

    def bullets(self, items):
        for item in items:
            p = self._para(after=3, left=15, hanging=8)
            _run(p, f"- {item}", font=SERIF, size=9.2, color=INK)

    def equation(self, text):
        p = self._para(before=4, after=8, left=12)
        _shade(p._p.get_or_add_pPr(), EQ_HEX)
        _box(p)
        lines = text.strip().split("\n")
        for index, line in enumerate(lines):
            if index:
                p.add_run().add_break()
            _run(p, line, font=MONO, size=7.6, color=INK)
        p.paragraph_format.line_spacing = Pt(10.2)

    def toc(self, items):
        for item in items:
            p = self._para(after=0, left=8)
            p.paragraph_format.line_spacing = Pt(14)
            _run(p, item, font=SANS, size=9.5, color=INK)

    def callout(self, head, text):
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=5)
        _run(p, head, font=SANS, size=8.5, color=BLUE, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=6)
        _run(p, text, font=SERIF, size=10.3, color=NAVY, bold=True)

    def table(self, kind, rows):
        spec = _TABLE_SPECS[kind]
        table = self.doc.add_table(rows=0, cols=len(rows[0]))
        table.style = "Table Grid"
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.autofit = False
        for row_index, row in enumerate(rows):
            cells = table.add_row().cells
            header = spec["header"] and row_index == 0
            for col_index, value in enumerate(row):
                cell = cells[col_index]
                cell.width = Inches(spec["widths"][col_index])
                p = cell.paragraphs[0]
                p.paragraph_format.space_before = Pt(2)
                p.paragraph_format.space_after = Pt(2)
                if spec.get("right") and col_index in spec["right"] and not header:
                    p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
                font = spec["font"]
                bold = header or (spec.get("bold_col") == col_index)
                if header:
                    font = SANS
                _run(
                    p, value,
                    font=font, size=spec["size"],
                    color=WHITE if header else spec.get("color", INK),
                    bold=bold,
                )
                fill = spec["fill"]
                if header:
                    fill = NAVY_HEX
                elif spec.get("zebra") and row_index % 2 == 0:
                    fill = ZEBRA_HEX
                if fill:
                    _shade(cell._tc.get_or_add_tcPr(), fill)
        self.doc.add_paragraph().paragraph_format.space_after = Pt(0)

    def spacer(self, points):
        p = self.doc.add_paragraph()
        p.paragraph_format.space_before = Pt(0)
        p.paragraph_format.space_after = Pt(0)
        p.add_run().font.size = Pt(max(1.0, points / 2.0))

    def pagebreak(self):
        self.doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)


_TABLE_SPECS = {
    "status": {
        "widths": [1.25, 5.25], "size": 8.2, "font": SANS, "fill": PALE_HEX,
        "header": False, "bold_col": 0, "color": INK, "zebra": False,
    },
    "limits": {
        "widths": [1.4, 1.1, 4.0], "size": 7.5, "font": SANS, "fill": None,
        "header": True, "zebra": True,
    },
    "ledger": {
        "widths": [2.15, 1.25, 1.15, 1.25], "size": 7.1, "font": MONO, "fill": None,
        "header": True, "zebra": True, "right": (1, 2),
    },
    "wolfram": {
        "widths": [1.1, 2.85, 1.15, 1.15], "size": 7.3, "font": SANS, "fill": None,
        "header": True, "zebra": True,
    },
}


def _page_furniture(document: Document) -> None:
    section = document.sections[0]
    section.left_margin = Inches(0.75)
    section.right_margin = Inches(0.75)
    section.top_margin = Inches(0.82)
    section.bottom_margin = Inches(0.72)

    # The PDF suppresses the running head on the cover page; match that.
    section.different_first_page_header_footer = True

    header = section.header
    p = header.paragraphs[0]
    p.alignment = WD_ALIGN_PARAGRAPH.LEFT
    _run(p, content.RUNNING_LEFT, font=SANS, size=7.4, color=MUTED)
    _run(p, "\t\t", font=SANS, size=7.4, color=MUTED)
    _run(p, content.RUNNING_RIGHT, font=SANS, size=7.4, color=MUTED)

    for footer in (section.footer, section.first_page_footer):
        p = footer.paragraphs[0]
        _run(p, f"Baseline HEAD {content.HEAD[:12]}", font=SANS, size=7.4, color=MUTED)


def build() -> None:
    cert = json.loads(CERT.read_text(encoding="utf-8"))
    document = Document()
    normal = document.styles["Normal"]
    normal.font.name = SERIF
    normal.font.size = Pt(9.35)

    _page_furniture(document)
    content.compose(DocxEmitter(document), cert)

    core = document.core_properties
    core.title = content.DOC_TITLE
    core.author = content.DOC_AUTHOR
    core.subject = content.DOC_SUBJECT
    core.comments = (
        "Local conditional theorem. No global PL, formal caging, finite-time phase "
        "completion, or unconditional transport/safety claim."
    )

    document.save(OUT)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    build()
