"""Build the standalone Theorem 1 proof supplement without a TeX engine.

The LaTeX source remains the canonical typeset source.  This deterministic
ReportLab route is used on hosts where latexmk/pdflatex is unavailable.

The prose itself lives in :mod:`supplement_content`, which also drives the Word
renderer, so the PDF and the DOCX cannot drift apart.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from xml.sax.saxutils import escape

from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY, TA_LEFT
from reportlab.lib.pagesizes import letter
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import inch
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (
    PageBreak,
    Paragraph,
    Preformatted,
    SimpleDocTemplate,
    Spacer,
    Table,
    TableStyle,
)

sys.path.insert(0, str(Path(__file__).resolve().parent))

import supplement_content as content  # noqa: E402


HERE = Path(__file__).resolve().parent
OUT = HERE / "DBACT_Theorem1_Proofs_and_Supporting_Results.pdf"
CERT = HERE.parent / "certificates" / "analytic_constants.json"
HEAD = content.HEAD
NAVY = colors.HexColor("#17324D")
BLUE = colors.HexColor("#2A6F97")
PALE = colors.HexColor("#EAF2F8")
INK = colors.HexColor("#17212B")
MUTED = colors.HexColor("#5C6B78")


def register_fonts() -> None:
    root = Path("C:/Windows/Fonts")
    pdfmetrics.registerFont(TTFont("DBSerif", str(root / "DejaVuSerif.ttf")))
    pdfmetrics.registerFont(TTFont("DBSerif-Bold", str(root / "DejaVuSerif-Bold.ttf")))
    pdfmetrics.registerFont(TTFont("DBSans", str(root / "DejaVuSans.ttf")))
    pdfmetrics.registerFont(TTFont("DBSans-Bold", str(root / "DejaVuSans-Bold.ttf")))
    pdfmetrics.registerFont(TTFont("DBMono", str(root / "DejaVuSansMono.ttf")))


def styles():
    base = getSampleStyleSheet()
    return {
        "body": ParagraphStyle(
            "Body", parent=base["BodyText"], fontName="DBSerif", fontSize=9.35,
            leading=13.2, textColor=INK, alignment=TA_JUSTIFY, spaceAfter=6,
        ),
        "lead": ParagraphStyle(
            "Lead", parent=base["BodyText"], fontName="DBSerif", fontSize=11.3,
            leading=16.2, textColor=NAVY, alignment=TA_LEFT, spaceAfter=12,
        ),
        "h1": ParagraphStyle(
            "H1", parent=base["Heading1"], fontName="DBSans-Bold", fontSize=16,
            leading=20, textColor=NAVY, spaceBefore=4, spaceAfter=10,
        ),
        "h2": ParagraphStyle(
            "H2", parent=base["Heading2"], fontName="DBSans-Bold", fontSize=11.5,
            leading=14, textColor=BLUE, spaceBefore=9, spaceAfter=5,
        ),
        "small": ParagraphStyle(
            "Small", parent=base["BodyText"], fontName="DBSans", fontSize=7.7,
            leading=10.5, textColor=MUTED, spaceAfter=4,
        ),
        "eq": ParagraphStyle(
            "Equation", parent=base["Code"], fontName="DBMono", fontSize=7.6,
            leading=10.2, textColor=INK, leftIndent=12, rightIndent=6,
            borderColor=colors.HexColor("#C7D8E5"), borderWidth=0.6,
            borderPadding=7, backColor=colors.HexColor("#F7FAFC"),
            spaceBefore=4, spaceAfter=8,
        ),
        "bullet": ParagraphStyle(
            "Bullet", parent=base["BodyText"], fontName="DBSerif", fontSize=9.2,
            leading=12.7, leftIndent=15, firstLineIndent=-8, textColor=INK, spaceAfter=3,
        ),
        "toc": ParagraphStyle(
            "TOC", parent=base["BodyText"], fontName="DBSans", fontSize=9.5,
            leading=14, leftIndent=8, textColor=INK,
        ),
    }


def header_footer(canvas, doc):
    canvas.saveState()
    if doc.page > 1:
        canvas.setStrokeColor(colors.HexColor("#D5DEE5"))
        canvas.setLineWidth(0.5)
        canvas.line(0.75 * inch, 10.35 * inch, 7.75 * inch, 10.35 * inch)
        canvas.setFont("DBSans", 7.4)
        canvas.setFillColor(MUTED)
        canvas.drawString(0.75 * inch, 10.48 * inch, content.RUNNING_LEFT)
        canvas.drawRightString(7.75 * inch, 10.48 * inch, content.RUNNING_RIGHT)
    canvas.setFont("DBSans", 7.4)
    canvas.setFillColor(MUTED)
    canvas.drawString(0.75 * inch, 0.48 * inch, f"Baseline HEAD {HEAD[:12]}")
    canvas.drawRightString(7.75 * inch, 0.48 * inch, f"{doc.page}")
    canvas.restoreState()


_STATUS_STYLE = TableStyle([
    ("FONTNAME", (0, 0), (-1, -1), "DBSans"),
    ("FONTNAME", (0, 0), (0, -1), "DBSans-Bold"),
    ("FONTSIZE", (0, 0), (-1, -1), 8.2),
    ("TEXTCOLOR", (0, 0), (0, -1), NAVY),
    ("BACKGROUND", (0, 0), (-1, -1), PALE),
    ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#B9CBD8")),
    ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ("LEFTPADDING", (0, 0), (-1, -1), 8),
    ("RIGHTPADDING", (0, 0), (-1, -1), 8),
    ("TOPPADDING", (0, 0), (-1, -1), 7),
    ("BOTTOMPADDING", (0, 0), (-1, -1), 7),
])

_LIMITS_STYLE = TableStyle([
    ("FONTNAME", (0, 0), (-1, 0), "DBSans-Bold"),
    ("FONTNAME", (0, 1), (-1, -1), "DBSans"),
    ("FONTSIZE", (0, 0), (-1, -1), 7.5),
    ("BACKGROUND", (0, 0), (-1, 0), NAVY),
    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
    ("GRID", (0, 0), (-1, -1), 0.35, colors.HexColor("#C4D0D8")),
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F4F7F9")]),
    ("LEFTPADDING", (0, 0), (-1, -1), 6), ("RIGHTPADDING", (0, 0), (-1, -1), 6),
    ("TOPPADDING", (0, 0), (-1, -1), 6), ("BOTTOMPADDING", (0, 0), (-1, -1), 6),
])

_LEDGER_STYLE = TableStyle([
    ("FONTNAME", (0, 0), (-1, 0), "DBSans-Bold"),
    ("FONTNAME", (0, 1), (-1, -1), "DBMono"),
    ("FONTSIZE", (0, 0), (-1, -1), 7.1),
    ("BACKGROUND", (0, 0), (-1, 0), NAVY),
    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
    ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#C8D3DA")),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F6F8FA")]),
    ("ALIGN", (1, 1), (2, -1), "RIGHT"),
    ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
    ("TOPPADDING", (0, 0), (-1, -1), 4), ("BOTTOMPADDING", (0, 0), (-1, -1), 4),
])

_WOLFRAM_STYLE = TableStyle([
    ("FONTNAME", (0, 0), (-1, 0), "DBSans-Bold"),
    ("FONTNAME", (0, 1), (-1, -1), "DBSans"),
    ("FONTSIZE", (0, 0), (-1, -1), 7.3),
    ("BACKGROUND", (0, 0), (-1, 0), NAVY),
    ("TEXTCOLOR", (0, 0), (-1, 0), colors.white),
    ("GRID", (0, 0), (-1, -1), 0.3, colors.HexColor("#C8D3DA")),
    ("VALIGN", (0, 0), (-1, -1), "TOP"),
    ("ROWBACKGROUNDS", (0, 1), (-1, -1), [colors.white, colors.HexColor("#F6F8FA")]),
    ("LEFTPADDING", (0, 0), (-1, -1), 5), ("RIGHTPADDING", (0, 0), (-1, -1), 5),
    ("TOPPADDING", (0, 0), (-1, -1), 5), ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
])

_TABLE_SPECS = {
    "status": ([1.25 * inch, 5.25 * inch], _STATUS_STYLE, 0),
    "limits": ([1.4 * inch, 1.1 * inch, 4.0 * inch], _LIMITS_STYLE, 1),
    "ledger": ([2.15 * inch, 1.25 * inch, 1.15 * inch, 1.25 * inch], _LEDGER_STYLE, 1),
    "wolfram": ([1.1 * inch, 2.85 * inch, 1.15 * inch, 1.15 * inch], _WOLFRAM_STYLE, 1),
}


class PdfEmitter:
    """Renders :func:`supplement_content.compose` into a ReportLab story."""

    def __init__(self, story, s):
        self.story = story
        self.s = s

    def title_block(self):
        self.story.append(Spacer(1, 0.5 * inch))
        self.story.append(Paragraph(content.KICKER, ParagraphStyle(
            "Kicker", fontName="DBSans-Bold", fontSize=12, leading=14,
            textColor=BLUE, alignment=TA_CENTER, spaceAfter=16,
        )))
        self.story.append(Paragraph(content.TITLE, ParagraphStyle(
            "Title", fontName="DBSerif-Bold", fontSize=27, leading=32,
            textColor=NAVY, alignment=TA_CENTER, spaceAfter=18,
        )))
        self.story.append(Paragraph(content.SUBTITLE, ParagraphStyle(
            "Subtitle", fontName="DBSerif", fontSize=12.5, leading=18,
            textColor=MUTED, alignment=TA_CENTER, spaceAfter=28,
        )))

    def section(self, title):
        self.story.append(Paragraph(title, self.s["h1"]))

    def subsection(self, title):
        self.story.append(Paragraph(title, self.s["h2"]))

    def lead(self, text):
        self.story.append(Paragraph(text, self.s["lead"]))

    def body(self, text):
        self.story.append(Paragraph(text, self.s["body"]))

    def bullets(self, items):
        for item in items:
            self.story.append(Paragraph(f"- {escape(item)}", self.s["bullet"]))

    def equation(self, text):
        self.story.append(Preformatted(text.strip(), self.s["eq"]))

    def toc(self, items):
        for item in items:
            self.story.append(Paragraph(item, self.s["toc"]))

    def callout(self, head, text):
        self.story.append(Paragraph(head, ParagraphStyle(
            "CalloutHead", fontName="DBSans-Bold", fontSize=8.5,
            textColor=BLUE, alignment=TA_CENTER, spaceAfter=5,
        )))
        self.story.append(Paragraph(text, ParagraphStyle(
            "Callout", fontName="DBSerif-Bold", fontSize=10.3,
            leading=15, textColor=NAVY, alignment=TA_CENTER,
        )))

    def table(self, kind, rows):
        widths, style, repeat = _TABLE_SPECS[kind]
        table = Table(rows, colWidths=widths, repeatRows=repeat)
        table.setStyle(style)
        self.story.append(table)

    def spacer(self, points):
        self.story.append(Spacer(1, points))

    def pagebreak(self):
        self.story.append(PageBreak())


def build() -> None:
    register_fonts()
    s = styles()
    cert = json.loads(CERT.read_text(encoding="utf-8"))
    story = []
    content.compose(PdfEmitter(story, s), cert)

    doc = SimpleDocTemplate(
        str(OUT), pagesize=letter, rightMargin=0.75 * inch, leftMargin=0.75 * inch,
        topMargin=0.82 * inch, bottomMargin=0.72 * inch,
        title=content.DOC_TITLE,
        author=content.DOC_AUTHOR,
        subject=content.DOC_SUBJECT,
    )
    doc.build(story, onFirstPage=header_footer, onLaterPages=header_footer)
    print(f"wrote {OUT}")


if __name__ == "__main__":
    build()
