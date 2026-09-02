"""Render the Theorem 1 supplement to Word, with native equation objects.

Formulas are authored once as LaTeX in :mod:`theorem1_document`.  Here they are
converted LaTeX -> MathML -> OMML, so Word receives real equation objects that
it can display, re-flow and edit -- not monospace ASCII.

The MathML->OMML step uses ``MML2OMML.XSL``, which ships with Office.  If it is
missing the build fails loudly rather than silently degrading the mathematics.
"""

from __future__ import annotations

import json
import sys
from functools import lru_cache
from pathlib import Path

import latex2mathml.converter as l2m
from docx import Document
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK, WD_TAB_ALIGNMENT
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Inches, Pt, RGBColor
from lxml import etree

sys.path.insert(0, str(Path(__file__).resolve().parent))

import theorem1_document as content  # noqa: E402


HERE = Path(__file__).resolve().parent
OUT = HERE / "DBACT_Theorem1_Proofs_and_Supporting_Results.docx"
CERT = HERE.parent / "certificates" / "analytic_constants.json"

MML2OMML_CANDIDATES = [
    Path(r"C:\Program Files\Microsoft Office\root\Office16\MML2OMML.XSL"),
    Path(r"C:\Program Files (x86)\Microsoft Office\root\Office16\MML2OMML.XSL"),
    Path(r"C:\Program Files\Microsoft Office\Office16\MML2OMML.XSL"),
    HERE / "MML2OMML.XSL",
]

NAVY = RGBColor(0x17, 0x32, 0x4D)
BLUE = RGBColor(0x2A, 0x6F, 0x97)
INK = RGBColor(0x17, 0x21, 0x2B)
MUTED = RGBColor(0x5C, 0x6B, 0x78)
WHITE = RGBColor(0xFF, 0xFF, 0xFF)

PALE_HEX = "EAF2F8"
CALLOUT_HEX = "F4F8FB"
NAVY_HEX = "17324D"
ZEBRA_HEX = "F6F8FA"
RULE_HEX = "C7D8E5"

SERIF = "Cambria"
SANS = "Segoe UI"
MATH = "Cambria Math"

BODY_PT = 10.0
TEXT_WIDTH_IN = 7.0


# --------------------------------------------------------------------- #
# LaTeX -> OMML
# --------------------------------------------------------------------- #

def _load_xslt() -> etree.XSLT:
    for candidate in MML2OMML_CANDIDATES:
        if candidate.exists():
            return etree.XSLT(etree.parse(str(candidate)))
    raise RuntimeError(
        "MML2OMML.XSL not found. It ships with Microsoft Office; without it the "
        "equations cannot be converted to Word's native format. Searched:\n  "
        + "\n  ".join(str(c) for c in MML2OMML_CANDIDATES)
    )


_XSLT = None

M = "{http://schemas.openxmlformats.org/officeDocument/2006/math}"

# latex2mathml passes \bigl\| and friends through as literal text, so the
# reader would see a backslash-pipe instead of a norm.  \left/\right convert
# correctly, and for our purposes size the same.
_BIG = ("biggl", "biggr", "bigl", "bigr", "Biggl", "Biggr", "Bigl", "Bigr",
        "bigg", "Bigg", "big", "Big")
_ESCAPED_DELIMS = ("\\|", "\\{", "\\}", "\\langle", "\\rangle", "\\lVert", "\\rVert")

# MathML <mover> becomes OMML m:limUpp -- a limit, drawn high and detached.
# An accent belongs in m:acc with the matching combining character.
_ACCENTS = {
    "^": "̂", "ˆ": "̂",   # circumflex / hat
    "˙": "̇", "̇": "̇",   # dot above
    "¯": "̄", "ˉ": "̄", "‾": "̄",  # macron / bar
    "~": "̃", "˜": "̃",   # tilde
    "⃗": "⃗", "→": "⃗",   # vector arrow
    "¨": "̈",                        # diaeresis / double dot
}

# Function names must be upright; Cambria Math italicises bare runs.
_UPRIGHT = {
    "min", "max", "sup", "inf", "lim", "limsup", "liminf", "dist", "exp",
    "log", "ln", "sin", "cos", "tan", "diag", "det", "tr", "arg", "Lip",
    "Vor", "dim", "span", "supp", "sgn",
}


def _normalize_latex(latex: str) -> str:
    """Rewrite the constructs the converter chain mishandles.

    Each substitution here stands for a defect verified against the real
    toolchain, not a stylistic preference:

    * ``\\bigl\\|`` and friends survive as literal backslash-pipe text;
      plain ``\\|`` converts correctly and sizes the same at our sizes.
    * a bare ``\\nabla`` directly inside a fence is silently swallowed by
      MML2OMML; wrapping it in a group keeps it.
    * ``\\qquad``/``\\quad``/``\\;`` are dropped outright, so equations run
      together. Only the escaped space survives.
    """
    out = latex
    for size in _BIG:
        for delim in _ESCAPED_DELIMS:
            out = out.replace("\\" + size + delim, delim)
    out = out.replace("\\left\\|", "\\|").replace("\\right\\|", "\\|")
    out = out.replace("{\\nabla}", "\\nabla").replace("\\nabla", "{\\nabla}")
    # A bare division slash is dropped outright by MML2OMML, turning
    # pi^{3/2} into pi^32.  Grouping it keeps it.
    out = out.replace("{/}", "/").replace("/", "{/}")
    out = _brace_fences(out)
    out = _brace_norms(out)
    out = out.replace("\\qquad", "\\ \\ \\ \\ ")
    out = out.replace("\\quad", "\\ \\ ")
    out = out.replace("\\;", "\\ ")
    return out


_DELIM_TOKENS = ("\\langle", "\\rangle", "\\lVert", "\\rVert", "\\lfloor",
                 "\\rfloor", "\\lceil", "\\rceil", "\\{", "\\}", "\\|", "\\.")


def _read_delim(latex: str, i: int) -> tuple[str, int]:
    for token in _DELIM_TOKENS:
        if latex.startswith(token, i):
            return token, i + len(token)
    if i < len(latex):
        return latex[i], i + 1
    return "", i


def _brace_fences(latex: str) -> str:
    """Group the body of every ``\\left...\\right`` pair.

    MML2OMML renders a fence by emitting each child as its own ``m:e`` and
    discards the operators between them, so ``\\left(a+b+c\\right)`` arrives as
    ``(abc)``.  Giving the fence a single grouped child avoids that entirely.
    """
    out = []
    i = 0
    n = len(latex)
    while i < n:
        if latex.startswith("\\left", i):
            delim, j = _read_delim(latex, i + 5)
            depth, k = 0, j
            while k < n:
                if latex.startswith("\\left", k):
                    depth += 1
                    k += 5
                elif latex.startswith("\\right", k):
                    if depth == 0:
                        break
                    depth -= 1
                    k += 6
                else:
                    k += 1
            if k >= n:                      # unmatched: leave untouched
                out.append(latex[i])
                i += 1
                continue
            body = _brace_fences(latex[j:k])
            if not (body.startswith("{") and body.endswith("}")):
                body = "{" + body + "}"
            close, after = _read_delim(latex, k + 6)
            out.append("\\left" + delim + body + "\\right" + close)
            i = after
        else:
            out.append(latex[i])
            i += 1
    return "".join(out)


def _brace_norms(latex: str) -> str:
    """Group the contents of every ``\\|...\\|`` pair.

    MML2OMML walks a fence by consuming siblings, and drops what it does not
    recognise -- so ``\\|d\\|`` loses the ``d`` entirely.  Wrapping the content
    in a group gives the fence a single child to carry, which survives.
    """
    parts = latex.split("\\|")
    if len(parts) < 3 or len(parts) % 2 == 0:
        return latex          # no pair, or an unmatched bar: leave it alone
    out = [parts[0]]
    for i in range(1, len(parts), 2):
        body = parts[i]
        if not (body.startswith("{") and body.endswith("}")):
            body = "{" + body + "}"
        out.append("\\|" + body + "\\|")
        if i + 1 < len(parts):
            out.append(parts[i + 1])
    return "".join(out)


_IGNORABLE = {
    "⁡", "⁢", "⁣", "⁤",  # invisible function/times/separator
    "​", " ", " ", " ", "−",
}


# The several spellings of an overbar collapse to one, so that MathML's glyph
# and OMML's structural m:bar compare equal.
_BAR_FORMS = str.maketrans({"‾": "¯", "ˉ": "¯", "̄": "¯"})


def _significant(text: str) -> str:
    text = text.translate(_BAR_FORMS)
    return "".join(c for c in text if not c.isspace() and c not in _IGNORABLE)


def _mathml_payload(mathml: str) -> str:
    root = etree.fromstring(mathml.encode("utf-8"))
    ml = "{http://www.w3.org/1998/Math/MathML}"
    parts = []
    for el in root.iter():
        if el.tag in (ml + "mi", ml + "mn", ml + "mo", ml + "mtext"):
            parts.append(el.text or "")
    return _significant("".join(parts))


def _omml_payload(root) -> str:
    parts = []
    for el in root.iter():
        if el.tag == M + "t":
            parts.append(el.text or "")
        elif el.tag in (M + "begChr", M + "endChr", M + "chr", M + "sepChr"):
            parts.append(el.get(M + "val") or "")
        elif el.tag == M + "d":
            # A delimiter with no dPr means Word's default, "(" and ")".
            pr = el.find(M + "dPr")
            if pr is None or pr.find(M + "begChr") is None:
                parts.append("(")
            if pr is None or pr.find(M + "endChr") is None:
                parts.append(")")
        elif el.tag == M + "rad":
            # Radicals carry no glyph of their own; MathML spells out the sign.
            parts.append("√")
        elif el.tag == M + "bar":
            # OMML draws an overbar structurally, MathML uses a glyph.
            parts.append("¯")
    return _significant("".join(parts))


def _fix_accents(root) -> None:
    """Rewrite ``m:limUpp`` carrying a single accent glyph into ``m:acc``."""
    for lim_upp in list(root.iter(M + "limUpp")):
        lim = lim_upp.find(M + "lim")
        base = lim_upp.find(M + "e")
        if lim is None or base is None:
            continue
        text = "".join(t.text or "" for t in lim.iter(M + "t")).strip()
        combining = _ACCENTS.get(text)
        if combining is None:
            continue
        acc = etree.SubElement(lim_upp.getparent(), M + "acc")
        acc_pr = etree.SubElement(acc, M + "accPr")
        chr_el = etree.SubElement(acc_pr, M + "chr")
        chr_el.set(M + "val", combining)
        acc.append(base)
        lim_upp.getparent().replace(lim_upp, acc)


def _grow_delimiters(root) -> None:
    """Let fences stretch to their content (norms around an integral, say)."""
    for d in root.iter(M + "d"):
        pr = d.find(M + "dPr")
        if pr is None:
            pr = etree.Element(M + "dPr")
            d.insert(0, pr)
        if pr.find(M + "grow") is None:
            grow = etree.Element(M + "grow")
            pr.insert(0, grow)


def _mark_upright(run) -> None:
    r_pr = run.find(M + "rPr")
    if r_pr is None:
        r_pr = etree.Element(M + "rPr")
        run.insert(0, r_pr)
    if r_pr.find(M + "sty") is None:
        sty = etree.SubElement(r_pr, M + "sty")
        sty.set(M + "val", "p")


def _upright_operators(root) -> None:
    """Set function names upright; Cambria Math italicises bare runs."""
    for t in list(root.iter(M + "t")):
        text = t.text or ""
        # "lim sup" arrives as lim + thin space + sup.
        squashed = "".join(text.split()).replace(" ", "").replace(" ", "")
        run = t.getparent()
        if run is None or run.tag != M + "r":
            continue
        if squashed in _UPRIGHT:
            _mark_upright(run)
            continue
        # MML2OMML merges "dist(P," into one run, so split off the name.
        head = ""
        for i, ch in enumerate(text):
            if not ch.isalpha():
                head = text[:i]
                break
        if head and head in _UPRIGHT and len(head) < len(text):
            t.text = head
            _mark_upright(run)
            tail_run = etree.Element(M + "r")
            tail_t = etree.SubElement(tail_run, M + "t")
            tail_t.set("{http://www.w3.org/XML/1998/namespace}space", "preserve")
            tail_t.text = text[len(head):]
            run.addnext(tail_run)


@lru_cache(maxsize=None)
def latex_to_omml(latex: str) -> bytes:
    """Convert one LaTeX fragment to an OMML ``m:oMath`` element, serialised."""
    global _XSLT
    if _XSLT is None:
        _XSLT = _load_xslt()
    source = _normalize_latex(latex)
    try:
        mathml = l2m.convert(source)
    except Exception as exc:  # pragma: no cover - authoring error
        raise ValueError(f"latex2mathml failed on: {latex!r}") from exc
    root = _XSLT(etree.fromstring(mathml.encode("utf-8"))).getroot()

    # MML2OMML can drop symbols silently (a bare \nabla inside a fence is the
    # case that prompted this).  A formula that loses a gradient operator still
    # renders, and still looks plausible, which is exactly why it must fail the
    # build rather than reach a reader.
    lost = _significant_diff(_mathml_payload(mathml), _omml_payload(root))
    if lost:
        raise ValueError(
            f"MathML->OMML dropped {lost!r} from: {latex!r}\n"
            f"  add a normalisation rule in _normalize_latex for this construct."
        )

    _fix_accents(root)
    _upright_operators(root)
    _grow_delimiters(root)

    leaked = [t.text for t in root.iter(M + "t") if t.text and "\\" in t.text]
    if leaked:
        raise ValueError(
            f"unconverted LaTeX leaked into the document: {leaked!r}\n  in: {latex!r}"
        )
    return etree.tostring(root)


def _significant_diff(before: str, after: str) -> str:
    """Characters present in the MathML that did not survive into the OMML."""
    from collections import Counter
    missing = Counter(before) - Counter(after)
    return "".join(sorted(missing.elements()))


def _append_math(paragraph, latex: str, *, display: bool = False) -> None:
    node = etree.fromstring(latex_to_omml(latex))
    if display:
        # Wrap in m:oMathPara so Word treats it as a display equation.
        wrapper = OxmlElement("m:oMathPara")
        wrapper.append(node)
        paragraph._p.append(wrapper)
    else:
        paragraph._p.append(node)


# --------------------------------------------------------------------- #
# rich text: $math$ and \emph{...}
# --------------------------------------------------------------------- #

def tokenize(text: str) -> list[tuple[str, str]]:
    """Split body text into ``("text"|"math"|"emph", payload)`` segments."""
    tokens: list[tuple[str, str]] = []
    buf: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        if ch == "$":
            end = text.find("$", i + 1)
            if end < 0:
                buf.append(ch)
                i += 1
                continue
            if buf:
                tokens.append(("text", "".join(buf)))
                buf = []
            tokens.append(("math", text[i + 1:end]))
            i = end + 1
        elif text.startswith("\\emph{", i):
            end = text.find("}", i)
            if end < 0:
                buf.append(ch)
                i += 1
                continue
            if buf:
                tokens.append(("text", "".join(buf)))
                buf = []
            tokens.append(("emph", text[i + 6:end]))
            i = end + 1
        else:
            buf.append(ch)
            i += 1
    if buf:
        tokens.append(("text", "".join(buf)))
    return tokens


# --------------------------------------------------------------------- #
# low-level docx helpers
# --------------------------------------------------------------------- #

def _shade(element, hex_color: str) -> None:
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hex_color)
    element.append(shd)


def _left_rule(paragraph, hex_color: str = "2A6F97", width: str = "18") -> None:
    borders = OxmlElement("w:pBdr")
    el = OxmlElement("w:left")
    el.set(qn("w:val"), "single")
    el.set(qn("w:sz"), width)
    el.set(qn("w:space"), "10")
    el.set(qn("w:color"), hex_color)
    borders.append(el)
    paragraph._p.get_or_add_pPr().append(borders)


def _keep_with_next(paragraph) -> None:
    ppr = paragraph._p.get_or_add_pPr()
    ppr.append(OxmlElement("w:keepNext"))


def _no_table_borders(table) -> None:
    borders = OxmlElement("w:tblBorders")
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        el = OxmlElement(f"w:{edge}")
        el.set(qn("w:val"), "none")
        el.set(qn("w:sz"), "0")
        borders.append(el)
    table._tbl.tblPr.append(borders)


def _run(paragraph, text, *, font=SERIF, size=BODY_PT, color=INK,
         bold=False, italic=False):
    run = paragraph.add_run(text)
    run.font.size = Pt(size)
    run.font.color.rgb = color
    run.bold = bold
    run.italic = italic
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.insert(0, rfonts)
    for attr in ("w:ascii", "w:hAnsi", "w:cs"):
        rfonts.set(qn(attr), font)
    return run


# --------------------------------------------------------------------- #
# renderer
# --------------------------------------------------------------------- #

_TABLE_SPECS = {
    "status": {"widths": [1.3, 5.2], "size": 8.6, "header": False,
               "fill": PALE_HEX, "bold_col": 0, "zebra": False},
    "assumptions": {"widths": [0.45, 3.85, 2.2], "size": 8.0, "header": True,
                    "fill": None, "zebra": True, "bold_col": 0},
    "limits": {"widths": [1.4, 1.25, 4.05], "size": 8.2, "header": True,
               "fill": None, "zebra": True},
    "ledger": {"widths": [2.2, 1.3, 1.2, 1.3], "size": 8.0, "header": True,
               "fill": None, "zebra": True, "right": (1, 2), "mono_col": 0},
    "wolfram": {"widths": [1.15, 2.9, 1.2, 1.25], "size": 8.0, "header": True,
                "fill": None, "zebra": True},
}


class DocxRenderer:
    def __init__(self, document: Document):
        self.doc = document

    # -- paragraph plumbing ------------------------------------------- #

    def _para(self, *, align=WD_ALIGN_PARAGRAPH.LEFT, before=0, after=6,
              left=0, hanging=None):
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

    def _rich(self, paragraph, text, *, size=BODY_PT, color=INK,
              font=SERIF, bold=False):
        for kind, payload in tokenize(text):
            if kind == "math":
                _append_math(paragraph, payload)
            elif kind == "emph":
                _run(paragraph, payload, font=font, size=size,
                     color=color, bold=bold, italic=True)
            else:
                _run(paragraph, payload, font=font, size=size,
                     color=color, bold=bold)

    # -- block handlers ------------------------------------------------ #

    def title_block(self):
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, before=54, after=14)
        _run(p, content.KICKER, font=SANS, size=12, color=BLUE, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=16)
        _run(p, content.TITLE, font=SERIF, size=28, color=NAVY, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=26)
        _run(p, content.SUBTITLE, font=SERIF, size=12, color=MUTED)
        self.table("status", content.STATUS_ROWS)
        self._para(after=0).add_run().font.size = Pt(10)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, before=14, after=5)
        _run(p, content.CALLOUT_HEAD, font=SANS, size=8.5, color=BLUE, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, after=6)
        _run(p, content.CALLOUT_BODY, font=SERIF, size=10.5, color=NAVY, bold=True)

    def h1(self, text):
        p = self._para(before=6, after=10)
        _keep_with_next(p)
        self._rich(p, text, size=16, color=NAVY, font=SANS, bold=True)

    def h2(self, text):
        p = self._para(before=11, after=5)
        _keep_with_next(p)
        self._rich(p, text, size=11.5, color=BLUE, font=SANS, bold=True)

    def h3(self, text):
        p = self._para(before=9, after=4)
        _keep_with_next(p)
        self._rich(p, text, size=10.2, color=NAVY, font=SANS, bold=True)

    def para(self, text):
        p = self._para(align=WD_ALIGN_PARAGRAPH.JUSTIFY, after=7)
        p.paragraph_format.line_spacing = 1.12
        self._rich(p, text)

    def eq(self, latex, tag=None):
        if tag is None:
            p = self._para(align=WD_ALIGN_PARAGRAPH.CENTER, before=7, after=9)
            _append_math(p, latex, display=True)
            return
        # A tagged equation goes in a borderless two-column row: the maths keeps
        # its display style (inline style shrinks fractions and radicals to
        # script size), and the tag sits hard right on the same baseline.
        table = self.doc.add_table(rows=1, cols=2)
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.autofit = False
        _no_table_borders(table)
        math_cell, tag_cell = table.rows[0].cells
        math_cell.width = Inches(6.2)
        tag_cell.width = Inches(0.8)
        p = math_cell.paragraphs[0]
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_before = Pt(5)
        p.paragraph_format.space_after = Pt(5)
        _append_math(p, latex, display=True)
        q = tag_cell.paragraphs[0]
        q.alignment = WD_ALIGN_PARAGRAPH.RIGHT
        q.paragraph_format.space_before = Pt(9)
        _run(q, f"({tag})", font=SANS, size=9, color=MUTED)
        self._para(after=3).add_run().font.size = Pt(3)

    def chain(self, lines):
        """Aligned derivation: maths left, justification right, no borders."""
        table = self.doc.add_table(rows=0, cols=2)
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.autofit = False
        _no_table_borders(table)
        for latex, why in lines:
            cells = table.add_row().cells
            cells[0].width = Inches(4.55)
            cells[1].width = Inches(2.45)
            left = cells[0].paragraphs[0]
            left.paragraph_format.space_before = Pt(3)
            left.paragraph_format.space_after = Pt(3)
            _append_math(left, latex)
            right = cells[1].paragraphs[0]
            right.alignment = WD_ALIGN_PARAGRAPH.RIGHT
            right.paragraph_format.space_before = Pt(4)
            right.paragraph_format.space_after = Pt(3)
            self._rich(right, why, size=8.4, color=MUTED)
        self._para(after=2).add_run().font.size = Pt(2)

    def bullets(self, items):
        for item in items:
            p = self._para(align=WD_ALIGN_PARAGRAPH.JUSTIFY, after=4,
                           left=16, hanging=10)
            p.paragraph_format.line_spacing = 1.1
            _run(p, "\u2022  ", font=SERIF)
            self._rich(p, item)

    def callout(self, head, text):
        p = self._para(before=9, after=3, left=12)
        _shade(p._p.get_or_add_pPr(), CALLOUT_HEX)
        _left_rule(p)
        _keep_with_next(p)
        _run(p, head, font=SANS, size=8.4, color=BLUE, bold=True)
        p = self._para(align=WD_ALIGN_PARAGRAPH.JUSTIFY, after=9, left=12)
        _shade(p._p.get_or_add_pPr(), CALLOUT_HEX)
        _left_rule(p)
        self._rich(p, text, size=9.3)

    def table(self, kind, rows):
        spec = _TABLE_SPECS[kind]
        table = self.doc.add_table(rows=0, cols=len(rows[0]))
        table.style = "Table Grid"
        table.alignment = WD_TABLE_ALIGNMENT.CENTER
        table.autofit = False
        for row_index, row in enumerate(rows):
            cells = table.add_row().cells
            header = spec["header"] and row_index == 0
            if header:
                cells[0]._tc.getparent().append(self._repeat_header())
            for col_index, value in enumerate(row):
                cell = cells[col_index]
                cell.width = Inches(spec["widths"][col_index])
                p = cell.paragraphs[0]
                p.paragraph_format.space_before = Pt(3)
                p.paragraph_format.space_after = Pt(3)
                if spec.get("right") and col_index in spec["right"] and not header:
                    p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
                bold = header or (spec.get("bold_col") == col_index)
                font = SANS if (header or kind != "ledger") else SERIF
                if not header and spec.get("mono_col") == col_index:
                    font = "Consolas"
                if not header and kind == "ledger":
                    font = "Consolas"
                self._rich(p, value, size=spec["size"],
                           color=WHITE if header else INK,
                           font=font, bold=bold)
                fill = spec["fill"]
                if header:
                    fill = NAVY_HEX
                elif spec.get("zebra") and row_index % 2 == 0:
                    fill = ZEBRA_HEX
                if fill:
                    _shade(cell._tc.get_or_add_tcPr(), fill)
        self._para(after=4).add_run().font.size = Pt(4)

    @staticmethod
    def _repeat_header():
        pr = OxmlElement("w:trPr")
        pr.append(OxmlElement("w:tblHeader"))
        return pr

    def pagebreak(self):
        self.doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

    # -- driver -------------------------------------------------------- #

    def render(self, blocks):
        for block in blocks:
            kind, args = block[0], block[1:]
            if kind == "pagebreak":
                self.pagebreak()
            elif kind == "eq":
                self.eq(args[0], args[1] if len(args) > 1 else None)
            else:
                getattr(self, kind)(*args)


def _page_furniture(document: Document) -> None:
    section = document.sections[0]
    section.left_margin = Inches(0.75)
    section.right_margin = Inches(0.75)
    section.top_margin = Inches(0.85)
    section.bottom_margin = Inches(0.75)
    section.different_first_page_header_footer = True

    p = section.header.paragraphs[0]
    fmt = p.paragraph_format
    fmt.tab_stops.add_tab_stop(Inches(TEXT_WIDTH_IN), WD_TAB_ALIGNMENT.RIGHT)
    _run(p, content.RUNNING_LEFT, font=SANS, size=7.6, color=MUTED)
    _run(p, "\t", font=SANS, size=7.6, color=MUTED)
    _run(p, content.RUNNING_RIGHT, font=SANS, size=7.6, color=MUTED)

    for footer in (section.footer, section.first_page_footer):
        p = footer.paragraphs[0]
        fmt = p.paragraph_format
        fmt.tab_stops.add_tab_stop(Inches(TEXT_WIDTH_IN), WD_TAB_ALIGNMENT.RIGHT)
        _run(p, f"Baseline HEAD {content.HEAD[:12]}", font=SANS, size=7.6, color=MUTED)
        _run(p, "\t", font=SANS, size=7.6, color=MUTED)
        _add_page_number(p)


def _add_page_number(paragraph) -> None:
    run = paragraph.add_run()
    rpr = run._element.get_or_add_rPr()
    rfonts = OxmlElement("w:rFonts")
    for attr in ("w:ascii", "w:hAnsi"):
        rfonts.set(qn(attr), SANS)
    rpr.insert(0, rfonts)
    sz = OxmlElement("w:sz")
    sz.set(qn("w:val"), "15")
    rpr.append(sz)
    for kind, text in (("begin", None), (None, "PAGE"), ("end", None)):
        if kind:
            fld = OxmlElement("w:fldChar")
            fld.set(qn("w:fldCharType"), kind)
            run._element.append(fld)
        else:
            instr = OxmlElement("w:instrText")
            instr.set(qn("xml:space"), "preserve")
            instr.text = f" {text} "
            run._element.append(instr)


def build() -> None:
    cert = json.loads(CERT.read_text(encoding="utf-8"))
    document = Document()

    normal = document.styles["Normal"]
    normal.font.name = SERIF
    normal.font.size = Pt(BODY_PT)
    # Word picks the math font from this style; without it equations render in
    # the body serif and stop looking like mathematics.
    try:
        document.styles["Normal"].element.rPr.rFonts.set(qn("w:eastAsia"), MATH)
    except AttributeError:
        pass

    _page_furniture(document)
    renderer = DocxRenderer(document)
    renderer.title_block()
    document.add_paragraph().add_run().add_break(WD_BREAK.PAGE)
    renderer.render(content.document(cert))

    core = document.core_properties
    core.title = content.DOC_TITLE
    core.author = content.DOC_AUTHOR
    core.subject = content.DOC_SUBJECT
    core.comments = (
        "Local conditional theorem. No global PL, formal caging, finite-time "
        "phase completion, or unconditional transport/safety claim."
    )

    document.save(OUT)
    print(f"wrote {OUT}  ({len(latex_to_omml.cache_info().__repr__()) and ''}"
          f"{latex_to_omml.cache_info().currsize} distinct equations)")


if __name__ == "__main__":
    build()
