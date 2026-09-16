# LaTeX compile log

`main_theorem.tex` uses only `article`, `amsmath`, `amssymb`, `geometry`.

This environment:

- `pdflatex` / `latexmk` / `tectonic` not on PATH
- conda search for tectonic hung without a package install
- a GitHub tectonic zip download did not unpack (truncated/corrupt archive; `ZipArchive` constructor failed)

This is an environment/tooling limit, not a TeX error in the source. Compile on a machine with TeX Live or MiKTeX:

```
pdflatex -interaction=nonstopmode -halt-on-error main_theorem.tex
```

Do not treat a missing PDF as a mathematical gap. The MD and TEX statements are aligned.
