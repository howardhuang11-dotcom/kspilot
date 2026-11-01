#!/usr/bin/env bash
# build_pdf.sh: typeset the kspilot system and development report (Markdown)
# as one single-column PDF.
#
# Usage:  paper/latex/build_pdf.sh [SRC.md] [OUT.pdf]
#   SRC.md   default: paper/kspilot_report.md
#   OUT.pdf  default: paper/latex/build/kspilot_report.pdf
# Figures are looked up in SRC_DIR/figures/ (png/jpg/pdf); the vector PDF of a
# PNG of the same name is used when it exists.  The source Markdown is read
# only, never modified.
#
# Requires: pandoc >= 3.1, tectonic (XeTeX engine; fetches LaTeX packages on
# first use), poppler (pdfinfo) for the page count.  On macOS:
#   brew install pandoc tectonic poppler
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="${1:-$HERE/../kspilot_report.md}"
OUT="${2:-$HERE/build/kspilot_report.pdf}"
[[ -f "$SRC" ]] || { echo "build_pdf: source not found: $SRC" >&2; exit 1; }
SRC="$(cd "$(dirname "$SRC")" && pwd)/$(basename "$SRC")"
SRC_DIR="$(dirname "$SRC")"
mkdir -p "$(dirname "$OUT")"
OUT="$(cd "$(dirname "$OUT")" && pwd)/$(basename "$OUT")"
for t in pandoc tectonic; do command -v "$t" >/dev/null || { echo "build_pdf: $t not installed" >&2; exit 1; }; done

# Stage in a directory without spaces (paths with spaces break \includegraphics).
STAGE="$(mktemp -d "${TMPDIR:-/tmp}/rpbuild.XXXXXX")"
trap 'rm -rf "$STAGE"' EXIT
mkdir -p "$STAGE/figures"
cp "$SRC" "$STAGE/report.md"
shopt -s nullglob nocaseglob
for f in "$SRC_DIR"/figures/*.{png,jpg,jpeg,pdf}; do cp "$f" "$STAGE/figures/"; done
shopt -u nullglob nocaseglob

# Every image referenced in the Markdown must exist (no silent figure loss).
missing=0
while IFS= read -r img; do
  [[ -z "$img" ]] && continue
  [[ -f "$STAGE/$img" ]] || { echo "build_pdf: MISSING figure $img" >&2; missing=1; }
done < <(grep -oE '!\[[^]]*\]\([^)]+\)' "$STAGE/report.md" | sed -E 's/.*\(([^) ]+).*/\1/')
[[ $missing -eq 0 ]] || exit 1

LOGDIR="$HERE/build/logs"; mkdir -p "$LOGDIR"
( cd "$STAGE" && pandoc report.md \
    -f markdown+autolink_bare_uris-implicit_figures-tex_math_dollars \
    -t latex --standalone --template="$HERE/report_template.tex" \
    --lua-filter="$HERE/report_filter.lua" -o main.tex ) 2> "$LOGDIR/pandoc.log" \
  || { cat "$LOGDIR/pandoc.log" >&2; exit 1; }
grep -h "report_filter" "$LOGDIR/pandoc.log" >&2 || true
( cd "$STAGE" && nice -n 10 tectonic --keep-logs --chatter minimal main.tex ) > "$LOGDIR/tectonic.out" 2>&1 \
  || { tail -40 "$LOGDIR/tectonic.out" >&2; cp "$STAGE/main.log" "$LOGDIR/" 2>/dev/null; exit 1; }
cp "$STAGE/main.pdf" "$OUT"
cp "$STAGE/main.tex" "$LOGDIR/kspilot_report.tex"
cp "$STAGE/main.log" "$LOGDIR/kspilot_report.log"

# ---- checks -----------------------------------------------------------
md_refs=$(awk '/^## /{inref = ($0 ~ /^## References[[:space:]]*$/)} inref && /^[0-9]+\. /{c++} END{print c+0}' "$SRC")
tex_refs=$(awk '/%SNREFS-BEGIN/{r=1} /%SNREFS-END/{r=0} r && /\\item/{c++} END{print c+0}' "$LOGDIR/kspilot_report.tex")
miss_glyphs=$(grep -c "Missing character" "$LOGDIR/kspilot_report.log" || true)
overfull=$(grep -c "^Overfull \\\\hbox" "$LOGDIR/kspilot_report.log" || true)
pages=$(command -v pdfinfo >/dev/null && pdfinfo "$OUT" | awk '/^Pages:/{print $2}' || echo "?")
echo "build_pdf: $OUT: $pages pages; references md=$md_refs pdf=$tex_refs; missing glyphs=$miss_glyphs; overfull hboxes=$overfull"
status=0
[[ "$md_refs" == "$tex_refs" ]] || { echo "build_pdf: REFERENCE COUNT MISMATCH" >&2; status=2; }
[[ "$miss_glyphs" == 0 ]] || { grep "Missing character" "$LOGDIR/kspilot_report.log" | sort | uniq -c >&2; status=3; }
exit $status
