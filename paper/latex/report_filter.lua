-- report_filter.lua: pandoc Lua filter that maps the kspilot report Markdown
-- (paper/kspilot_report.md conventions) onto a plain single-column article.
--
-- Markdown conventions handled (the source file is never modified):
--   # Title; author/affiliation paragraphs up to the first ---  -> title block
--   <sup>..</sup>, <sub>..</sub>    -> \textsuperscript / \textsubscript
--   Unicode superscripts (s^-1), U+2009 -> \, ;  `code` -> \texttt with break points
--   ## References + numbered list   -> small numbered list
--   image paragraph + "**Fig. N | ..**" legend paragraph -> figure block
--   "**Table N | ..**" + pipe table + notes paragraph     -> table block
--   Headings are demoted by one level (## -> \section); numbers are written in the headings.
-- Diagnostics go to stderr, prefixed "report_filter:".

local stringify = pandoc.utils.stringify

local function warn(msg) io.stderr:write("report_filter: " .. msg .. "\n") end

------------------------------------------------------------------------
-- Pass 1: inline normalisation
------------------------------------------------------------------------
local SUPMAP = {
  ["⁰"]="0", ["¹"]="1", ["²"]="2", ["³"]="3", ["⁴"]="4", ["⁵"]="5",
  ["⁶"]="6", ["⁷"]="7", ["⁸"]="8", ["⁹"]="9", ["⁻"]="−", ["⁺"]="+",
}

-- symbols missing from STIX Two Text: typeset with STIX Two Math (\snsym, see template)
local MATHSYM = { [0x2264]=true, [0x2265]=true, [0x2248]=true, [0x2260]=true, [0x2192]=true,
  [0x221E]=true, [0x221D]=true, [0x223C]=true, [0x2213]=true, [0x2032]=true, [0x2033]=true,
  [0x2202]=true }   -- r2: partial derivative (Methods, predictor-corrector cutoff)

local function split_str(text)
  -- returns list of inlines, or nil if nothing to change
  local out, buf, sup = {}, {}, {}
  local changed = false
  local function flush_buf()
    if #buf > 0 then out[#out+1] = pandoc.Str(table.concat(buf)); buf = {} end
  end
  local function flush_sup()
    if #sup > 0 then out[#out+1] = pandoc.Superscript({pandoc.Str(table.concat(sup))}); sup = {} end
  end
  for _, cp in utf8.codes(text) do
    local ch = utf8.char(cp)
    if SUPMAP[ch] then
      flush_buf(); sup[#sup+1] = SUPMAP[ch]; changed = true
    elseif MATHSYM[cp] then
      flush_sup(); flush_buf(); out[#out+1] = pandoc.RawInline("latex", "{\\snsym " .. ch .. "}"); changed = true
    elseif cp == 0x2009 or cp == 0x202F then
      flush_sup(); flush_buf(); out[#out+1] = pandoc.RawInline("latex", "\\,"); changed = true
    else
      flush_sup(); buf[#buf+1] = ch
    end
  end
  flush_sup(); flush_buf()
  if changed then return out end
  return nil
end

local function tex_escape_code(s)
  local map = {
    ["\\"]="\\textbackslash{}", ["{"]="\\{", ["}"]="\\}", ["$"]="\\$",
    ["&"]="\\&", ["#"]="\\#", ["^"]="\\^{}", ["_"]="\\_\\allowbreak{}",
    ["%"]="\\%", ["~"]="\\textasciitilde{}", ["/"]="/\\allowbreak{}",
  }
  return (s:gsub("[\\{}$&#^_%%~/]", map))
end

local function merge_html(inlines)
  local out, i, n = pandoc.List(), 1, #inlines
  while i <= n do
    local el = inlines[i]
    local tag = (el.t == "RawInline" and el.format == "html") and el.text:match("^<(%a+)>$")
    if tag == "sup" or tag == "sub" then
      local j, inner, closed = i + 1, pandoc.List(), false
      while j <= n do
        local e = inlines[j]
        if e.t == "RawInline" and e.format == "html" and e.text == "</" .. tag .. ">" then
          closed = true; break
        end
        inner:insert(e); j = j + 1
      end
      if closed then
        out:insert(tag == "sup" and pandoc.Superscript(inner) or pandoc.Subscript(inner))
        i = j + 1
      else
        warn("unclosed <" .. tag .. ">"); out:insert(el); i = i + 1
      end
    else
      if el.t == "RawInline" and el.format == "html" then
        warn("raw HTML inline dropped by LaTeX: " .. el.text)
      end
      out:insert(el); i = i + 1
    end
  end
  return out
end

local pass1 = {
  traverse = "topdown",
  Inlines = function(inlines)
    inlines = merge_html(inlines)
    local out = pandoc.List()
    for _, el in ipairs(inlines) do
      if el.t == "Str" then
        local parts = split_str(el.text)
        if parts then out:extend(parts) else out:insert(el) end
      elseif el.t == "Code" then
        out:insert(pandoc.RawInline("latex", "\\texttt{" .. tex_escape_code(el.text) .. "}"))
      else
        out:insert(el)
      end
    end
    return out
  end,
  RawBlock = function(el)
    if el.format == "html" then warn("raw HTML block dropped: " .. el.text:sub(1, 60)) end
  end,
}

------------------------------------------------------------------------
-- Pass 2: table column widths (proportional to content)
------------------------------------------------------------------------
local function cell_len(cell) return utf8.len(stringify(cell.contents)) or 0 end
local function longest_word(s)
  local m = 0
  for w in s:gmatch("%S+") do m = math.max(m, utf8.len(w) or #w) end
  return m
end

-- Table style by column count: font size (pt), leading (pt), \tabcolsep (pt).
local function table_style(ncol)
  if ncol >= 9 then return 6.5, 7.8, 2.0 end
  if ncol >= 6 then return 7.0, 8.4, 3.0 end
  return 7.5, 9.0, 4.0
end
-- text widths (pt): sn-jnl single-column 31pc; iicol 160 mm full width, column = (160 mm - 8 mm)/2
local WIDTH_1COL, WIDTH_FULL2, WIDTH_COL2 = 453, 453.5, 215.4
TWOCOL = false   -- set from metadata (pandoc -M twocol=true)

local function measure_table(tbl)
  local ncol = #tbl.colspecs
  local minw, pref = {}, {}
  for c = 1, ncol do minw[c] = 2; pref[c] = 2 end
  local function visit(rows, is_head)
    for _, row in ipairs(rows) do
      local c = 1
      for _, cell in ipairs(row.cells) do
        if c <= ncol then
          local txt = stringify(cell.contents)
          local code = false
          pandoc.walk_block(pandoc.Div(cell.contents), {Code = function() code = true end})
          local k = code and 1.2 or 1.0
          local lw = longest_word(txt) * k
          if code then  -- code may break after "_" "/": longest unbreakable piece
            lw = 0
            for piece in txt:gmatch("[^%s_/]+[_/]?") do lw = math.max(lw, (utf8.len(piece) or #piece) * k) end
          end
          local L = (utf8.len(txt) or #txt) * k
          if not is_head and L <= 11 then lw = L end   -- short values ("110 × 110") stay on one line
          minw[c] = math.max(minw[c], lw + 0.5)
          if is_head then L = math.min(L, math.max(lw, 12)) end
          pref[c] = math.max(pref[c], math.min(L, 60) + 0.5)
        end
        c = c + (cell.col_span or 1)
      end
    end
  end
  visit(tbl.head.rows, true)
  for _, body in ipairs(tbl.bodies) do visit(body.body, false) end
  local smin, spref = 0, 0
  for c = 1, ncol do pref[c] = math.max(pref[c], minw[c]); smin = smin + minw[c]; spref = spref + pref[c] end
  return minw, pref, smin, spref
end

local function avail_chars(width, ncol, sep, fs) return (width - 2 * ncol * sep) / (fs * 0.5) end

-- set column widths for a table typeset in a block `width` pt wide
local function size_table(tbl, width, place)
  local ncol = #tbl.colspecs
  local fs, lead, sep = table_style(ncol)
  local minw, pref, smin, spref = measure_table(tbl)
  local avail = avail_chars(width, ncol, sep, fs)
  while smin > avail and fs > 5.5 do      -- too wide: shrink type and padding
    fs = fs - 0.5; lead = fs * 1.2; sep = math.max(sep - 0.5, 1.5)
    avail = avail_chars(width, ncol, sep, fs)
  end
  tbl.attr.attributes["snfs"] = string.format("%.1f,%.1f,%.1f", fs, lead, sep)
  tbl.attr.attributes["snplace"] = place
  local w = {}
  if spref <= avail then
    for c = 1, ncol do w[c] = pref[c] / avail end            -- natural width (may be < full)
  else
    local extra = math.max(avail - smin, 0)
    for c = 1, ncol do
      w[c] = minw[c] + (spref > smin and extra * (pref[c] - minw[c]) / (spref - smin) or 0)
    end
    local tot = 0
    for c = 1, ncol do tot = tot + w[c] end
    for c = 1, ncol do w[c] = w[c] / tot end
    if smin > avail then warn(string.format("table with %d columns is wider than the text block (%.0f > %.0f chars)", ncol, smin, avail)) end
  end
  for c = 1, ncol do tbl.colspecs[c] = {pandoc.AlignLeft, w[c]} end
end

local pass0 = {
  Meta = function(m)
    local v = m.twocol
    TWOCOL = (v == true) or (v ~= nil and stringify(v) == "true")
  end,
}

local pass2 = {
  Pandoc = function(doc)
    for _, b in ipairs(doc.blocks) do
      if b.t == "Table" then
        if not TWOCOL then
          size_table(b, WIDTH_1COL, "inline")
        else
          -- narrow tables float in one column, the rest span both columns
          local ncol = #b.colspecs
          local fs, _, sep = table_style(ncol)
          local _, _, smin, spref = measure_table(b)
          if spref <= avail_chars(WIDTH_COL2, ncol, sep, fs) then
            size_table(b, WIDTH_COL2, "column")
          else
            size_table(b, WIDTH_FULL2, "span")
          end
        end
      end
    end
    return doc
  end,
}

------------------------------------------------------------------------
-- Pass 3: document structure
------------------------------------------------------------------------
local function first_strong_text(blk)
  if not blk or blk.t ~= "Para" or #blk.content == 0 then return nil end
  local f = blk.content[1]
  if f.t == "Strong" then return stringify(f) end
  return nil
end
local function is_fig_legend(blk)
  local s = first_strong_text(blk)
  return s and s:match("^Fig%.") and true or false
end
local function is_table_title(blk)
  local s = first_strong_text(blk)
  return s and s:match("^Table ") and true or false
end
local function image_of(blk)
  if not blk or (blk.t ~= "Para" and blk.t ~= "Plain") then return nil end
  local img
  for _, e in ipairs(blk.content) do
    if e.t == "Image" then
      if img then return nil end
      img = e
    elseif e.t ~= "Space" and e.t ~= "SoftBreak" and e.t ~= "LineBreak" then
      return nil
    end
  end
  return img
end
local function raw(s) return pandoc.RawBlock("latex", s) end
-- the Markdown references PNGs (readable on the web); typeset the vector PDF of the same name when it exists
local function graphic_src(src)
  local pdf = src:gsub("%.[Pp][Nn][Gg]$", ".pdf")
  if pdf ~= src then
    local f = io.open(pdf, "r")
    if f then f:close(); return pdf end
  end
  return src
end
local function to_latex_inlines(inlines)
  local s = pandoc.write(pandoc.Pandoc({pandoc.Plain(inlines)}), "latex")
  return (s:gsub("%s+$", ""))
end
local cited = {}
local function note_citations(sup)
  local s = stringify(sup):gsub("–", "-"):gsub("%s", "")
  if not s:match("^[%d,%-]+$") then return end
  for a, b in s:gmatch("(%d+)%-(%d+)") do
    for k = tonumber(a), tonumber(b) do cited[k] = true end
  end
  for n in s:gmatch("%d+") do cited[tonumber(n)] = true end
end

local function structure(doc)
  local blocks = doc.blocks
  local out = pandoc.List()
  local i, n = 1, #blocks
  local title
  local front = {}
  if blocks[1] and blocks[1].t == "Header" and blocks[1].level == 1 then
    title = blocks[1].content; i = 2
  else
    warn("no level-1 title header found")
  end
  while i <= n and blocks[i].t ~= "HorizontalRule" do
    if blocks[i].t == "Para" then front[#front+1] = to_latex_inlines(blocks[i].content) end
    i = i + 1
  end
  i = i + 1
  local refs_total, refs_numbers = 0, {}
  local figs, tabs = 0, 0
  local pending_legend = nil
  local function note_para(nb)
    return nb and nb.t == "Para" and not first_strong_text(nb) and not image_of(nb)
  end
  while i <= n do
    local b = blocks[i]
    local img = image_of(b)
    if b.t == "Header" and stringify(b):lower() == "references" and blocks[i+1] and blocks[i+1].t == "OrderedList" then
      local ol = blocks[i+1]
      local start = ol.start or 1
      for k = 1, #ol.content do refs_numbers[#refs_numbers+1] = start + k - 1 end
      refs_total = refs_total + #ol.content
      b.level = math.max(b.level - 1, 1)
      out:insert(b)
      out:insert(raw("%SNREFS-BEGIN\n\\begingroup\\snrefsize\\setlength{\\parskip}{0pt}\\raggedright"))
      out:insert(ol)
      out:insert(raw("\\endgroup\n%SNREFS-END"))
      i = i + 2
    elseif b.t == "Header" then
      b.level = math.max(b.level - 1, 1)
      out:insert(b); i = i + 1
    elseif is_fig_legend(b) and image_of(blocks[i+1]) then
      pending_legend = b; i = i + 1
    elseif img then
      local legend = pending_legend
      pending_legend = nil
      if not legend and is_fig_legend(blocks[i+1]) then legend = blocks[i+1]; i = i + 1 end
      if not legend then warn("image without legend: " .. img.src) end
      figs = figs + 1
      out:insert(raw("\\par\\medskip\\noindent\\begin{minipage}{\\linewidth}\\centering\n"
        .. "\\includegraphics[width=\\linewidth,height=0.5\\textheight,keepaspectratio]{"
        .. graphic_src(img.src) .. "}\\par\\smallskip\n\\snjust\\footnotesize"))
      if legend then out:insert(legend) end
      out:insert(raw("\\end{minipage}\\par\\medskip"))
      i = i + 1
    elseif is_table_title(b) and blocks[i+1] and blocks[i+1].t == "Table" then
      local tbl = blocks[i+1]
      tabs = tabs + 1
      local fs, lead, sep = table_style(#tbl.colspecs)
      local st = tbl.attr.attributes["snfs"]
      if st then fs, lead, sep = st:match("([%d.]+),([%d.]+),([%d.]+)") end
      local size = string.format("\\fontsize{%s}{%s}\\selectfont\\setlength{\\tabcolsep}{%spt}", fs, lead, sep)
      out:insert(raw("\\Needspace{10\\baselineskip}\\par\\medskip\\begingroup\\small\\snjust"))
      out:insert(b)
      out:insert(raw("\\endgroup\\begingroup" .. size .. "\\setlength{\\LTpre}{2pt}\\setlength{\\LTpost}{2pt}"))
      out:insert(tbl)
      out:insert(raw("\\endgroup"))
      i = i + 2
      if note_para(blocks[i]) then
        out:insert(raw("\\begingroup\\footnotesize\\snjust"))
        out:insert(blocks[i])
        out:insert(raw("\\par\\endgroup\\medskip"))
        i = i + 1
      end
    elseif b.t == "Table" then
      local fs, lead, sep = table_style(#b.colspecs)
      local st = b.attr.attributes["snfs"]
      if st then fs, lead, sep = st:match("([%d.]+),([%d.]+),([%d.]+)") end
      out:insert(raw(string.format("\\begingroup\\fontsize{%s}{%s}\\selectfont\\setlength{\\tabcolsep}{%spt}"
        .. "\\setlength{\\LTpre}{2pt}\\setlength{\\LTpost}{2pt}", fs, lead, sep)))
      out:insert(b)
      out:insert(raw("\\endgroup"))
      i = i + 1
      if note_para(blocks[i]) then
        out:insert(raw("\\begingroup\\footnotesize\\snjust"))
        out:insert(blocks[i])
        out:insert(raw("\\par\\endgroup\\medskip"))
        i = i + 1
      end
    else
      out:insert(b); i = i + 1
    end
  end
  if pending_legend then out:insert(pending_legend) end

  pandoc.walk_block(pandoc.Div(out), { Superscript = function(s) note_citations(s) end })
  local maxref = 0
  for k, num in ipairs(refs_numbers) do
    if num ~= k then warn(string.format("reference numbering not contiguous at item %d (number %d)", k, num)) end
    maxref = math.max(maxref, num)
  end
  for k in pairs(cited) do
    if k > maxref then warn("cited reference " .. k .. " has no entry") end
  end
  local uncited = {}
  for k = 1, maxref do if not cited[k] then uncited[#uncited+1] = tostring(k) end end
  if #uncited > 0 then warn("references never cited: " .. table.concat(uncited, ",")) end
  warn(string.format("references=%d (max number %d); figures=%d; tables=%d", refs_total, maxref, figs, tabs))

  if title then doc.meta.title = pandoc.MetaInlines(title) end
  doc.meta.rfront = pandoc.MetaBlocks({raw(table.concat(front, "\n\n"))})
  doc.blocks = out
  return doc
end

-- table widths run first, while inline code is still a Code element
return { pass0, pass2, pass1, { Pandoc = structure } }
