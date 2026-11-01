// HUD v8 (designed at 1920x1080 and scaled with the window
// height). Base layout inspired by public launch webcasts (no panels, light bottom shade, DIN type, timeline arc with the mission clock) plus our
// own instruments. Monochrome except the plan-deviation labels (amber / red). Geometry is GL (with faint dark halos),
// text is IMGUI with a soft shadow.
// overlay.txt (kspilot/hud.py, v=8): t0 ut warp evx left right hud_tx asc orb target pos pred dist thr t cam camtel
// camuntil (+ legacy keys: clock events met pip; recording pipeline v2 only: rec=full hides the game-warp tag, cd=<UT>
// planned lift-off -> T- countdown clock; V12: no big 10..1 / LIFTOFF text; V15: data panels hidden before lift-off). The widget in the bottom-right corner follows the UNIFIED SHOW
// TIMELINE (kspilot/showrun.py, show-timeline notes): every transition is animated at exactly the UT listed in hud_tx,
// so the director can keep its cuts clear of them:
//   ASCENT_IN (lift-off + 3 s) ascent curve slides in | ORBIT_ICON (ORBIT + 2 s) it shrinks into the 72 px orbit
//   icon beside the right table | PAYLOAD_SPLIT the icon's dot splits once | FUNNEL_IN (S7->S8 cut + 2 s) entry
//   funnel slides in | HANDOFF (1 km) the funnel zooms into its mouth = top of the fixed-scale 1 km convergence
//   volume | TD_PLOT (touchdown + 2.5 s) touchdown plot slides in left of it. Fixed viewing direction, no rotation,
//   no collapse after the landing.
using System;
using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace AutoLab
{
    public class HudSeries
    {
        public string title, scale;
        public float ymin, ymax;
        public List<Vector2> pts = new List<Vector2>();
    }

    public static class HudV2
    {
        // brightness raised after review feedback that the HUD was too dark (2026-10-20): lighter bottom shade, brighter lines, dark halos
        static readonly Color HI = Color.white, MID = new Color(1f, 1f, 1f, 0.92f), RING = new Color(1f, 1f, 1f, 0.5f),
            ARC = new Color(1f, 1f, 1f, 0.7f), PAST = new Color(1f, 1f, 1f, 0.8f),
            AMBER = new Color(1f, 0.75f, 0.25f), RED = new Color(1f, 0.353f, 0.302f), DARK = new Color(0.05f, 0.07f, 0.09f, 1f);
        static bool halo = true;
        static float gA = 1f;               // group alpha (animations)
        static Vector2 gOff = Vector2.zero; // group offset (animations), design px
        static Color Ca(Color c) { return new Color(c.r, c.g, c.b, c.a * gA); }
        static Color Wa(float a) { return new Color(1f, 1f, 1f, a); }
        static Material mat;
        static Font din;
        static Texture2D shade, white;
        static GUIStyle sTitle, sTitleR, sGLbl, sGNum, sClock, sMission, sEvt, sEvtPast, sSpark, sSparkR;
        static float K, W, H;
        static double UT;                   // game time of this frame (Planetarium)
        static bool v8;

        static float P(Dictionary<string, string> kv, string k, float def = float.NaN)
        {
            string s; float v;
            return kv.TryGetValue(k, out s) && float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v) ? v : def;
        }

        static bool D(Dictionary<string, string> kv, string k, out double v)
        {
            string s; v = double.NaN;
            return kv.TryGetValue(k, out s) && double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v);
        }

        static string S(Dictionary<string, string> kv, string k, string def = "")
        {
            string s; return kv.TryGetValue(k, out s) && s.Length > 0 ? s : def;
        }

        static float[] Floats(Dictionary<string, string> kv, string k, int n)
        {   // comma list; empty / bad fields -> NaN; null if the key is missing or too short
            string s;
            if (!kv.TryGetValue(k, out s) || s.Length == 0) return null;
            string[] a = s.Split(',');
            if (a.Length < n) return null;
            var r = new float[n];
            for (int i = 0; i < n; i++)
                if (!float.TryParse(a[i], NumberStyles.Float, CultureInfo.InvariantCulture, out r[i])) r[i] = float.NaN;
            return r;
        }

        static GUIStyle St(int size, Color c, TextAnchor a)
        {
            var st = new GUIStyle(GUI.skin.label) { font = din, fontSize = size, alignment = a, clipping = TextClipping.Overflow, wordWrap = false };
            st.padding = new RectOffset(0, 0, 0, 0);
            st.margin = new RectOffset(0, 0, 0, 0);
            st.normal.textColor = c;
            return st;
        }

        static void Init()
        {
            mat = new Material(Shader.Find("Hidden/Internal-Colored")) { hideFlags = HideFlags.HideAndDontSave };
            mat.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            mat.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
            mat.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            mat.SetInt("_ZWrite", 0);
            mat.SetInt("_ZTest", (int)UnityEngine.Rendering.CompareFunction.Always);
            try { din = Font.CreateDynamicFontFromOSFont(new[] { "DIN Alternate", "DIN Condensed", "Helvetica Neue", "Arial" }, 16); }
            catch (Exception) { din = null; }
            shade = new Texture2D(1, 64, TextureFormat.ARGB32, false) { wrapMode = TextureWrapMode.Clamp, filterMode = FilterMode.Bilinear };
            for (int y = 0; y < 64; y++) shade.SetPixel(0, y, new Color(0f, 0f, 0f, 0.35f * (1f - y / 63f)));   // row 0 = bottom
            shade.Apply();
            white = new Texture2D(1, 1); white.SetPixel(0, 0, Color.white); white.Apply();
            sTitle = St(15, HI, TextAnchor.MiddleLeft);
            sTitleR = St(13, MID, TextAnchor.MiddleRight);
            sGLbl = St(11, MID, TextAnchor.MiddleCenter);
            sGNum = St(30, HI, TextAnchor.MiddleCenter);
            sClock = St(34, HI, TextAnchor.MiddleCenter);
            sMission = St(12, MID, TextAnchor.MiddleCenter);
            sEvt = St(12, HI, TextAnchor.MiddleCenter);
            sEvtPast = St(12, PAST, TextAnchor.MiddleCenter);
            sSpark = St(10, MID, TextAnchor.MiddleLeft);   // also the scope ring labels
            sSparkR = St(11, HI, TextAnchor.MiddleRight);
        }

        // per-frame string work is cached (perf): the same labels are re-spaced / measured every frame
        static readonly Dictionary<string, string> sp1 = new Dictionary<string, string>(), sp2 = new Dictionary<string, string>();
        static readonly System.Text.StringBuilder spSb = new System.Text.StringBuilder();
        static string Sp(string s, int gap = 1)
        {   // letter-spaced caps (IMGUI has no tracking): "AERO GLIDE" -> "A E R O   G L I D E"
            var m = gap == 1 ? sp1 : gap == 2 ? sp2 : null;
            string r;
            if (m != null && m.TryGetValue(s, out r)) return r;
            var sb = spSb; sb.Length = 0;
            foreach (char c in s.ToUpperInvariant()) { if (sb.Length > 0) sb.Append(' ', gap); sb.Append(c); }
            r = sb.ToString();
            if (m != null) { if (m.Count > 1000) m.Clear(); m[s] = r; }
            return r;
        }

        static readonly GUIContent gc = new GUIContent();
        static readonly Dictionary<GUIStyle, Dictionary<string, float>> wdCache = new Dictionary<GUIStyle, Dictionary<string, float>>();
        static float Wd(string t, GUIStyle s)
        {
            Dictionary<string, float> m; float w;
            if (!wdCache.TryGetValue(s, out m)) wdCache[s] = m = new Dictionary<string, float>();
            if (m.TryGetValue(t, out w)) return w;
            if (m.Count > 1000) m.Clear();
            gc.text = t; w = s.CalcSize(gc).x; m[t] = w;
            return w;
        }

        static readonly string[] chr = new string[128];
        static string Ch(char c) { return c < 128 ? (chr[c] ?? (chr[c] = c.ToString())) : c.ToString(); }

        // ---------- GL helpers (design units, origin top-left) ----------
        // Batching (perf, 2026-10-21): every primitive used to open its own GL.Begin/GL.End pair (one draw call each;
        // a line segment with its halo = 2, a 1500-point track = 3000 per frame). Consecutive primitives of the same
        // kind now share one batch, closed only when the kind changes or at EndGL: same drawing order (halo under the
        // line, discs over lines), same picture.
        static int glMode = -1;             // kind of the open GL batch (-1: none)
        public static int nBatch, nVert;    // counters for the perf log (AutoLab Recorder)
        static void Prim(int mode)
        {
            if (glMode == mode) return;
            if (glMode >= 0) GL.End();
            GL.Begin(mode); glMode = mode; nBatch++;
        }
        static void V(float x, float y, float z) { GL.Vertex3(x, y, z); nVert++; }
        static void BeginGL() { mat.SetPass(0); GL.PushMatrix(); GL.LoadPixelMatrix(0, W, H, 0); GL.MultMatrix(Matrix4x4.Translate(new Vector3(gOff.x, gOff.y, 0f))); glMode = -1; }
        static void EndGL() { if (glMode >= 0) GL.End(); glMode = -1; GL.PopMatrix(); }
        static Vector2 Pol(float cx, float cy, float r, float deg) { float a = deg * Mathf.Deg2Rad; return new Vector2(cx + r * Mathf.Cos(a), cy + r * Mathf.Sin(a)); }

        // optional clip rectangle (the zooming funnel of the 1 km hand-off stays inside its window)
        static bool clipOn;
        static UnityEngine.Rect clipR;
        static readonly float[] clipP = new float[4], clipQ = new float[4];
        static bool Clip(ref Vector2 a, ref Vector2 b)
        {   // Liang-Barsky
            float t0 = 0f, t1 = 1f, dx = b.x - a.x, dy = b.y - a.y;
            float[] p = clipP, q = clipQ;
            p[0] = -dx; p[1] = dx; p[2] = -dy; p[3] = dy;
            q[0] = a.x - clipR.xMin; q[1] = clipR.xMax - a.x; q[2] = a.y - clipR.yMin; q[3] = clipR.yMax - a.y;
            for (int i = 0; i < 4; i++)
            {
                if (Mathf.Abs(p[i]) < 1e-6f) { if (q[i] < 0f) return false; continue; }
                float r = q[i] / p[i];
                if (p[i] < 0f) { if (r > t1) return false; if (r > t0) t0 = r; }
                else { if (r < t0) return false; if (r < t1) t1 = r; }
            }
            Vector2 a0 = a;
            a = a0 + t0 * new Vector2(dx, dy); b = a0 + t1 * new Vector2(dx, dy);
            return true;
        }

        static void Arc(float cx, float cy, float r, float w, float a0, float a1, Color c, bool caps)
        {
            if (a1 <= a0) return;
            int n = Mathf.Max(8, (int)((a1 - a0) / 2f));
            Prim(GL.QUADS); GL.Color(Ca(c));
            for (int i = 0; i < n; i++)
            {
                float t0 = a0 + (a1 - a0) * i / n, t1 = a0 + (a1 - a0) * (i + 1) / n;
                Vector2 p0 = Pol(cx, cy, r - w / 2, t0), p1 = Pol(cx, cy, r + w / 2, t0), p2 = Pol(cx, cy, r + w / 2, t1), p3 = Pol(cx, cy, r - w / 2, t1);
                V(p0.x, p0.y, 0); V(p1.x, p1.y, 0); V(p2.x, p2.y, 0); V(p3.x, p3.y, 0);
            }
            if (caps) { Disc(Pol(cx, cy, r, a0), w / 2, c); Disc(Pol(cx, cy, r, a1), w / 2, c); }
        }

        static void Disc(Vector2 p, float r, Color c)
        {
            if (clipOn && !clipR.Contains(p)) return;
            int n = Mathf.Max(12, (int)(r * 4));
            Prim(GL.TRIANGLES); GL.Color(Ca(c));
            for (int i = 0; i < n; i++)
            {
                Vector2 a = Pol(p.x, p.y, r, 360f * i / n), b = Pol(p.x, p.y, r, 360f * (i + 1) / n);
                V(p.x, p.y, 0); V(a.x, a.y, 0); V(b.x, b.y, 0);
            }
        }

        static void Seg(Vector2 a, Vector2 b, float w, Color c)
        {
            Vector2 d = b - a; if (d.sqrMagnitude < 1e-6f) return;
            if (clipOn && !Clip(ref a, ref b)) return;
            d = b - a; if (d.sqrMagnitude < 1e-6f) return;
            if (halo)
            {   // faint dark outline first: lines stay readable over a bright sky
                halo = false; Seg(a, b, w + 2.5f, new Color(0f, 0f, 0f, 0.3f * c.a)); halo = true;
            }
            Vector2 nrm = new Vector2(-d.y, d.x).normalized * (w / 2);
            Prim(GL.QUADS); GL.Color(Ca(c));
            V(a.x + nrm.x, a.y + nrm.y, 0); V(a.x - nrm.x, a.y - nrm.y, 0);
            V(b.x - nrm.x, b.y - nrm.y, 0); V(b.x + nrm.x, b.y + nrm.y, 0);
        }

        static void Poly(List<Vector2> pts, float w, Color c) { for (int i = 1; i < pts.Count; i++) Seg(pts[i - 1], pts[i], w, c); }

        static void DashPoly(List<Vector2> pts, float w, Color c, float on, float off)
        {   // dashed polyline, the dash phase runs on over the vertices
            float phase = 0f, per = on + off;
            for (int i = 1; i < pts.Count; i++)
            {
                Vector2 a = pts[i - 1], dv = pts[i] - a;
                float len = dv.magnitude, pos = 0f;
                if (len < 1e-4f) continue;
                dv /= len;
                while (pos < len)
                {
                    float inPh = phase % per, e = Mathf.Min(len, pos + (inPh < on ? on - inPh : per - inPh));
                    if (inPh < on) Seg(a + dv * pos, a + dv * e, w, c);
                    phase += e - pos; pos = e;
                }
            }
        }

        static readonly List<Vector2> dash2 = new List<Vector2> { Vector2.zero, Vector2.zero };
        static void Dash(Vector2 a, Vector2 b, float w, Color c, float on, float off) { dash2[0] = a; dash2[1] = b; DashPoly(dash2, w, c, on, off); }

        static void Cross(Vector2 c, float a, float b)
        {
            Seg(c + new Vector2(-a, 0), c + new Vector2(-b, 0), 1.6f, HI); Seg(c + new Vector2(b, 0), c + new Vector2(a, 0), 1.6f, HI);
            Seg(c + new Vector2(0, -a), c + new Vector2(0, -b), 1.6f, HI); Seg(c + new Vector2(0, b), c + new Vector2(0, a), 1.6f, HI);
        }

        static void Diamond(Vector2 p, float s, float w)
        {
            Prim(GL.QUADS); GL.Color(Ca(DARK));
            V(p.x, p.y - s, 0); V(p.x + s, p.y, 0); V(p.x, p.y + s, 0); V(p.x - s, p.y, 0);
            Vector2 n = p + new Vector2(0, -s), e = p + new Vector2(s, 0), so = p + new Vector2(0, s), we = p + new Vector2(-s, 0);
            Seg(n, e, w, HI); Seg(e, so, w, HI); Seg(so, we, w, HI); Seg(we, n, w, HI);
        }

        static GUIStyle sWinLS, sWinRS;
        static GUIStyle sWinL() { return sWinLS ?? (sWinLS = St(12, HI, TextAnchor.MiddleLeft)); }
        static GUIStyle sWinR() { return sWinRS ?? (sWinRS = St(12, MID, TextAnchor.MiddleRight)); }

        static readonly Vector2[] rrC = new Vector2[4];
        static void RoundRect(float x, float y, float w, float h, float r, Color fill, Color line)
        {   // filled rounded rectangle (centre cross + corner discs) and a 1 px outline
            bool hk = halo; halo = false;
            r = Mathf.Min(r, Mathf.Min(w, h) / 2f);
            Prim(GL.QUADS); GL.Color(Ca(fill));
            V(x + r, y, 0); V(x + w - r, y, 0); V(x + w - r, y + h, 0); V(x + r, y + h, 0);
            V(x, y + r, 0); V(x + r, y + r, 0); V(x + r, y + h - r, 0); V(x, y + h - r, 0);
            V(x + w - r, y + r, 0); V(x + w, y + r, 0); V(x + w, y + h - r, 0); V(x + w - r, y + h - r, 0);
            rrC[0] = new Vector2(x + r, y + r); rrC[1] = new Vector2(x + w - r, y + r); rrC[2] = new Vector2(x + w - r, y + h - r); rrC[3] = new Vector2(x + r, y + h - r);
            foreach (var c in rrC)
            {   // quarter discs (a full disc would double the alpha where it overlaps the cross)
                float a0 = c.x < x + w / 2 ? (c.y < y + h / 2 ? 180f : 90f) : (c.y < y + h / 2 ? 270f : 0f);
                Prim(GL.TRIANGLES); GL.Color(Ca(fill));
                for (int i = 0; i < 8; i++) { Vector2 p = Pol(c.x, c.y, r, a0 + 90f * i / 8), q = Pol(c.x, c.y, r, a0 + 90f * (i + 1) / 8); V(c.x, c.y, 0); V(p.x, p.y, 0); V(q.x, q.y, 0); }
                Arc(c.x, c.y, r, 1f, a0, a0 + 90f, line, false);
            }
            Seg(new Vector2(x + r, y), new Vector2(x + w - r, y), 1f, line); Seg(new Vector2(x + r, y + h), new Vector2(x + w - r, y + h), 1f, line);
            Seg(new Vector2(x, y + r), new Vector2(x, y + h - r), 1f, line); Seg(new Vector2(x + w, y + r), new Vector2(x + w, y + h - r), 1f, line);
            halo = hk;
        }

        static void L(float x, float y, float w, float h, string t, GUIStyle s)
        {   // 1 px soft shadow under every label, then the label
            x += gOff.x; y += gOff.y;
            Color keep = GUI.contentColor, keepC = GUI.color;
            GUI.color = new Color(1f, 1f, 1f, gA);
            GUI.contentColor = new Color(0f, 0f, 0f, 0.5f);
            GUI.Label(new UnityEngine.Rect(x + 1, y + 1, w, h), t, s);
            GUI.contentColor = keep;
            GUI.Label(new UnityEngine.Rect(x, y, w, h), t, s);
            GUI.color = keepC;
        }

        // ---------- the HUD ----------
        // animation state (game time: smooth in recordings, which fix the frame rate)
        static float tIn = -1f, tPip = -1f, tEnd = -1f;
        static float Ease(float x) { x = Mathf.Clamp01(x); return x * x * (3f - 2f * x); }
        static float gPan = 1f;             // V15 panel reveal (0 before lift-off in recording mode, else 1)
        const float PANEL_IN = 0.8f;        // s (game): V15 data panels slide / fade in after the lift-off (showrun.PANEL_IN)

        /// <summary>Called by the recorder whenever no HUD is shown: the next HUD starts with its intro.</summary>
        public static void Idle()
        {   // no HUD on screen: the next one starts fresh (intro, no stale track from a previous flight)
            tIn = tPip = tEnd = -1f;
            track.Clear(); trackThr.Clear(); funTrack.Clear(); predTrail.Clear(); ascTrack.Clear();
            lastT = -1e30f; lastPredT = -1e30f; pipUt = float.NaN; lastAscUt = -1.0; orbLast = null; orbUt = double.NaN;
            updKv = evKv = txKv = rowsKv = null;
        }
        static Dictionary<string, string> updKv;        // overlay the tracks were last updated from

        public static void Draw(Dictionary<string, string> kv, IList<HudSeries> plots)
        {
            if (mat == null) Init();
            if (!ReferenceEquals(kv, updKv))
            {   // track recording once per new overlay (the recorder hands over a new dictionary only when the file changed)
                updKv = kv;
                ConvergenceUpdate(kv);
                AscentUpdate(kv);
            }
            // perf: nothing on screen is interactive, so all work (string layout, labels, GL) is done in the Repaint
            // pass only; OnGUI also runs for Layout and every input event (>= 2x per frame)
            if (Event.current.type != EventType.Repaint) return;
            K = Screen.height / 1080f;
            W = Screen.width / K; H = 1080f;
            bool paint = true;
            Matrix4x4 keep = GUI.matrix;
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(K, K, 1f));
            float now = Time.time;
            UT = Planetarium.GetUniversalTime();
            v8 = S(kv, "v") == "8";
            if (tIn < 0f) tIn = now;
            if (S(kv, "end") == "1" && tEnd < 0f) tEnd = now;      // aborted runs only: a landing holds its last frame
            float pIn = Ease((now - tIn) / 1.0f), pOut = tEnd < 0f ? 0f : Ease((now - tEnd) / 1.4f);
            float alpha = pIn * (1f - pOut);

            gA = alpha; gOff = Vector2.zero;
            if (paint) { GUI.color = new Color(1, 1, 1, alpha); GUI.DrawTexture(new UnityEngine.Rect(0, H - 240, W, 240), shade); GUI.color = Color.white; }

            // V15 (requirement 2026-10-21, recording HUD only: rec=full + cd=): before lift-off only the timeline arc
            // and the T- clock; the data panels (readout rows, caption, side widget) slide / fade in from the lift-off
            // UT over PANEL_IN s. pPan = 1 for every other HUD (unchanged). V12: no big countdown digits / LIFTOFF word.
            // V15 leftover (video requirements v2): the NEXT block at the arc's right shoulder is a data panel too - it is hidden
            // before lift-off and fades / slides in with the panels (TimelineArc reads gPan; the arc's node labels stay).
            double cdV, t0V;
            bool haveCdV = D(kv, "cd", out cdV), haveT0V = D(kv, "t0", out t0V);
            float pPan = RecMath.PanelReveal(v8 && S(kv, "rec") == "full", haveCdV, haveT0V, UT, t0V, PANEL_IN);
            float pSlide = (1f - pPan) * 120f;
            gPan = pPan;
            // timeline arc + clock: rises in at the start, sinks away at the end
            gOff = new Vector2(0f, (1f - pIn) * 140f + pOut * 140f);
            TimelineArc(kv, paint);
            // readouts: slide out sideways at the end
            if (pPan > 0f)
            {
                gA = alpha * pPan;
                gOff = new Vector2(-pOut * 260f - pSlide, 0f); Rows(kv, true, paint);
                gOff = new Vector2(pOut * 260f + pSlide, 0f); Rows(kv, false, paint);
                gA = alpha;
            }
            gOff = Vector2.zero;
            // ground telemetry caption (director shot S1): two small lines, top left; fades out at camuntil (T+30)
            if (S(kv, "cam").Length > 0 && pPan > 0f)
            {
                double cu; float ca = D(kv, "camuntil", out cu) ? 1f - Ease((float)(UT - cu) / 0.8f) : 1f;
                if (ca > 0f) { gA = alpha * ca * pPan; gOff = new Vector2(-pOut * 260f - pSlide, 0f); CamCaption(S(kv, "cam"), S(kv, "camtel"), paint); gA = alpha; }
            }

            WX = W / 2 + 530f;
            if (v8 && pPan > 0f) { gA = alpha * pPan; Widgets(kv, paint, pOut); gA = alpha; }
            else
            {   // legacy overlay (lander without the whole-mission feed): convergence slides in with the landing burn
                bool pip = S(kv, "pip") == "1" || (!kv.ContainsKey("v") && P(kv, "phase", 0f) >= 3f);
                if (pip && tPip < 0f) tPip = now;
                if (tPip >= 0f)
                {
                    gOff = new Vector2((1f - Ease((now - tPip) / 0.6f)) * 460f + pOut * 460f, 0f);
                    ConvBox(kv, paint, false);
                }
            }
            gA = 1f; gOff = Vector2.zero;
            if (S(kv, "plots") == "on") Sparks(plots, 64, 72, paint);       // research sparklines (top left)
            GUI.matrix = keep;
        }

        static GUIStyle sCamTelS;
        static GUIStyle sCamTel() { return sCamTelS ?? (sCamTelS = St(13, MID, TextAnchor.MiddleLeft)); }
        static void CamCaption(string cam, string tel, bool paint)
        {   // "● PAD TRACKING CAMERA" + one line of what the tracking station measures (range, elevation, azimuth)
            if (paint) { BeginGL(); Disc(new Vector2(70f, 61f), 4f, HI); EndGL(); }
            L(82, 52, 640, 18, Sp(cam), sWinL());
            if (tel.Length > 0) L(82, 74, 640, 18, tel, sCamTel());
        }

        static string Dur(float s)
        {
            int n = Mathf.Max(0, Mathf.RoundToInt(s));
            if (n >= 3600) return string.Format("{0}H {1:00}M", n / 3600, n / 60 % 60);
            return n >= 60 ? string.Format("{0}:{1:00}", n / 60, n % 60) : n + " S";
        }

        static string CountClock(float rem)
        {   // v2 countdown clock before lift-off (overlay key cd = planned lift-off UT): T- HH:MM:SS, whole seconds up
            int n = Mathf.Max(0, Mathf.CeilToInt(rem));
            return string.Format("T− {0:00}:{1:00}:{2:00}", n / 3600, n / 60 % 60, n % 60);
        }

        static string Clock(float met)
        {   // mission clock, always T+ HH:MM:SS (v8)
            int n = Mathf.Max(0, Mathf.FloorToInt(met));
            return string.Format("T+ {0:00}:{1:00}:{2:00}", n / 3600, n / 60 % 60, n % 60);
        }

        // ---------- whole-flight timeline: compressive scale (29 deg * tanh(dt / 120 s)), "now" under the apex ----------
        class Ev { public string name, dev; public float dt; public int lvl; public double ut; public bool done; }
        static readonly List<Vector4> arcBoxes = new List<Vector4>();
        static readonly List<KeyValuePair<Ev, Vector4>> arcLay = new List<KeyValuePair<Ev, Vector4>>();
        static Dictionary<string, string> evKv;
        static readonly List<Ev> evList = new List<Ev>();
        static List<Ev> Events(Dictionary<string, string> kv)
        {   // the event list only changes with the overlay (4 Hz): parse it once, not every frame
            if (ReferenceEquals(kv, evKv)) return evList;
            evKv = kv; evList.Clear();
            if (S(kv, "v") == "8")
            {
                foreach (string e in S(kv, "evx").Split('|'))
                {   // NAME;ut;planned;done;level;dev
                    string[] f = e.Split(';'); double u;
                    if (f.Length < 6 || !double.TryParse(f[1], NumberStyles.Float, CultureInfo.InvariantCulture, out u)) continue;
                    int lvl; int.TryParse(f[4], out lvl);
                    evList.Add(new Ev { name = f[0], ut = u, lvl = lvl, dev = f[5], done = f[3] == "1" });
                }
                evList.Sort((a, b) => a.ut.CompareTo(b.ut));               // = order of dt = ut - UT at any UT
            }
            else
            {
                foreach (string e in S(kv, "events").Split('|'))
                {
                    int k = e.LastIndexOf(':'); float t;
                    if (k > 0 && float.TryParse(e.Substring(k + 1), NumberStyles.Float, CultureInfo.InvariantCulture, out t))
                        evList.Add(new Ev { name = e.Substring(0, k), dt = t, dev = "" });
                }
                evList.Sort((a, b) => a.dt.CompareTo(b.dt));
            }
            return evList;
        }
        static GUIStyle sEvtT, sEvtTPast, sNextL, sNextV, sDevA, sDevR, sDevP, sTag;
        static void TimelineArc(Dictionary<string, string> kv, bool paint)
        {
            if (sEvtT == null)
            {
                sEvtT = St(11, new Color(1, 1, 1, 0.85f), TextAnchor.MiddleLeft); sEvtTPast = St(11, new Color(1, 1, 1, 0.6f), TextAnchor.MiddleLeft);
                sNextL = St(11, new Color(1, 1, 1, 0.75f), TextAnchor.MiddleRight); sNextV = St(14, HI, TextAnchor.MiddleRight);
                sDevA = St(11, AMBER, TextAnchor.MiddleLeft); sDevR = St(11, RED, TextAnchor.MiddleLeft); sDevP = St(11, new Color(1, 1, 1, 0.6f), TextAnchor.MiddleLeft);
                sTag = St(11, new Color(1, 1, 1, 0.9f), TextAnchor.MiddleLeft);
            }
            float cx = W / 2, R = 1000f, cy = 1080f - 70f + R - 60f;
            Func<float, float> ANG = dt => 29f * (float)Math.Tanh(dt / 120.0);
            Func<float, Vector2> A = a => new Vector2(cx + R * Mathf.Sin(a * Mathf.Deg2Rad), cy - R * Mathf.Cos(a * Mathf.Deg2Rad));
            Func<float, float> arcY = x => cy - Mathf.Sqrt(Mathf.Max(R * R - (x - cx) * (x - cx), 0f));
            List<Ev> evs = Events(kv);                                       // parsed once per overlay, sorted by time
            if (v8) foreach (var e in evs) e.dt = (float)(e.ut - UT);        // nodes move with the plugin's own clock
            double t0 = double.NaN; bool haveT0 = v8 && D(kv, "t0", out t0);
            float met = haveT0 ? (float)(UT - t0) : P(kv, "met");

            // NEXT above the arc's right shoulder; it is an obstacle for the arc labels
            var boxes = arcBoxes; boxes.Clear();                             // x0, x1, y0, y1
            Ev nx = null;
            // recording v2 fix round 3 (rec=full only; QA flight #3 showed "DEORBIT BURN · IN 14 S" while burning and
            // "IN 6 S" after the cutoff - the HUD node is the planned node UT, reported done with that future UT):
            // a done event is never NEXT, and a burn event while the engine is lit reads "NOW"
            bool v2rec = S(kv, "rec") == "full";
            foreach (var e in evs) if (e.dt > 0 && !(v2rec && e.done)) { nx = e; break; }
            bool burnNow = v2rec && nx != null && nx.name.EndsWith("BURN") && nx.dt < 120f &&
                           P(kv, "thr_now", P(kv, "thr", 0f)) > 0.02f;
            string nxt = nx == null ? "" : Sp(nx.name) + "   ·   " + Sp(burnNow ? "NOW" : "IN " + Dur(nx.dt));
            bool showNext = nx != null && gPan > 0f;                        // V15: hidden before lift-off (rec=full)
            if (showNext) boxes.Add(new Vector4(cx + 500 - Mathf.Max(Wd(nxt, sNextV), Wd(Sp("NEXT", 2), sNextL)) - 10, cx + 510, 898, 940));
            // arc labels: always ABOVE the arc line over their whole width, stacked clear of each other and of NEXT
            var lay = arcLay; lay.Clear();                                    // x, top, node y, label half width
            foreach (var e in evs)
            {
                float a = ANG(e.dt); if (Mathf.Abs(a) > 24f) continue;          // far events: dots only
                Vector2 p = A(a);
                string tt = TimeLabel(e, met);
                float wt = Wd(tt, sEvtT), wdv = e.dev.Length > 0 ? Wd(e.dev, sDevA) + 6f : 0f;
                float half = Mathf.Max(Mathf.Max(Wd(Sp(e.name), sEvt), wt + wdv), 70f) / 2f + 8f;
                float top = Mathf.Min(p.y, Mathf.Min(arcY(p.x - half), arcY(p.x + half))) - 48f;
                for (bool moved = true; moved;)
                {
                    moved = false;
                    foreach (var b in boxes)
                        if (p.x + half > b.x && p.x - half < b.y && top < b.w && top + 30f > b.z) { top = b.z - 32f; moved = true; }
                }
                boxes.Add(new Vector4(p.x - half, p.x + half, top, top + 30f));
                lay.Add(new KeyValuePair<Ev, Vector4>(e, new Vector4(p.x, top, p.y, wt + wdv)));
            }
            if (paint)
            {
                BeginGL();
                Vector2 prev = A(-30f);
                for (int i = 1; i <= 120; i++) { Vector2 p = A(-30f + 60f * i / 120); Seg(prev, p, 2f, ARC); prev = p; }
                for (int m = -5; m <= 5; m++)                            // minute ticks (bunch up towards the ends)
                {
                    if (m == 0) continue;
                    float a = ANG(m * 60f); Vector2 p = A(a), d = new Vector2(Mathf.Sin(a * Mathf.Deg2Rad), -Mathf.Cos(a * Mathf.Deg2Rad));
                    Seg(p, p + 7f * d, 1f, Wa(0.5f));
                }
                foreach (var e in evs)
                {
                    Vector2 p = A(ANG(e.dt));
                    if (e.dt <= 0 || (v2rec && e.done)) Disc(p, 5.5f, HI);       // done (rec=full): solid
                    else { Disc(p, 5.5f, HI); Disc(p, 3.8f, DARK); }
                }
                foreach (var l in lay)                                    // leader line when the label had to move up
                    if (l.Value.y < l.Value.z - 48f) Seg(new Vector2(l.Value.x, l.Value.z - 7f), new Vector2(l.Value.x, l.Value.y + 34f), 1f, Wa(0.35f));
                Vector2 ap = A(0f);                                      // "now" marker under the apex
                Prim(GL.TRIANGLES); GL.Color(Ca(HI));
                V(ap.x - 8, ap.y + 22, 0); V(ap.x + 8, ap.y + 22, 0); V(ap.x, ap.y + 10, 0);
                EndGL();
            }
            foreach (var l in lay)
            {
                Ev e = l.Key; float x = l.Value.x, top = l.Value.y, wtot = l.Value.w;
                bool past = e.dt <= 0;
                L(x - 90, top + 2, 180, 16, Sp(e.name), past ? sEvtPast : sEvt);
                string tt = TimeLabel(e, met);
                float x0 = x - wtot / 2f, wt = Wd(tt, sEvtT);
                L(x0, top + 19, wt + 4, 14, tt, past ? sEvtTPast : sEvtT);
                if (e.dev.Length > 0) L(x0 + wt + 6, top + 19, 60, 14, e.dev, e.lvl == 1 ? sDevA : e.lvl == 2 ? sDevR : sDevP);
            }
            // mission clock: game time since lift-off (the plugin's own UT: smooth, right under warp) + WARP tag
            double cdUt = double.NaN; bool haveCd = v8 && D(kv, "cd", out cdUt);           // v2: T- countdown before lift-off
            string clock = v8 ? (haveT0 ? Clock(met) : haveCd ? CountClock((float)(cdUt - UT)) : "T+ --:--:--") : S(kv, "clock", "");
            L(cx - 200, 1014, 400, 40, clock, sClock);
            L(cx - 250, 1052, 500, 18, Sp(S(kv, "mission", "ORBITAL RETURN"), 2), sMission);
            float rate = TimeWarp.CurrentRate;
            if (v8 && rate < 1.05f) rate = P(kv, "warp", 1f);
            int rateI = Mathf.RoundToInt(rate);
            // rate rounds to >= 2 (a 1.05-1.5 transition used to show "WARP x1"); v2 recordings (rec=full) never draw the
            // game-warp tag: the editor's playback-speed badge is the only speed shown in those videos (spec 3.2)
            if (rate > 1.05f && rateI >= 2 && clock.Length > 0 && S(kv, "rec") != "full")
            {
                string tag = Sp("WARP ×" + rateI.ToString(CultureInfo.InvariantCulture));
                float wx = cx + Wd(clock, sClock) / 2f + 14f, tw = Wd(tag, sTag) + 12f;
                if (paint) { BeginGL(); RoundRect(wx, 1022f, tw, 18f, 4f, new Color(0, 0, 0, 0), Wa(0.6f)); EndGL(); }
                L(wx + 6f, 1022f, tw, 18f, tag, sTag);
            }
            if (showNext)
            {   // fades / slides in with the data panels (gPan = 1 outside the recording pre-lift-off phase)
                float keepA = gA; Vector2 keepO = gOff;
                gA = keepA * gPan; gOff = keepO + new Vector2((1f - gPan) * 120f, 0f);
                L(cx + 500 - 300, 900, 300, 14, Sp("NEXT", 2), sNextL);
                L(cx + 500 - 400, 918, 400, 20, nxt, sNextV);
                gA = keepA; gOff = keepO;
            }
        }

        static string TimeLabel(Ev e, float met)
        {   // mission-clock time of the node (actual in the past, live prediction ahead)
            if (float.IsNaN(met)) return (e.dt <= 0 ? "−" : "IN ") + Dur(Mathf.Abs(e.dt));
            return Clock(met + e.dt);
        }

        // ---------- readout tables (same on both sides): label | value LEFT-aligned, fixed-advance digits | unit ----------
        static GUIStyle sRowL, sRowV, sRowC;
        static float BW = 190f;                 // table width (the orbit icon sits right of the right table)
        static Dictionary<string, string> rowsKv;
        static readonly List<string[]> rowsL = new List<string[]>(), rowsR = new List<string[]>();
        static void Rows(Dictionary<string, string> kv, bool left, bool paint)
        {
            if (!ReferenceEquals(kv, rowsKv)) { rowsKv = kv; BuildRows(kv, true, rowsL); BuildRows(kv, false, rowsR); }   // once per overlay
            Table(left ? W / 2 - 700 : W / 2 + 530, left ? rowsL : rowsR, paint);
        }

        static void BuildRows(Dictionary<string, string> kv, bool left, List<string[]> rows)
        {
            string raw = S(kv, left ? "left" : "right");
            rows.Clear();
            if (raw.Length > 0) foreach (string r in raw.Split('|')) { string[] a = r.Split(';'); if (a.Length >= 3) rows.Add(a); }
            else if (left)
            {   // legacy overlay (no whole-mission feed): altitude / descent
                float alt = P(kv, "alt", 0f), vs = P(kv, "vs");
                rows.Add(new[] { "ALT", alt.ToString(alt < 10 ? "F1" : "F0", CultureInfo.InvariantCulture), "KM" });
                rows.Add(new[] { "VS", float.IsNaN(vs) ? "—" : Mathf.Abs(vs).ToString("F0", CultureInfo.InvariantCulture), "M/S" });
            }
            else
            {
                float thr = Mathf.Clamp01(P(kv, "thr", 0f)), dist = P(kv, "dist");
                rows.Add(new[] { "THR", (thr * 100f).ToString("F0", CultureInfo.InvariantCulture), "%" });
                rows.Add(new[] { "PAD", float.IsNaN(dist) ? "—" : dist.ToString("N0", CultureInfo.InvariantCulture), "M" });
            }
        }

        static void Table(float x0, List<string[]> rows, bool paint)
        {
            if (sRowL == null)
            {
                sRowL = St(12, new Color(1, 1, 1, 0.8f), TextAnchor.MiddleLeft); sRowV = St(26, HI, TextAnchor.MiddleLeft);
                sRowC = St(26, HI, TextAnchor.MiddleCenter);
            }
            float DW = Wd("0", sRowV), PW = Wd(".", sRowV);
            const float VX = 64f;
            float UX = VX + 4 * DW + PW + 2 + 12;
            BW = UX + 44f;
            int n = Mathf.Min(rows.Count, 3);
            for (int i = 0; i < n; i++)
            {
                float y = 976f + 36f * i;
                L(x0, y - 9, VX, 16, Sp(rows[i][0]), sRowL);
                float x = x0 + VX;
                foreach (char c in rows[i][1])
                {
                    float w = (char.IsDigit(c) || c == '+' || c == '-' || c == '−') ? DW : (c == '.' || c == ':') ? PW + 2 : Wd(Ch(c), sRowV);
                    L(x, y - 17, w, 30, Ch(c), sRowC);
                    x += w;
                }
                if (rows[i][2].Length > 0) L(Mathf.Max(x0 + UX, x + 8), y - 9, 120, 16, Sp(rows[i][2]), sRowL);
            }
            if (paint && n > 1)
            {   // thin low-alpha separators between the rows
                BeginGL();
                bool hk = halo; halo = false;
                for (int i = 0; i < n - 1; i++) Seg(new Vector2(x0, 976f + 36f * i + 22f), new Vector2(x0 + BW, 976f + 36f * i + 22f), 1f, Wa(0.13f));
                halo = hk;
                EndGL();
            }
        }

        // ---------- recorded tracks (also while the widget is not shown) ----------
        static readonly List<Vector4> track = new List<Vector4>();     // (E, N, h, game time) below 3 km: convergence
        static readonly List<float> trackThr = new List<float>();
        static readonly List<Vector3> funTrack = new List<Vector3>();  // (E, N, h) below 40 km: entry funnel
        static readonly List<Vector2> predTrail = new List<Vector2>();  // predicted touchdown points (E, N), every 2 s
        static readonly List<Vector2> ascTrack = new List<Vector2>();   // (downrange km, altitude km)
        static float lastT = -1e30f, lastPredT = -1e30f, pipUt = float.NaN;
        // point caps (perf): every point is a GL segment per frame. Old caps 2000 / 1500 / 1500 dropped the OLDEST
        // points (the start of the ascent curve vanished); now the older part is thinned to every 2nd point instead,
        // so the whole curve stays, at <= 800 / 600 / 500 points (a 340 px widget cannot show more)
        const int TRACK_MAX = 800, FUN_MAX = 600, ASC_MAX = 500;
        static void Thin<T>(List<T> l, int keepTail)
        {   // drop every 2nd point of the older part (index 0 and the last keepTail points stay)
            int end = Math.Max(1, l.Count - keepTail), w = 1;
            for (int i = 1; i < l.Count; i++)
                if (i >= end || i % 2 == 0) l[w++] = l[i];
            l.RemoveRange(w, l.Count - w);
        }
        static double lastAscUt = -1.0;

        static bool V3(Dictionary<string, string> kv, string k, out Vector3 v)
        {
            v = Vector3.zero; string s;
            if (!kv.TryGetValue(k, out s)) return false;
            string[] a = s.Split(',');
            float x, y, z = 0f;
            if (a.Length < 2 || !float.TryParse(a[0], NumberStyles.Float, CultureInfo.InvariantCulture, out x)
                || !float.TryParse(a[1], NumberStyles.Float, CultureInfo.InvariantCulture, out y)) return false;
            if (a.Length > 2) float.TryParse(a[2], NumberStyles.Float, CultureInfo.InvariantCulture, out z);
            v = new Vector3(x, y, z); return true;
        }

        static void ConvergenceUpdate(Dictionary<string, string> kv)
        {
            Vector3 pos;
            if (!V3(kv, "pos", out pos)) return;
            float t = P(kv, "t", float.NaN);
            if (float.IsNaN(t) || t == lastT) return;
            if (t < lastT) { track.Clear(); trackThr.Clear(); funTrack.Clear(); predTrail.Clear(); pipUt = float.NaN; lastPredT = -1e30f; }   // a new flight
            lastT = t;
            if (S(kv, "pip") == "1" && float.IsNaN(pipUt)) pipUt = t;
            if (pos.z < 3000f && (track.Count == 0 || (pos - (Vector3)track[track.Count - 1]).magnitude > Mathf.Max(0.3f, 0.01f * pos.z)))
            {
                track.Add(new Vector4(pos.x, pos.y, pos.z, t));
                trackThr.Add(Mathf.Clamp01(P(kv, "thr", 0f)));
                if (track.Count > TRACK_MAX) { Thin(track, 200); Thin(trackThr, 200); }   // same indices: stays aligned
            }
            if (pos.z < 40000f && (funTrack.Count == 0 || (pos - funTrack[funTrack.Count - 1]).magnitude > Mathf.Max(30f, 0.01f * pos.z)))
            {
                funTrack.Add(pos);
                if (funTrack.Count > FUN_MAX) Thin(funTrack, 100);
            }
            Vector3 pr;
            if (V3(kv, "pred", out pr) && t - lastPredT > 2f)
            {
                predTrail.Add(new Vector2(pr.x, pr.y)); lastPredT = t;
                if (predTrail.Count > 20) predTrail.RemoveAt(0);
            }
        }

        static void AscentUpdate(Dictionary<string, string> kv)
        {
            float[] a = Floats(kv, "asc", 6); double ut;
            if (a == null || !D(kv, "ut", out ut) || ut == lastAscUt) return;
            if (ut < lastAscUt - 1.0) ascTrack.Clear();                                     // a new flight
            lastAscUt = ut;
            var p = new Vector2(a[1], a[0]);
            if (ascTrack.Count == 0 || (p - ascTrack[ascTrack.Count - 1]).magnitude > 0.05f) ascTrack.Add(p);
            if (ascTrack.Count > ASC_MAX) Thin(ascTrack, 60);
        }

        // ---------- the phase widget (bottom right) on the unified show timeline ----------
        static float WX, WY = 600f, WW = 340f, WH = 330f;
        static readonly float YAW = 35f * Mathf.Deg2Rad, PIT = 24f * Mathf.Deg2Rad;     // FIXED viewing direction

        static Dictionary<string, string> txKv;
        static Dictionary<string, double[]> txCache;
        static Dictionary<string, double[]> ParseTx(Dictionary<string, string> kv)
        {   // hud_tx=NAME@ut@dur|...  (parsed once per overlay)
            if (ReferenceEquals(kv, txKv)) return txCache;
            txKv = kv;
            var r = txCache = new Dictionary<string, double[]>();
            foreach (string e in S(kv, "hud_tx").Split('|'))
            {
                string[] f = e.Split('@'); double u, d;
                if (f.Length >= 3 && double.TryParse(f[1], NumberStyles.Float, CultureInfo.InvariantCulture, out u)
                    && double.TryParse(f[2], NumberStyles.Float, CultureInfo.InvariantCulture, out d)) r[f[0]] = new[] { u, Math.Max(d, 0.05) };
            }
            return r;
        }

        static float Prog(Dictionary<string, double[]> tx, string name)
        {   // -1: not started (or not planned); 0..1 animation progress
            double[] t;
            if (!tx.TryGetValue(name, out t) || UT < t[0]) return -1f;
            return (float)Math.Min((UT - t[0]) / t[1], 1.0);
        }

        static void Widgets(Dictionary<string, string> kv, bool paint, float pOut)
        {
            var tx = ParseTx(kv);
            float pA = Prog(tx, "ASCENT_IN"), pO = Prog(tx, "ORBIT_ICON"), pF = Prog(tx, "FUNNEL_IN"), pH = Prog(tx, "HANDOFF"), pT = Prog(tx, "TD_PLOT");
            if (pF < 0f && pH >= 0f) pF = 1f;
            if (pO < 0f && pA < 0f && kv.ContainsKey("orb")) pO = 1f;     // checkpoint run from orbit: the icon at once
            float baseA = gA, so = pOut * 460f;
            if (pF < 0f)
            {
                if (pO < 0f) { if (pA >= 0f) { gOff = new Vector2((1f - Ease(pA)) * 460f + so, 0f); Box(paint); AscentContent(kv, paint); } }
                else if (pO < 1f) { gOff = new Vector2(so, 0f); Shrink(kv, paint, Ease(pO), tx); }
                else { gOff = new Vector2(so, 0f); OrbitIcon(kv, paint, tx); }
            }
            else
            {
                float io = 1f - Ease(pF);                                      // the orbit icon gives way to the funnel
                if (io > 0f && pO >= 0f) { gA = baseA * io; gOff = new Vector2(so, 0f); OrbitIcon(kv, paint, tx); gA = baseA; }
                gOff = new Vector2((1f - Ease(pF)) * 460f + so, 0f);
                if (pH < 0f) { Box(paint); FunnelContent(kv, paint, false); }
                else if (pH < 1f) Handoff(kv, paint, pH);
                else ConvBox(kv, paint, pT >= 0f);
                if (pT >= 0f)
                {   // touchdown plot: slides in left of CONVERGENCE and stays
                    float keepX = WX;
                    WX = W / 2 + 530f - WW - 16f;
                    gA = baseA * Ease(pT); gOff = new Vector2((1f - Ease(pT)) * 40f + so, 0f);
                    TouchdownBox(kv, paint);
                    WX = keepX; gA = baseA;
                }
            }
            gA = baseA; gOff = Vector2.zero;
        }

        static void Box(bool paint)
        {
            if (paint) { BeginGL(); RoundRect(WX, WY, WW, WH, 10f, new Color(0.03f, 0.04f, 0.055f, 0.32f), Wa(0.22f)); EndGL(); }
        }

        static void Head(string title, string right)
        {
            L(WX + 14, WY + 10, 200, 18, Sp(title), sWinL());
            if (right.Length > 0) L(WX + WW - 214, WY + 10, 200, 18, Sp(right), sWinR());
        }

        static string Km(float v) { return (Mathf.Abs(v) < 100f ? v.ToString("F1", CultureInfo.InvariantCulture) : v.ToString("F0", CultureInfo.InvariantCulture)).Replace("-", "−"); }
        static string M_(float m)
        {
            if (float.IsNaN(m)) return "—";
            return m >= 1000f ? (m / 1000f).ToString("F2", CultureInfo.InvariantCulture) + " KM"
                : m.ToString(m < 10f ? "F2" : m < 100f ? "F1" : "F0", CultureInfo.InvariantCulture) + " M";
        }

        // ASCENT: altitude vs downrange flown, target orbit line, ballistic coast arc to the apoapsis
        static void AscentContent(Dictionary<string, string> kv, bool paint)
        {
            float[] a = Floats(kv, "asc", 6);
            Head("ASCENT", a == null ? "" : "AP " + Km(a[4]) + " KM");
            if (a == null) return;
            float alt = a[0], x = a[1], vh = a[2], vv = a[3], tgt = P(kv, "target", 100f);
            float x0 = WX + 46, x1 = WX + WW - 22, y0 = WY + WH - 44, y1 = WY + 56;
            const float g = 9.81f;
            var coast = scr0; scr0.Clear();
            float tv = Mathf.Max(vv, 0f) / g;
            for (int i = 0; i <= 24; i++) { float t = tv * i / 24; coast.Add(new Vector2(x + vh * t / 1e3f, alt + (vv * t - g * t * t / 2f) / 1e3f)); }
            Vector2 ce = coast[coast.Count - 1];
            float xm = Mathf.Max(40f, 1.25f * ce.x);
            Func<float, float> X = v => x0 + (x1 - x0) * v / xm;
            Func<float, float> Y = h => y0 - (y0 - y1) * Mathf.Sqrt(Mathf.Max(h, 0f) / 120f);
            if (paint)
            {
                BeginGL();
                Seg(new Vector2(x0, y0), new Vector2(x1, y0), 1f, Wa(0.6f));
                foreach (float h in new[] { 10f, 50f }) Seg(new Vector2(x0 - 4, Y(h)), new Vector2(x0, Y(h)), 1f, Wa(0.6f));
                Dash(new Vector2(x0, Y(tgt)), new Vector2(x1, Y(tgt)), 1f, Wa(0.75f), 5f, 4f);
                var path = scr1; scr1.Clear();
                foreach (var p in ascTrack) path.Add(new Vector2(X(p.x), Y(p.y)));
                path.Add(new Vector2(X(x), Y(alt)));
                Poly(path, 2.4f, HI);
                var cp = scr2; scr2.Clear();
                foreach (var p in coast) cp.Add(new Vector2(X(p.x), Y(p.y)));
                DashPoly(cp, 1.2f, Wa(0.7f), 2f, 4f);
                Seg(new Vector2(X(ce.x) - 7, Y(ce.y)), new Vector2(X(ce.x) + 7, Y(ce.y)), 1.6f, HI);
                Disc(new Vector2(X(x), Y(alt)), 4.5f, HI);
                EndGL();
            }
            foreach (float h in new[] { 10f, 50f }) L(x0 - 48, Y(h) - 7, 40, 14, h.ToString("F0", CultureInfo.InvariantCulture), sSparkEnd());
            L(x0 - 48, Y(tgt) - 7, 40, 14, tgt.ToString("F0", CultureInfo.InvariantCulture), sSparkEnd());
            L(x1 - 200, Y(tgt) - 20, 200, 14, Sp("TARGET ORBIT"), sSparkEnd());
            L(x0 - 48, y1 - 17, 40, 14, "KM", sSparkEnd());
            L(x1 - 260, y0 + 9, 260, 14, Sp("DOWNRANGE  0–" + xm.ToString("F0", CultureInfo.InvariantCulture) + " KM"), sSparkEnd());
            L(X(ce.x) + 11, Y(ce.y) - 7, 40, 14, "AP", sSpark);
            L(WX + 14, WY + WH - 21, 200, 14, Sp("PE " + Km(a[5]) + " KM"), sSpark);
        }

        static readonly List<Vector2> scr0 = new List<Vector2>(), scr1 = new List<Vector2>(), scr2 = new List<Vector2>();   // per-frame scratch
        static float OrbX() { return W / 2 + 530f + BW + 62f; }
        static float[] orbLast;
        static double orbUt = double.NaN;
        const float ORB_Y = 1003f, ORB_R = 36f;

        // ORBIT_ICON transition: the ascent window shrinks into the 72 px orbit icon beside the right table
        static void Shrink(Dictionary<string, string> kv, bool paint, float e, Dictionary<string, double[]> tx)
        {
            float ox = OrbX(), oy = ORB_Y, keepA = gA;
            float bx = Mathf.Lerp(WX, ox - ORB_R, e), by = Mathf.Lerp(WY, oy - ORB_R, e), bw = Mathf.Lerp(WW, 2 * ORB_R, e), bh = Mathf.Lerp(WH, 2 * ORB_R, e);
            if (paint) { BeginGL(); RoundRect(bx, by, bw, bh, Mathf.Lerp(10f, ORB_R, e), new Color(0.03f, 0.04f, 0.055f, 0.32f * (1f - e)), Wa(Mathf.Lerp(0.22f, 0.6f, e))); EndGL(); }
            if (e < 0.5f) { gA = keepA * (1f - 2f * e); AscentContent(kv, paint); }
            else { gA = keepA * (2f * e - 1f); OrbitIcon(kv, paint, tx); }
            gA = keepA;
        }

        // minimal orbit icon (no box, no text): Kerbin, the orbit, the deorbit node, the stage with a short trail;
        // PAYLOAD_SPLIT: the dot splits once (a small ring pulse, the payload drifts a little ahead and fades)
        static void OrbitIcon(Dictionary<string, string> kv, bool paint, Dictionary<string, double[]> tx)
        {
            if (!paint) return;
            float ox = OrbX(), oy = ORB_Y, r = ORB_R;
            Func<float, Vector2> Pp = th => new Vector2(ox + r * Mathf.Cos(th * Mathf.Deg2Rad), oy - r * Mathf.Sin(th * Mathf.Deg2Rad));
            float[] o = Floats(kv, "orb", 3);
            double utw;
            if (o != null && D(kv, "ut", out utw)) { orbLast = o; orbUt = utw; }
            else { o = orbLast; utw = orbUt; }              // the lander's feed has no orbit data: keep moving the last
            float u = float.NaN;
            if (o != null && !float.IsNaN(o[0]))
                u = o[0] + (o[2] > 0f && !double.IsNaN(utw) ? (float)(360.0 * (UT - utw) / o[2]) : 0f);
            BeginGL();
            Disc(new Vector2(ox, oy), 11f, Wa(0.18f));
            Arc(ox, oy, 11f, 1f, 0f, 360f, Wa(0.55f), false);
            Arc(ox, oy, r, 1f, 0f, 360f, Wa(0.6f), false);
            if (o != null && !float.IsNaN(o[1])) Diamond(Pp(o[1]), 4f, 1.2f);
            if (!float.IsNaN(u))
            {
                for (int k = 1; k <= 8; k++) Seg(Pp(u - 3 * k), Pp(u - 3 * (k - 1)), 1.6f, Wa(0.55f * (1f - k / 9f)));
                double[] sp;
                if (tx.TryGetValue("PAYLOAD_SPLIT", out sp) && UT >= sp[0])
                {
                    float s = (float)(UT - sp[0]), fade = 1f - Mathf.Clamp01((s - 20f) / 10f);
                    if (fade > 0f) Disc(Pp(u + Mathf.Min(1.5f * s, 8f)), 1.8f, Wa(0.7f * fade));
                    float pc = s / (float)sp[1];
                    if (pc < 1f) Arc(Pp(u).x, Pp(u).y, 3f + 8f * pc, 1f, 0f, 360f, Wa(0.8f * (1f - pc)), false);
                }
                Disc(Pp(u), 3.2f, HI);
            }
            EndGL();
        }

        // ENTRY CORRIDOR: oblique wireframe funnel 40 km -> 1 km (radius = reachable corridor r = 1 km + 3 (h - 1 km)),
        // power-law screen scales; live track inside, predicted impact converging at the mouth
        const float FNK = 16.5f;
        static readonly Vector3[] FunRings = { new Vector3(40, 0.6f, 1.1f), new Vector3(20, 0.3f, 1f), new Vector3(10, 0.3f, 1f), new Vector3(5, 0.3f, 1f), new Vector3(2, 0.3f, 1f), new Vector3(1, 0.85f, 1.4f) };   // km, alpha, width
        static float FnTY() { return WY + 86f; }
        static float FnMY() { return WY + 268f; }
        static float FnX0() { return WX + WW / 2f; }
        static float Fy(float hkm) { return FnMY() - (FnMY() - FnTY()) * (Mathf.Pow(Mathf.Max(hkm, 1f), 0.4f) - 1f) / (Mathf.Pow(40f, 0.4f) - 1f); }
        static float Fr(float dkm) { return FNK * Mathf.Pow(Mathf.Max(dkm, 0f), 0.4f); }
        static float Cor(float hkm) { return 1f + 3f * (hkm - 1f); }
        static bool fzOn; static Vector2 fzO, fzT; static float fzS = 1f;      // hand-off zoom into the mouth
        static Vector2 FZ(Vector2 p) { return fzOn ? fzT + fzS * (p - fzO) : p; }
        static Vector2 FP(float e, float n, float h)
        {
            float s = Fr(Mathf.Sqrt(e * e + n * n)), a = Mathf.Atan2(n, e) + YAW;
            return FZ(new Vector2(FnX0() + s * Mathf.Cos(a), Fy(h) + s * Mathf.Sin(a) * Mathf.Sin(PIT)));
        }

        static void FunnelContent(Dictionary<string, string> kv, bool paint, bool noHead)
        {
            Vector3 pos, pr; bool hp = V3(kv, "pos", out pos), hpr = V3(kv, "pred", out pr);
            if (!noHead) Head("ENTRY CORRIDOR", "PRED MISS " + (hpr ? M_(Mathf.Sqrt(pr.x * pr.x + pr.y * pr.y)) : "—"));
            if (paint)
            {
                BeginGL();
                for (int k = 0; k < 12; k++)                                    // wire meridians
                {
                    float a = k * 30f * Mathf.Deg2Rad;
                    Vector2 prev = Vector2.zero;
                    for (int i = 0; i <= 24; i++)
                    {
                        float h = 40f * Mathf.Pow(1f / 40f, i / 24f);
                        Vector2 p = FP(Cor(h) * Mathf.Cos(a), Cor(h) * Mathf.Sin(a), h);
                        if (i > 0) Seg(prev, p, 1f, Wa(0.22f));
                        prev = p;
                    }
                }
                foreach (var ring in FunRings)
                {
                    Vector2 prev = Vector2.zero;
                    for (int k = 0; k <= 72; k++)
                    {
                        float a = k * 5f * Mathf.Deg2Rad, rr = Cor(ring.x);
                        Vector2 p = FP(rr * Mathf.Cos(a), rr * Mathf.Sin(a), ring.x);
                        if (k > 0) Seg(prev, p, ring.z, Wa(ring.y));
                        prev = p;
                    }
                }
                Dash(FZ(new Vector2(FnX0(), Fy(40f))), FZ(new Vector2(FnX0(), FnMY())), 1f, Wa(0.35f), 2f, 4f);   // pad axis
                var tr = scr0; scr0.Clear();
                foreach (var p in funTrack) tr.Add(FP(p.x / 1e3f, p.y / 1e3f, Mathf.Clamp(p.z / 1e3f, 1f, 40f)));
                if (hp && pos.z < 40000f) tr.Add(FP(pos.x / 1e3f, pos.y / 1e3f, Mathf.Clamp(pos.z / 1e3f, 1f, 40f)));
                Poly(tr, 1.8f, HI);
                for (int k = 0; k < predTrail.Count; k++)
                    Disc(FP(predTrail[k].x / 1e3f, predTrail[k].y / 1e3f, 1f), 1.5f / fzS, Wa(0.25f + 0.6f * k / Mathf.Max(predTrail.Count, 1)));
                if (hpr && fzS < 1.5f) Cross(FP(pr.x / 1e3f, pr.y / 1e3f, 1f), 7f, 2.5f);
                if (tr.Count > 0) Disc(tr[tr.Count - 1], 4.5f / fzS, HI);
                EndGL();
            }
            if (!noHead)
                foreach (float h in new[] { 40f, 10f, 1f })
                    L(FnX0() - Fr(Cor(h)) - 6f - 60f, Fy(h) - 7f, 60f, 14f, h.ToString("F0", CultureInfo.InvariantCulture) + " KM", sSparkEnd());
        }

        // CONVERGENCE: fixed 3-D landing volume, log radius 0.1 m .. 1 km (rings 1 / 10 / 100 m / 1 km), log height to
        // 1 km, FIXED view (no rotation, no auto-zoom); its dashed top ring is the funnel's 1 km mouth
        const float CVS = 140f;
        static float CvX0() { return WX + WW / 2f; }
        static float CvY0() { return WY + 228f; }
        static float Cq(float r) { return r <= 0.1f ? 0f : Mathf.Min(Mathf.Log10(r / 0.1f) / 4f, 1.08f); }
        static float Cz(float h) { return Mathf.Log10(1f + Mathf.Max(h, 0f) / 2f) / Mathf.Log10(501f); }
        static Vector2 CP(float e, float n, float h)
        {
            float q = Cq(Mathf.Sqrt(e * e + n * n)), a = Mathf.Atan2(n, e), x = q * Mathf.Cos(a), y = q * Mathf.Sin(a);
            float xr = x * Mathf.Cos(YAW) - y * Mathf.Sin(YAW), yr = x * Mathf.Sin(YAW) + y * Mathf.Cos(YAW);
            return new Vector2(CvX0() + CVS * xr, CvY0() + CVS * (yr * Mathf.Sin(PIT) - 0.95f * Cz(h) * Mathf.Cos(PIT)));
        }

        static void ConvBox(Dictionary<string, string> kv, bool paint, bool landed)
        {
            Box(paint);
            ConvContent(kv, paint, false, landed);
        }

        static void ConvContent(Dictionary<string, string> kv, bool paint, bool noHead, bool landed)
        {
            Vector3 pos, pr; bool hp = V3(kv, "pos", out pos), hpr = V3(kv, "pred", out pr);
            float miss = hpr ? Mathf.Sqrt(pr.x * pr.x + pr.y * pr.y) : (hp ? Mathf.Sqrt(pos.x * pos.x + pos.y * pos.y) : float.NaN);
            if (!noHead) Head("CONVERGENCE", landed ? "LANDED" : "MISS " + M_(miss));
            if (paint)
            {
                BeginGL();
                float[] rr = { 1f, 10f, 100f, 1000f };
                for (int i = 0; i < rr.Length; i++)
                {
                    var ring = scr0; scr0.Clear();
                    for (int k = 0; k <= 72; k++) { float a = k * 5f * Mathf.Deg2Rad; ring.Add(CP(rr[i] * Mathf.Cos(a), rr[i] * Mathf.Sin(a), 0f)); }
                    Poly(ring, 1.2f, Wa(i == 3 ? 0.75f : 0.45f));
                }
                var top = scr1; scr1.Clear();
                for (int k = 0; k <= 72; k++) { float a = k * 5f * Mathf.Deg2Rad; top.Add(CP(1000f * Mathf.Cos(a), 1000f * Mathf.Sin(a), 1000f)); }
                DashPoly(top, 1f, Wa(0.3f), 3f, 4f);                            // top of the volume = the funnel mouth
                Seg(CP(0, 0, 0), CP(0, 0, 1000f), 1f, Wa(0.6f));                 // height axis over the pad (log)
                foreach (float hh in new[] { 10f, 100f, 1000f }) { Vector2 p = CP(0, 0, hh); Seg(p - new Vector2(4, 0), p + new Vector2(4, 0), 1f, Wa(0.9f)); }
                var shadow = scr2; scr2.Clear();
                for (int i = 0; i < track.Count; i++) if (track[i].z <= 1000f) shadow.Add(CP(track[i].x, track[i].y, 0));
                DashPoly(shadow, 1f, Wa(0.45f), 3f, 4f);                         // ground shadow
                for (int i = 1; i < track.Count; i++)
                {
                    if (track[i - 1].z > 1000f) continue;
                    Seg(CP(track[i - 1].x, track[i - 1].y, track[i - 1].z), CP(track[i].x, track[i].y, track[i].z), 1.5f + 3f * trackThr[i], HI);   // width = throttle
                }
                if (hp)
                {
                    float hz = Mathf.Min(pos.z, 1000f);
                    Vector2 vp = CP(pos.x, pos.y, hz), gp = CP(pos.x, pos.y, 0);
                    if (hz > 1f) Dash(vp, gp, 1f, Wa(0.7f), 2f, 3f);
                    Disc(vp, 4.5f, HI);
                    if (hpr && !landed) Cross(CP(pr.x, pr.y, 0), 9f, 3f);
                }
                Disc(CP(0, 0, 0), 3f, HI);
                EndGL();
            }
            float la = 75f * Mathf.Deg2Rad - YAW;                                // ring labels: in front, right of centre
            string[] rl = { "10 M", "100 M", "1 KM" };
            float[] rv = { 10f, 100f, 1000f };
            for (int i = 0; i < 3; i++) { Vector2 p = CP(rv[i] * Mathf.Cos(la), rv[i] * Mathf.Sin(la), 0); L(p.x + 5, p.y + 2, 80, 14, rl[i], sSpark); }
            for (int i = 0; i < 3; i++) { Vector2 p = CP(0, 0, rv[i]); L(p.x - 88, p.y - 7, 80, 14, rl[i], sSparkEnd()); }
        }

        // HANDOFF at 1 km: the funnel zooms into its mouth until the mouth ring IS the dashed top ring of the landing
        // volume (same centre, same size, same fixed view); the funnel fades, the landing volume fades in
        static void Handoff(Dictionary<string, string> kv, bool paint, float p)
        {
            Box(paint);
            float z = Ease(p), keepA = gA;
            Vector3 pos, pr; bool hp = V3(kv, "pos", out pos), hpr = V3(kv, "pred", out pr);
            float miss = hpr ? Mathf.Sqrt(pr.x * pr.x + pr.y * pr.y) : (hp ? Mathf.Sqrt(pos.x * pos.x + pos.y * pos.y) : float.NaN);
            fzO = new Vector2(FnX0(), FnMY());
            Vector2 tgt = CP(0, 0, 1000f);
            fzT = fzO + (tgt - fzO) * z; fzS = Mathf.Pow(CVS / FNK, z);
            clipOn = true; clipR = new UnityEngine.Rect(WX, WY, WW, WH);
            gA = keepA * (1f - Ease((p - 0.35f) / 0.5f));
            if (gA > 0f) { fzOn = true; FunnelContent(kv, paint, true); fzOn = false; }
            fzS = 1f;
            gA = keepA * Ease((p - 0.45f) / 0.5f);
            if (gA > 0f) ConvContent(kv, paint, true, false);
            clipOn = false;
            gA = keepA * (1f - Ease(p / 0.5f)); if (gA > 0f) Head("ENTRY CORRIDOR", "PRED MISS " + M_(miss));
            gA = keepA * Ease((p - 0.5f) / 0.5f); if (gA > 0f) Head("CONVERGENCE", "MISS " + M_(miss));
            gA = keepA;
        }

        // TOUCHDOWN: top-down, actual touchdown point vs pad centre (log rings 1 / 10 / 100 m), last 60 m of the track
        static void TouchdownBox(Dictionary<string, string> kv, bool paint)
        {
            Vector3 pos; bool hp = V3(kv, "pos", out pos);
            float miss = hp ? Mathf.Sqrt(pos.x * pos.x + pos.y * pos.y) : float.NaN;
            Box(paint);
            Head("TOUCHDOWN", "MISS " + M_(miss));
            float ox = WX + WW / 2f, oy = WY + WH / 2f + 16f, Sc = 126f;
            Func<float, float> Q = r => r <= 0.1f ? 0f : Mathf.Min(Mathf.Log10(r / 0.1f) / 3f, 1.06f);
            Func<float, float, Vector2> Pt = (e, n) => { float q = Q(Mathf.Sqrt(e * e + n * n)), a = Mathf.Atan2(n, e); return new Vector2(ox + Sc * q * Mathf.Cos(a), oy - Sc * q * Mathf.Sin(a)); };
            float[] rv = { 1f, 10f, 100f };
            string[] rl = { "1 M", "10 M", "100 M" };
            if (paint)
            {
                BeginGL();
                for (int i = 0; i < 3; i++) Arc(ox, oy, Sc * Q(rv[i]), 1.1f, 0f, 360f, Wa(i == 2 ? 0.7f : 0.45f), false);
                Seg(new Vector2(ox - Sc * 1.06f, oy), new Vector2(ox + Sc * 1.06f, oy), 1f, Wa(0.18f));
                Seg(new Vector2(ox, oy - Sc * 1.06f), new Vector2(ox, oy + Sc * 1.06f), 1f, Wa(0.18f));
                var tr = scr0; scr0.Clear();
                foreach (var t in track) if (t.z <= 60f) tr.Add(Pt(t.x, t.y));
                DashPoly(tr, 1f, Wa(0.4f), 2f, 3f);
                Disc(new Vector2(ox, oy), 2.5f, Wa(0.8f));
                if (hp)
                {
                    Vector2 p = Pt(pos.x, pos.y);
                    Seg(new Vector2(ox, oy), p, 1f, Wa(0.5f));
                    Arc(p.x, p.y, 6f, 1.6f, 0f, 360f, HI, false);
                    Disc(p, 2.2f, HI);
                }
                EndGL();
            }
            for (int i = 0; i < 3; i++) { float rr = Sc * Q(rv[i]); L(ox + rr * 0.7f + 4f, oy + rr * 0.71f + 3f, 60, 14, rl[i], sSpark); }
            L(ox + Sc * 1.06f - 22f, oy - 13f, 20, 12, "E", sSparkEnd());
            L(ox + 4f, oy - Sc * 1.06f + 2f, 20, 12, "N", sSpark);
        }

        static GUIStyle sSparkE;
        static GUIStyle sSparkEnd() { return sSparkE ?? (sSparkE = St(10, MID, TextAnchor.MiddleRight)); }

        static float MapY(HudSeries s, float v)
        {
            if (s.scale == "log")
            {
                float lo = Mathf.Log10(Mathf.Max(s.ymin, 1e-3f)), hi = Mathf.Log10(Mathf.Max(s.ymax, s.ymin * 10f));
                return Mathf.Clamp01((Mathf.Log10(Mathf.Max(Mathf.Abs(v), s.ymin)) - lo) / (hi - lo));
            }
            return Mathf.Clamp01((v - s.ymin) / Mathf.Max(s.ymax - s.ymin, 1e-6f));
        }

        static void Sparks(IList<HudSeries> plots, float x0, float y0, bool paint)
        {   // research sparklines, only with plots=on (research recordings)
            const float gw = 240f, gh = 34f;
            for (int k = 0; k < plots.Count && k < 3; k++)
            {
                HudSeries s = plots[k];
                float y = y0 + k * 64f;
                int n = s.pts.Count;
                float last = n > 0 ? s.pts[n - 1].y : float.NaN;
                L(x0, y - 16, 180, 14, Sp(s.title), sSpark);
                L(x0 + gw - 100, y - 16, 100, 14, float.IsNaN(last) ? "—" : last.ToString(Mathf.Abs(last) >= 100 ? "N0" : "F1", CultureInfo.InvariantCulture), sSparkR);
                if (!paint || n < 2) continue;
                BeginGL();
                float ta = s.pts[0].x, tb = Mathf.Max(s.pts[n - 1].x, ta + 1e-3f);
                int stepN = Mathf.Max(1, n / 240);
                Vector2 prev = Vector2.zero; bool first = true;
                for (int i = 0; i < n; i += stepN)
                {
                    var p = new Vector2(x0 + gw * (s.pts[i].x - ta) / (tb - ta), y + gh - MapY(s, s.pts[i].y) * gh);
                    if (!first) Seg(prev, p, 1.2f, new Color(1, 1, 1, 0.85f));
                    prev = p; first = false;
                }
                EndGL();
            }
        }
    }
}
