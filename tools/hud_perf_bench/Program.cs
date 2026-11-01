// Offline microbenchmark of the AutoLab overlay parsing path (old: every OnGUI event; new: once per file change)
// plus a GL batch count model of the HUD draw (old: one GL.Begin/End per primitive; new: per run of one kind).
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Globalization;
using System.IO;

static class B
{
    static Dictionary<string, string> Parse(string overlay)
    {
        var kv = new Dictionary<string, string>();
        foreach (string line in overlay.Split('\n'))
        {
            int i = line.IndexOf('=');
            if (i > 0) kv[line.Substring(0, i).Trim()] = line.Substring(i + 1).Trim();
        }
        return kv;
    }

    static string S(Dictionary<string, string> kv, string k) { string s; return kv.TryGetValue(k, out s) ? s : ""; }

    // the per-event string work the old HudV2.Draw did on top of the dictionary (evx, hud_tx, rows, asc, Sp, sort)
    static int HudParse(Dictionary<string, string> kv, double UT)
    {
        int n = 0;
        var evs = new List<KeyValuePair<string, float>>();
        foreach (string e in S(kv, "evx").Split('|'))
        {
            string[] f = e.Split(';'); double u;
            if (f.Length < 6 || !double.TryParse(f[1], NumberStyles.Float, CultureInfo.InvariantCulture, out u)) continue;
            evs.Add(new KeyValuePair<string, float>(f[0], (float)(u - UT)));
        }
        evs.Sort((a, b) => a.Value.CompareTo(b.Value));
        var tx = new Dictionary<string, double[]>();
        foreach (string e in S(kv, "hud_tx").Split('|'))
        {
            string[] f = e.Split('@'); double u, d;
            if (f.Length >= 3 && double.TryParse(f[1], NumberStyles.Float, CultureInfo.InvariantCulture, out u)
                && double.TryParse(f[2], NumberStyles.Float, CultureInfo.InvariantCulture, out d)) tx[f[0]] = new[] { u, d };
        }
        foreach (string side in new[] { "left", "right" })
            foreach (string r in S(kv, side).Split('|')) n += r.Split(';').Length;
        for (int k = 0; k < 2; k++)       // AscentContent + AscentUpdate
        {
            string[] a = S(kv, "asc").Split(',');
            var r = new float[6];
            for (int i = 0; i < 6; i++) float.TryParse(a[i], NumberStyles.Float, CultureInfo.InvariantCulture, out r[i]);
        }
        foreach (var e in evs)          // Sp() of every arc label (uncached)
        {
            var sb = new System.Text.StringBuilder();
            foreach (char c in e.Key.ToUpperInvariant()) { if (sb.Length > 0) sb.Append(' '); sb.Append(c.ToString()); }
            n += sb.ToString().Length;
        }
        return n + evs.Count + tx.Count;
    }

    static double Time(Action a, int iters)
    {
        for (int i = 0; i < iters / 10; i++) a();
        var sw = Stopwatch.StartNew();
        for (int i = 0; i < iters; i++) a();
        return sw.Elapsed.TotalMilliseconds * 1000.0 / iters;   // us per call
    }

    // --- GL batch model: primitives in drawing order, 'Q' quad run, 'T' triangle run ---
    // old: Seg with halo = 2 Begin/End, Disc = 1, Arc = 1 (+caps); new: one per change of kind
    static void Batches(string name, int segs, int discs, int arcs, int interleave)
    {
        long oldB = 2L * segs + discs + arcs;
        long newB = 1 + 2L * interleave;    // a disc between line runs splits the QUADS batch
        Console.WriteLine("  {0,-34} segments {1,5}  old GL.Begin/End {2,6}  new {3,4}", name, segs, oldB, newB);
    }

    static void Main(string[] args)
    {
        string text = File.ReadAllText(args[0]);
        const double UT = 100000.0;
        int N = 200000;
        double tSplit = Time(() => Parse(text), N);
        var kv = Parse(text);
        double tHud = Time(() => HudParse(kv, UT), N);
        string copy = new string(text.ToCharArray());
        double tCmp = Time(() => { if (copy != text) throw new Exception(); }, N * 10);
        Console.WriteLine("overlay {0} bytes", text.Length);
        Console.WriteLine("parse dictionary         {0,8:F2} us", tSplit);
        Console.WriteLine("HUD string parsing       {0,8:F2} us", tHud);
        Console.WriteLine("string compare (no change) {0,6:F3} us", tCmp);
        // old: per OnGUI event (Layout + Repaint = 2 per frame, more with mouse input); new: compare at 5 Hz + parse at 4 Hz
        foreach (int fps in new[] { 15, 30, 60 })
        {
            double oldPerSec = fps * 2 * (tSplit + tHud);
            double newPerSec = 5 * tCmp + 4 * (tSplit + tHud);
            Console.WriteLine("  {0} fps: old {1,7:F0} us/s ({2:F3} ms/frame)   new {3,5:F0} us/s ({4:F4} ms/frame)",
                fps, oldPerSec, oldPerSec / fps / 1000, newPerSec, newPerSec / fps / 1000);
        }
        // GC garbage per call (bytes)
        long g0 = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 1000; i++) { var d = Parse(text); HudParse(d, UT); }
        long g1 = GC.GetAllocatedBytesForCurrentThread();
        Console.WriteLine("garbage per OnGUI event (old)  {0} bytes", (g1 - g0) / 1000);

        Console.WriteLine("GL batch model (per frame, from the HudV2 code paths):");
        // timeline arc: 120 arc segments + 10 minute ticks + 10 event discs (2 for future ones) + leader lines + now marker
        Batches("timeline arc", 120 + 10 + 2, 16, 0, 2);
        // ascent widget: axis + 2 ticks + target dash (~31 dashes) + track (N) + coast dashes (~17) + apex + disc
        foreach (int n in new[] { 50, 500, 1500 })
            Batches("ascent widget, track " + n + " pts", 1 + 2 + 31 + n + 17 + 1, 1, 8, 1);
        // funnel: 12 meridians x 24 + 6 rings x 72 + axis dashes + track (600) + pred discs
        Batches("entry funnel, track 600 pts", 12 * 24 + 6 * 72 + 30 + 600, 21, 0, 3);
        // convergence: 4 rings x 72 + top dashes + axis + ticks + shadow dashes + track (800)
        Batches("convergence, track 800 pts", 4 * 72 + 50 + 4 + 200 + 800, 3, 0, 3);
        Batches("convergence, old cap 2000 pts", 4 * 72 + 50 + 4 + 500 + 2000, 3, 0, 3);
    }
}
