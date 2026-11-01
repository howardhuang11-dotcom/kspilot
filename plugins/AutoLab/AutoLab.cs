// AutoLab: headless test-bench helper for the kspilot autopilot.
// At the main menu it loads (or creates) the sandbox save "AutoPilotLab"
// so the game can be driven entirely over kRPC with no UI interaction.
using System;
using System.IO;
using UnityEngine;

namespace AutoLab
{
    // Headless operation: log and dismiss any modal popup (pre-flight warnings,
    // addon notices) so automated runs never block on UI.
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class PopupKiller : MonoBehaviour
    {
        float next;
        // reflection result per type (perf): in flight the scan visits thousands of part modules every 2 s
        static readonly System.Collections.Generic.Dictionary<Type, System.Reflection.MethodInfo> promo =
            new System.Collections.Generic.Dictionary<Type, System.Reflection.MethodInfo>();

        void Update()
        {
            if (Time.realtimeSinceStartup < next) return;
            next = Time.realtimeSinceStartup + 2f;
            // DLC advert / promo windows: any behaviour with a "don't show again" toggle
            foreach (MonoBehaviour mb in FindObjectsOfType<MonoBehaviour>())
            {
                System.Reflection.MethodInfo m;
                Type ty = mb.GetType();
                if (!promo.TryGetValue(ty, out m))
                    promo[ty] = m = ty.GetMethod("OnDontShowAgainToggle",
                        System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic);
                if (m == null || !mb.gameObject.activeInHierarchy) continue;
                try
                {
                    Debug.Log("[AutoLab] closing promo " + mb.GetType().FullName);
                    if (m.GetParameters().Length == 1 && m.GetParameters()[0].ParameterType == typeof(bool))
                        m.Invoke(mb, new object[] { true });
                    Transform root = mb.transform;
                    while (root.parent != null && root.parent.GetComponent("Canvas") == null)
                        root = root.parent;
                    root.gameObject.SetActive(false);
                }
                catch (Exception e) { Debug.LogError("[AutoLab] promo " + e.Message); }
            }
            foreach (PopupDialog p in FindObjectsOfType<PopupDialog>())
            {
                try
                {
                    MultiOptionDialog d = p.dialogToDisplay;
                    Debug.Log("[AutoLab] dismissing popup: " + (d != null ? d.title + " | " + d.message : "?"));
                    p.Dismiss();
                }
                catch (Exception e) { Debug.LogError("[AutoLab] popup " + e.Message); }
            }
        }
    }

    // Screenshot on request: write a target .png path into
    // GameData/AutoLab/shot.req and the next frame is captured there.
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class Shooter : MonoBehaviour
    {
        float next;

        void Update()
        {
            if (Time.realtimeSinceStartup < next) return;
            next = Time.realtimeSinceStartup + 0.5f;
            string req = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/shot.req");
            if (!File.Exists(req)) return;
            try
            {
                string target = File.ReadAllText(req).Trim();
                File.Delete(req);
                bool clean = target.StartsWith("clean:");
                if (clean) target = target.Substring(6);
                StartCoroutine(Capture(target, clean));
            }
            catch (Exception e) { Debug.LogError("[AutoLab] shot " + e.Message); }
        }

        // clean = hide the game UI for the shot (presentation screenshots)
        System.Collections.IEnumerator Capture(string target, bool clean)
        {
            if (clean) GameEvents.onHideUI.Fire();
            yield return new WaitForEndOfFrame();
            yield return new WaitForEndOfFrame();
            ScreenCapture.CaptureScreenshot(target);
            yield return new WaitForEndOfFrame();
            if (clean && !Recorder.Active) GameEvents.onShowUI.Fire();   // a recording keeps the stock UI hidden
            Debug.Log("[AutoLab] screenshot -> " + target);
        }
    }

    // Video recorder + telemetry overlay.
    //  rec.req   : "<dir>|<fps>" starts recording JPG frames (game UI hidden)
    //  rec.stop  : stops recording
    //  rec.pause : while this file exists no frames are WRITTEN; nothing else changes (capture rate, game time,
    //              physics, control). The director cuts dead air with it; frame numbers stay contiguous.
    //  overlay.txt: text drawn in the lower-left corner (written by kspilot)
    //  rec.req "<dir>|<fps>|<pid>|v2" (recording pipeline v2, HOP_REC=full only, director v2 specification): also
    //              (a) one line per WRITTEN frame in <dir>/index.csv: frame, UT, timeScale, warp rate, warp mode,
    //                  capture rate, director shot -> tools/edit_video.py cuts the video by UT;
    //              (b) physics warp is compensated in the capture rate (captureFramerate = fps x timeScale, spec C1-B1):
    //                  every frame still advances 1/fps s of game time, so the entry at physics x2 plays at 1x.
    //              Without "v2" nothing of this runs (byte-identical behaviour to before).
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class Recorder : MonoBehaviour
    {
        static string dir;
        static float interval;
        static int frame;
        static bool paused;
        /// <summary>A recording is running (other addons must not re-show the stock UI).</summary>
        public static bool Active { get { return dir != null; } }
        static int ownerPid;            // process that requested the recording (rec.req "<dir>|<fps>|<pid>"), 0 = unknown
        static bool v2;                 // recording pipeline v2 (rec.req 4th field "v2")
        static int fps0;                // the requested capture rate
        static StreamWriter index;      // v2: <dir>/index.csv
        static float nextOwnerCheck;
        string overlay = "";
        // overlay parsed once per change (perf, 2026-10-21): OnGUI runs >= 2x per frame (Layout + Repaint + input
        // events) and used to split the whole file into a new dictionary every time; the file changes at 4 Hz
        System.Collections.Generic.Dictionary<string, string> kvCache = new System.Collections.Generic.Dictionary<string, string>();
        readonly System.Collections.Generic.List<HudSeries> plotList = new System.Collections.Generic.List<HudSeries>();
        float nextPoll;
        bool hudShown;
        float nextHide;
        GUIStyle style;

        // ---- perf log (every 10 s in flight or while recording): KSP.log "[AutoLab] perf ..." + GameData/AutoLab/perf.txt
        //  fps, frame ms (avg / worst), game/real = game seconds per real second (< 0.95 at warp 1 = slow motion: frame
        //  time above Time.maximumDeltaTime, or a recording's fixed captureFramerate), HUD draw ms and GL batches
        float perfT0 = -1f, perfWorst, perfGame;
        int perfFrames, perfHudN, perfBatch, perfVert;
        double perfUt0 = double.NaN, perfHudMs, perfHudWorst;
        readonly System.Diagnostics.Stopwatch hudSw = new System.Diagnostics.Stopwatch();

        void PerfTick()
        {
            float now = Time.realtimeSinceStartup;
            if (perfT0 < 0f) { perfT0 = now; return; }
            perfFrames++;
            perfWorst = Mathf.Max(perfWorst, Time.unscaledDeltaTime);
            perfGame += Time.deltaTime;
            float span = now - perfT0;
            if (span < 10f) return;
            bool flight = HighLogic.LoadedScene == GameScenes.FLIGHT;
            double ut = double.NaN;
            if (flight || dir != null)
            {
                double utRate = double.NaN;
                try { if (flight) { ut = Planetarium.GetUniversalTime(); utRate = (ut - perfUt0) / span; } } catch (Exception) { }
                float warp = flight ? TimeWarp.CurrentRate : 1f;
                float gr = perfGame / span;
                string rec = dir == null ? "off" : paused ? "paused" : "ON";
                string line = string.Format(System.Globalization.CultureInfo.InvariantCulture,
                    "perf fps={0:F1} frame_ms avg={1:F1} max={2:F1} game/real={3:F2} ut/real={4:F2} warp={5:F0} timeScale={6:F2} " +
                    "maxDT={7:F3} fixedDT={8:F3} captureFramerate={9} rec={10} hud_ms avg={11:F2} max={12:F2} gl_batches={13} gl_verts={14}{15}",
                    perfFrames / span, 1000f * span / Mathf.Max(perfFrames, 1), 1000f * perfWorst, gr, utRate, warp, Time.timeScale,
                    Time.maximumDeltaTime, Time.fixedDeltaTime, Time.captureFramerate, rec,
                    perfHudN > 0 ? perfHudMs / perfHudN : 0.0, perfHudWorst, perfHudN > 0 ? perfBatch / perfHudN : 0,
                    perfHudN > 0 ? perfVert / perfHudN : 0,
                    warp <= 1.01f && Time.timeScale > 0.99f && gr < 0.95f
                        ? (Time.captureFramerate > 0 ? "  SLOW-MO: recording at a fixed " + Time.captureFramerate + " fps, game runs at fps/" + Time.captureFramerate
                                                     : "  SLOW-MO: frame time above maxDT") : "");
                Debug.Log("[AutoLab] " + line);
                try { File.WriteAllText(F("perf.txt"), DateTime.Now.ToString("HH:mm:ss") + " " + line + "\n"); } catch (Exception) { }
            }
            perfUt0 = ut;
            perfT0 = now; perfFrames = 0; perfWorst = 0f; perfGame = 0f;
            perfHudN = 0; perfHudMs = 0; perfHudWorst = 0; perfBatch = 0; perfVert = 0;
        }

        // The recording's owner process is gone (a killed run: SIGKILL skips atexit, so no rec.stop) -> stop it.
        // 2026-10-21: an orphaned recording (owner died 03:49) kept captureFramerate = 30 while capturing ~15 fps, so the
        // NEXT flight (03:53 own ascent) ran at ~0.45x game speed - first blamed on the HUD.
        // Platform note: the liveness probe is POSIX kill(pid, 0) from the macOS system library. On other platforms
        // the check is skipped (the recording then behaves as if the owner were alive, as before the guard existed).
        [System.Runtime.InteropServices.DllImport("/usr/lib/libSystem.B.dylib", EntryPoint = "kill", SetLastError = true)]
        static extern int sys_kill(int pid, int sig);
        static bool pidCheckBroken;
        static bool OwnerAlive(int pid)
        {
            if (pid <= 0 || pidCheckBroken) return true;
            if (Application.platform != RuntimePlatform.OSXPlayer && Application.platform != RuntimePlatform.OSXEditor)
                return true;
            try
            {
                if (sys_kill(pid, 0) == 0) return true;
                int err = System.Runtime.InteropServices.Marshal.GetLastWin32Error();
                return err != 3;                                  // ESRCH = no such process; EPERM etc.: it exists
            }
            catch (Exception e)
            {   // fail safe: without the check a recording behaves as before
                pidCheckBroken = true;
                Debug.LogWarning("[AutoLab] recording owner check unavailable (" + e.GetType().Name + "): orphan guard off");
                return true;
            }
        }

        void StopRecording(string why)
        {
            if (dir != null) Debug.Log("[AutoLab] recording stopped (" + why + "), frames=" + frame);
            dir = null;
            ownerPid = 0;
            paused = false;
            v2 = false;
            if (index != null) { try { index.Flush(); index.Close(); } catch (Exception) { } index = null; }
            if (File.Exists(F("rec.pause"))) File.Delete(F("rec.pause"));
            Time.captureFramerate = 0;
            GameEvents.onShowUI.Fire();
        }

        void ParseOverlay(string text)
        {   // a NEW dictionary per change: HudV2 keys its per-overlay caches on the reference
            var kv = new System.Collections.Generic.Dictionary<string, string>();
            if (text.StartsWith("HUD"))
                foreach (string line in text.Split('\n'))
                {
                    int i = line.IndexOf('=');
                    if (i > 0) kv[line.Substring(0, i).Trim()] = line.Substring(i + 1).Trim();
                }
            kvCache = kv;
            SampleSeries(kv);
            plotList.Clear();
            foreach (string id in order) plotList.Add(series[id]);
        }

        string F(string n) { return Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/" + n); }

        void Start() { if (dir != null) GameEvents.onHideUI.Fire(); }

        // v2 (C1-B1): physics warp (timeScale > 1) would make every captured frame cover timeScale/fps s of game time;
        // raising the capture rate by the same factor keeps it at exactly 1/fps s. Rails warp is not touched (time-lapse
        // footage); the game-time dynamics are unchanged, only real time per game second grows (recording slow motion).
        static void Compensate()
        {
            int want = fps0;
            try
            {
                if (TimeWarp.fetch != null && TimeWarp.WarpMode == TimeWarp.Modes.LOW && Time.timeScale > 1.01f)
                    want = Mathf.RoundToInt(fps0 * Mathf.Min(Time.timeScale, 8f));
            }
            catch (Exception) { want = fps0; }
            if (want > 0 && Time.captureFramerate != want)
            {
                Time.captureFramerate = want;
                Debug.Log("[AutoLab] recording v2: captureFramerate " + want + " (timeScale " + Time.timeScale.ToString("F2", System.Globalization.CultureInfo.InvariantCulture) + ")");
            }
        }

        void Update()
        {
            PerfTick();
            if (dir != null && v2) Compensate();
            if (Time.realtimeSinceStartup < nextPoll) return;
            nextPoll = Time.realtimeSinceStartup + 0.2f;
            try
            {
                string o = F("overlay.txt");
                string text = File.Exists(o) ? File.ReadAllText(o) : "";
                if (text != overlay) { overlay = text; ParseOverlay(text); }   // ~2 KB compare; parse at most 4x per second
                bool hud = overlay.StartsWith("HUD");
                // our HUD replaces the stock flight UI; re-hide every 2 s while it is shown (KSP rebuilds and
                // re-shows its UI some time after a save is loaded)
                if (dir == null && (hud != hudShown || (hud && Time.realtimeSinceStartup > nextHide)))
                {
                    if (hud) GameEvents.onHideUI.Fire(); else GameEvents.onShowUI.Fire();
                    hudShown = hud;
                    nextHide = Time.realtimeSinceStartup + 2f;
                }
                if (File.Exists(F("rec.req")))
                {
                    string[] a = File.ReadAllText(F("rec.req")).Trim().Split('|');
                    File.Delete(F("rec.req"));
                    dir = a[0];
                    interval = 1f / (a.Length > 1 ? float.Parse(a[1], System.Globalization.CultureInfo.InvariantCulture) : 30f);
                    if (a.Length < 3 || !int.TryParse(a[2], out ownerPid)) ownerPid = 0;
                    nextOwnerCheck = Time.realtimeSinceStartup + 2f;
                    frame = 0;
                    paused = false;
                    if (File.Exists(F("rec.pause"))) File.Delete(F("rec.pause"));   // stale pause from an aborted run
                    // fixed game-time step per rendered frame: every frame is captured, video is smooth
                    Time.captureFramerate = Mathf.RoundToInt(1f / interval);
                    Directory.CreateDirectory(dir);
                    v2 = a.Length > 3 && a[3].Trim() == "v2";
                    fps0 = Time.captureFramerate;
                    if (index != null) { try { index.Close(); } catch (Exception) { } index = null; }
                    if (v2)
                    {
                        index = new StreamWriter(Path.Combine(dir, "index.csv"));
                        index.WriteLine("frame,ut,timescale,warp,mode,capture,shot");
                    }
                    GameEvents.onHideUI.Fire();
                    Debug.Log("[AutoLab] recording -> " + dir + " (owner pid " + ownerPid + ", captureFramerate " + Time.captureFramerate + ")");
                }
                if (File.Exists(F("rec.stop")))
                {
                    File.Delete(F("rec.stop"));
                    StopRecording("rec.stop");
                }
                if (dir != null && ownerPid > 0 && Time.realtimeSinceStartup >= nextOwnerCheck)
                {
                    nextOwnerCheck = Time.realtimeSinceStartup + 2f;
                    if (!OwnerAlive(ownerPid)) StopRecording("ORPHAN: owner pid " + ownerPid + " has exited");
                }
                bool p = dir != null && File.Exists(F("rec.pause"));
                if (p != paused)
                {   // paused: frames are only not WRITTEN. The capture rate (game-time step per frame) stays as set at
                    // rec.req for the whole recording, so a pause/resume never changes how game time runs
                    // (mishap 2026-10-21 review: a pause must not touch time, physics or control)
                    paused = p;
                    Debug.Log("[AutoLab] recording " + (p ? "paused" : "resumed") + " at frame " + frame);
                }
            }
            catch (Exception e) { Debug.LogError("[AutoLab] rec " + e.Message); }
        }

        void LateUpdate()
        {
            if (dir != null && !paused) StartCoroutine(Grab());
        }

        System.Collections.IEnumerator Grab()
        {
            yield return new WaitForEndOfFrame();
            if (dir == null || paused) yield break;
            Texture2D t = ScreenCapture.CaptureScreenshotAsTexture();
            byte[] jpg = ImageConversion.EncodeToJPG(t, 85);
            Destroy(t);
            int f = frame++;
            File.WriteAllBytes(Path.Combine(dir, f.ToString("D6") + ".jpg"), jpg);
            if (v2 && index != null)
            {
                try
                {
                    double ut = Planetarium.GetUniversalTime();
                    string mode = TimeWarp.fetch == null ? "-" : TimeWarp.WarpMode == TimeWarp.Modes.HIGH ? "rails" : "phys";
                    index.WriteLine(string.Format(System.Globalization.CultureInfo.InvariantCulture, "{0},{1:F4},{2:F3},{3:F2},{4},{5},{6}",
                        f, ut, Time.timeScale, TimeWarp.CurrentRate, mode, Time.captureFramerate, CameraDirector.Current));
                    if (f % 30 == 0) index.Flush();
                }
                catch (Exception e) { Debug.LogWarning("[AutoLab] index " + e.Message); }
            }
        }

        GUIStyle big, small, label, phaseOn, phaseOff;
        Texture2D panel, rail, panelOn, marker;
        GUIStyle etaStyle;

        void InitStyles()
        {
            panel = new Texture2D(1, 1);
            panel.SetPixel(0, 0, new Color(0f, 0f, 0f, 0.55f));
            panel.Apply();
            panelOn = new Texture2D(1, 1);
            panelOn.SetPixel(0, 0, new Color(1f, 1f, 1f, 0.9f));
            panelOn.Apply();
            marker = new Texture2D(10, 10);
            for (int mxp = 0; mxp < 10; mxp++)
                for (int myp = 0; myp < 10; myp++)
                {
                    float d = Mathf.Sqrt((mxp - 4.5f) * (mxp - 4.5f) + (myp - 4.5f) * (myp - 4.5f));
                    marker.SetPixel(mxp, myp, new Color(1f, 1f, 1f, Mathf.Clamp01(5f - d)));
                }
            marker.Apply();
            rail = new Texture2D(1, 1);
            rail.SetPixel(0, 0, new Color(1f, 1f, 1f, 0.35f));
            rail.Apply();
            style = new GUIStyle(GUI.skin.box) { fontSize = 18, alignment = TextAnchor.UpperLeft, richText = true };
            style.normal.textColor = Color.white;
            big = new GUIStyle(GUI.skin.label) { fontSize = 34, fontStyle = FontStyle.Bold, alignment = TextAnchor.MiddleCenter };
            big.normal.textColor = Color.white;
            label = new GUIStyle(GUI.skin.label) { fontSize = 13, alignment = TextAnchor.MiddleCenter };
            label.normal.textColor = new Color(0.75f, 0.8f, 0.85f);
            small = new GUIStyle(GUI.skin.label) { fontSize = 15, alignment = TextAnchor.MiddleLeft };
            small.normal.textColor = Color.white;
            phaseOn = new GUIStyle(GUI.skin.label) { fontSize = 12, fontStyle = FontStyle.Bold, alignment = TextAnchor.UpperCenter, wordWrap = true };
            phaseOn.normal.textColor = new Color(1f, 1f, 1f);
            phaseOff = new GUIStyle(GUI.skin.label) { fontSize = 12, alignment = TextAnchor.UpperCenter, wordWrap = true };
            phaseOff.normal.textColor = new Color(0.55f, 0.6f, 0.65f);
            etaStyle = new GUIStyle(GUI.skin.label) { fontSize = 12, alignment = TextAnchor.UpperCenter };
            etaStyle.normal.textColor = new Color(0.45f, 0.85f, 1f);
        }

        // overlay.txt: either free text (legacy box) or "HUD" + key=value lines:
        //   title=..  speed=km/h  alt=km  phases=A|B|C|D  phase=<index>  info=<one line>
        void OnGUI()
        {
            if (string.IsNullOrEmpty(overlay)) { HudV2.Idle(); return; }
            if (style == null) InitStyles();
            if (!overlay.StartsWith("HUD"))
            {
                GUI.Box(new Rect(12, Screen.height - 132, 520, 120), overlay, style);
                return;
            }
            var kv = kvCache;                       // parsed in Update when the file changed
            string Get(string k) { string v; return kv.TryGetValue(k, out v) ? v : ""; }
            if (Get("style") != "classic")
            {   // HUD v2; the plot series are sampled in ParseOverlay
                bool rp = Event.current.type == EventType.Repaint;
                if (rp) { hudSw.Reset(); hudSw.Start(); HudV2.nBatch = HudV2.nVert = 0; }
                HudV2.Draw(kv, plotList);
                if (rp)
                {
                    hudSw.Stop();
                    double ms = hudSw.Elapsed.TotalMilliseconds;
                    perfHudN++; perfHudMs += ms; perfHudWorst = Math.Max(perfHudWorst, ms);
                    perfBatch += HudV2.nBatch; perfVert += HudV2.nVert;
                }
                return;
            }
            // layout designed for a 1400 px tall window: scale so the HUD keeps its proportions at any size
            float uiK = Screen.height / 1400f;
            Matrix4x4 keep = GUI.matrix;
            GUI.matrix = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, new Vector3(uiK, uiK, 1f));
            float W = Screen.width / uiK, H = Screen.height / uiK;
            // telemetry blocks, launch-webcast style: speed (left) and altitude (right) of the stage
            GUI.DrawTexture(new Rect(W / 2 - 450, H - 112, 900, 100), panel);
            GUI.Label(new Rect(W / 2 - 440, H - 108, 180, 20), "STAGE 1 · SPEED", label);
            GUI.Label(new Rect(W / 2 - 440, H - 88, 180, 50), Get("speed"), big);
            GUI.Label(new Rect(W / 2 - 440, H - 44, 180, 20), "KM/H", label);
            GUI.Label(new Rect(W / 2 + 260, H - 108, 180, 20), "ALTITUDE", label);
            GUI.Label(new Rect(W / 2 + 260, H - 88, 180, 50), Get("alt"), big);
            GUI.Label(new Rect(W / 2 + 260, H - 44, 180, 20), "KM", label);
            // mission timeline in the middle: one dot per phase, live time-to-go under each upcoming phase,
            // and a marker that slides along the rail with the mission progress (progress = index + fraction)
            string[] phases = Get("phases").Split('|');
            string[] etas = Get("eta").Split('|');
            int cur; int.TryParse(Get("phase"), out cur);
            float prog; if (!float.TryParse(Get("progress"), System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out prog)) prog = cur;
            float span = 420f, x0 = W / 2 - span / 2, step = phases.Length > 1 ? span / (phases.Length - 1) : 0f;
            GUI.DrawTexture(new Rect(x0, H - 83, span, 2), rail);
            GUI.DrawTexture(new Rect(x0, H - 83, Mathf.Clamp(prog, 0, phases.Length - 1) * step, 2), panelOn);
            for (int i = 0; i < phases.Length; i++)
            {
                float x = x0 + i * step;
                GUI.Label(new Rect(x - 8, H - 92, 16, 18), i <= cur ? "●" : "○", i == cur ? phaseOn : phaseOff);
                GUI.Label(new Rect(x - step / 2 + 2, H - 72, step - 4, 30), phases[i], i == cur ? phaseOn : phaseOff);
                if (i < etas.Length && etas[i].Length > 0 && i > cur)
                    GUI.Label(new Rect(x - step / 2 + 2, H - 40, step - 4, 18), etas[i], etaStyle);
            }
            float mx = x0 + Mathf.Clamp(prog, 0, phases.Length - 1) * step;
            GUI.DrawTexture(new Rect(mx - 5, H - 87, 10, 10), marker);
            string info = Get("info");
            if (info.Length > 0)
            {
                GUI.DrawTexture(new Rect(12, 12, 520, 28), panel);
                GUI.Label(new Rect(20, 12, 510, 28), info, small);
            }
            SidePanel(kv, W, H);
            GUI.matrix = keep;
        }

        // ---- research side panel: live time series ("plot.<id>=title|value|ymin|ymax|lin/log", sampled
        //      whenever t=<game time> changes) and the live controller parameters ("params=a: 1;b: 2")
        class Series : HudSeries { }
        System.Collections.Generic.Dictionary<string, Series> series = new System.Collections.Generic.Dictionary<string, Series>();
        System.Collections.Generic.List<string> order = new System.Collections.Generic.List<string>();
        string lastT = "";
        float lastTT = -1e30f;
        Texture2D dot, grid;
        GUIStyle tiny, tinyR, mono;

        static float Map(Series s, float v)
        {
            if (s.scale == "log")
            {
                float lo = Mathf.Log10(Mathf.Max(s.ymin, 1e-3f)), hi = Mathf.Log10(Mathf.Max(s.ymax, s.ymin * 10f));
                return Mathf.Clamp01((Mathf.Log10(Mathf.Max(Mathf.Abs(v), s.ymin)) - lo) / (hi - lo));
            }
            return Mathf.Clamp01((v - s.ymin) / Mathf.Max(s.ymax - s.ymin, 1e-6f));
        }

        void SampleSeries(System.Collections.Generic.Dictionary<string, string> kv)
        {   // plot series: one sample per new game time t (once per overlay change, from Update)
            string t; kv.TryGetValue("t", out t);
            bool sample = t != null && t != lastT;
            if (sample) lastT = t;
            float tt = 0f; if (t != null) float.TryParse(t, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out tt);
            if (sample && tt < lastTT) { series.Clear(); order.Clear(); }     // a new flight: start fresh plots
            if (sample) lastTT = tt;
            foreach (var e in kv)
            {
                if (!e.Key.StartsWith("plot.")) continue;
                string[] a = e.Value.Split('|');
                if (a.Length < 5) continue;
                Series sr;
                if (!series.TryGetValue(e.Key, out sr)) { sr = new Series(); series[e.Key] = sr; order.Add(e.Key); }
                sr.title = a[0];
                float.TryParse(a[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out sr.ymin);
                float.TryParse(a[3], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out sr.ymax);
                sr.scale = a[4];
                float v;
                if (sample && float.TryParse(a[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out v))
                {
                    sr.pts.Add(new Vector2(tt, v));
                    if (sr.pts.Count > 4000) sr.pts.RemoveRange(0, 1000);
                }
            }
        }

        void SidePanel(System.Collections.Generic.Dictionary<string, string> kv, float W, float H)
        {
            if (dot == null)
            {
                dot = new Texture2D(1, 1); dot.SetPixel(0, 0, new Color(0.35f, 0.85f, 1f, 1f)); dot.Apply();
                grid = new Texture2D(1, 1); grid.SetPixel(0, 0, new Color(1f, 1f, 1f, 0.12f)); grid.Apply();
                tiny = new GUIStyle(GUI.skin.label) { fontSize = 11, alignment = TextAnchor.UpperLeft };
                tiny.normal.textColor = new Color(0.8f, 0.85f, 0.9f);
                tinyR = new GUIStyle(tiny) { alignment = TextAnchor.UpperRight };
                tinyR.normal.textColor = Color.white;
                mono = new GUIStyle(tiny) { fontSize = 12, richText = true };
            }
            if (order.Count == 0) return;
            float pw = 290f, x0 = W - pw - 14, y = 14, ch = 66;
            string prm; kv.TryGetValue("params", out prm);
            string[] plines = string.IsNullOrEmpty(prm) ? new string[0] : prm.Split(';');
            float total = order.Count * (ch + 30) + 26 + (plines.Length > 0 ? 26 + plines.Length * 17 : 0);
            GUI.DrawTexture(new Rect(x0, y, pw, total), panel);
            GUI.Label(new Rect(x0 + 10, y + 4, pw - 20, 18), "GUIDANCE TELEMETRY · LIVE", label);
            y += 26;
            foreach (string id in order)
            {
                Series sr = series[id];
                string cur = sr.pts.Count > 0 ? sr.pts[sr.pts.Count - 1].y.ToString(Mathf.Abs(sr.pts[sr.pts.Count - 1].y) >= 100 ? "F0" : "F2") : "-";
                GUI.Label(new Rect(x0 + 10, y, pw - 90, 18), sr.title, tiny);
                GUI.Label(new Rect(x0 + pw - 90, y, 80, 18), cur, tinyR);
                Rect r = new Rect(x0 + 10, y + 18, pw - 20, ch);
                for (int g = 0; g <= 4; g++) GUI.DrawTexture(new Rect(r.x, r.y + r.height * g / 4f, r.width, 1), grid);
                GUI.Label(new Rect(r.xMax - 60, r.y, 58, 16), sr.ymax.ToString("G3"), tinyR);
                GUI.Label(new Rect(r.xMax - 60, r.yMax - 16, 58, 16), sr.ymin.ToString("G3"), tinyR);
                int n = sr.pts.Count;
                if (n > 1)
                {
                    float ta = sr.pts[0].x, tb = Mathf.Max(sr.pts[n - 1].x, ta + 1e-3f);
                    int cols = (int)r.width;
                    float prevY = -1;
                    int j = 0;
                    for (int c = 0; c < cols; c++)
                    {
                        float tc = ta + (tb - ta) * c / (cols - 1);
                        while (j < n - 1 && sr.pts[j + 1].x <= tc) j++;
                        float py = r.yMax - Map(sr, sr.pts[j].y) * r.height;
                        float lo = prevY < 0 ? py : Mathf.Min(py, prevY), hi = prevY < 0 ? py : Mathf.Max(py, prevY);
                        GUI.DrawTexture(new Rect(r.x + c, lo - 1, 2, hi - lo + 2), dot);
                        prevY = py;
                    }
                    GUI.Label(new Rect(r.x, r.yMax + 1, 160, 16), "last " + (tb - ta).ToString("F0") + " s", tiny);
                }
                y += ch + 30;
            }
            if (plines.Length > 0)
            {
                GUI.Label(new Rect(x0 + 10, y, pw - 20, 18), "CONTROLLER PARAMETERS", label);
                y += 20;
                foreach (string pl in plines) { GUI.Label(new Rect(x0 + 14, y, pw - 24, 18), pl.Trim(), mono); y += 17; }
            }
        }
    }

    // Test-bench commands from kspilot: write a command into GameData/AutoLab/cmd.req
    //   clearpad : destroy launch-clamp debris left standing on the ground (flight scene)
    [KSPAddon(KSPAddon.Startup.EveryScene, false)]
    public class BenchCommands : MonoBehaviour
    {
        float next;

        void Update()
        {
            if (Time.realtimeSinceStartup < next) return;
            next = Time.realtimeSinceStartup + 0.25f;
            string f = Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/cmd.req");
            if (!File.Exists(f)) return;
            string cmd = File.ReadAllText(f).Trim();
            File.Delete(f);
            try
            {
                if (cmd.StartsWith("resize "))       // "resize W H": windowed resolution in pixels, applied live
                {
                    string[] a = cmd.Split(' ');
                    Screen.SetResolution(int.Parse(a[1]), int.Parse(a[2]), false);
                    GameSettings.SCREEN_RESOLUTION_WIDTH = int.Parse(a[1]);
                    GameSettings.SCREEN_RESOLUTION_HEIGHT = int.Parse(a[2]);
                    Debug.Log("[AutoLab] resize " + a[1] + "x" + a[2]);
                    File.WriteAllText(f.Replace("cmd.req", "cmd.done"), cmd);
                }
                else if (cmd == "clearpad")
                {
                    int n = 0;
                    foreach (Vessel v in FlightGlobals.Vessels.ToArray())
                    {
                        if (v == FlightGlobals.ActiveVessel || !v.loaded) continue;
                        bool allClamps = v.parts.Count > 0;
                        foreach (Part p in v.parts)
                            if (!p.Modules.Contains("LaunchClamp")) { allClamps = false; break; }
                        // also old test wrecks lying on the ground near the active stage's landing zone
                        bool wreck = (v.situation == Vessel.Situations.LANDED || v.situation == Vessel.Situations.SPLASHED)
                                     && FlightGlobals.ActiveVessel != null
                                     && Vector3d.Distance(v.GetWorldPos3D(), FlightGlobals.ActiveVessel.GetWorldPos3D()) < 2500;
                        if (!allClamps && !wreck) continue;
                        v.Die();
                        n++;
                    }
                    Debug.Log("[AutoLab] clearpad: destroyed " + n + " clamp vessels");
                    File.WriteAllText(f.Replace("cmd.req", "cmd.done"), "clearpad " + n);
                }
            }
            catch (Exception e) { Debug.LogError("[AutoLab] cmd " + e); }
        }
    }

    // While an autonomous flight runs (the kspilot watchdog keeps GameData/AutoLab/lock.hb fresh),
    // keyboard throttle / pause / quicksave are locked so a stray key (Shift = throttle up,
    // Esc = pause) cannot fly the vehicle. kRPC writes the control state directly and is unaffected.
    // traj.txt ("pad lat lon alt" then "lat lon alt" per point) is drawn as the predicted track.
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class FlightGuard : MonoBehaviour
    {
        const string LOCK = "kspilot_autoflight";
        // staging and time warp stay unlocked (kspilot stages and warps over kRPC; unchanged since MechJeb's removal)
        const ControlTypes LOCKED = ControlTypes.THROTTLE | ControlTypes.PAUSE
                                    | ControlTypes.QUICKSAVE | ControlTypes.QUICKLOAD;
        float next;
        bool locked;
        DateTime trajStamp;
        double[][] track = new double[0][];
        double[] pad;
        LineRenderer line, marker;
        Vector3[] posBuf = new Vector3[0];
        float[] cumBuf = new float[0];

        static string F(string n) { return Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/" + n); }

        LineRenderer MakeLine(Color c)
        {
            var go = new GameObject("kspilot_line");
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = true;
            // the shader kRPC's Drawing service uses (known to render in the flight cameras)
            Shader sh = Shader.Find("Legacy Shaders/Particles/Additive") ?? Shader.Find("Sprites/Default");
            Debug.Log("[AutoLab] line shader: " + (sh != null ? sh.name : "none"));
            lr.material = new Material(sh);
            lr.startColor = lr.endColor = c;
            lr.material.SetColor("_TintColor", c);
            lr.positionCount = 0;
            return lr;
        }

        void Start() { Debug.Log("[AutoLab] FlightGuard start, watching " + F("lock.hb")); }

        void Update()
        {
            try { Tick(); }
            catch (Exception e)
            {
                if (!failed) Debug.LogError("[AutoLab] FlightGuard " + e);
                failed = true;
            }
        }

        bool failed, logged, drawLogged;

        void Tick()
        {
            if (Time.realtimeSinceStartup >= next)
            {
                next = Time.realtimeSinceStartup + 0.5f;
                string hb = F("lock.hb");
                bool want = File.Exists(hb) && Math.Abs((DateTime.UtcNow - File.GetLastWriteTimeUtc(hb)).TotalSeconds) < 30;
                if (want != locked)
                {
                    if (want) InputLockManager.SetControlLock(LOCKED, LOCK);
                    else InputLockManager.RemoveControlLock(LOCK);
                    locked = want;
                    Debug.Log("[AutoLab] keyboard flight controls " + (want ? "locked" : "unlocked"));
                }
                string tf = F("traj.txt");
                try
                {
                    if (!File.Exists(tf)) { track = new double[0][]; pad = null; }
                    else if (File.GetLastWriteTime(tf) != trajStamp)
                    {
                        trajStamp = File.GetLastWriteTime(tf);
                        var pts = new System.Collections.Generic.List<double[]>();
                        pad = null;
                        foreach (string ln in File.ReadAllLines(tf))
                        {
                            string[] s = ln.Trim().Split(' ');
                            if (s.Length == 4 && s[0] == "pad")
                                pad = new[] { double.Parse(s[1]), double.Parse(s[2]), double.Parse(s[3]) };
                            else if (s.Length == 3)
                                pts.Add(new[] { double.Parse(s[0]), double.Parse(s[1]), double.Parse(s[2]) });
                        }
                        track = pts.ToArray();
                        if (!logged) { Debug.Log("[AutoLab] traj: " + track.Length + " points, pad " + (pad != null)); logged = true; }
                    }
                }
                catch (Exception e) { Debug.LogWarning("[AutoLab] traj " + e.Message); }
            }
            Draw();
        }

        // world positions are recomputed every frame (floating origin); the width follows the camera
        // distance so the line stays a few pixels wide however close it passes to the camera
        void Draw()
        {
            CelestialBody body = FlightGlobals.currentMainBody;
            Camera cam = FlightCamera.fetch != null && FlightCamera.fetch.mainCamera != null ? FlightCamera.fetch.mainCamera : Camera.main;
            if (line == null) { line = MakeLine(new Color(1f, 0.75f, 0.1f, 0.9f)); marker = MakeLine(new Color(0.2f, 1f, 0.4f, 0.9f)); }
            if (body == null || cam == null || track.Length < 2) { line.positionCount = 0; marker.positionCount = 0; return; }
            Vector3 cp = cam.transform.position;
            line.positionCount = track.Length;
            // LineRenderer width curves are indexed by the fraction of the line's LENGTH (not vertex index):
            // key every vertex at its cumulative-length fraction so the width really follows camera distance
            if (posBuf.Length != track.Length) { posBuf = new Vector3[track.Length]; cumBuf = new float[track.Length]; }   // reused (perf)
            var pos = posBuf;
            var cum = cumBuf;
            for (int i = 0; i < track.Length; i++)
            {
                pos[i] = body.GetWorldSurfacePosition(track[i][0], track[i][1], track[i][2]);
                cum[i] = i == 0 ? 0f : cum[i - 1] + Vector3.Distance(pos[i - 1], pos[i]);
            }
            line.SetPositions(pos);
            float len = Mathf.Max(cum[track.Length - 1], 1e-3f);
            var curve = new AnimationCurve();
            for (int i = 0; i < track.Length; i++)
                curve.AddKey(new Keyframe(cum[i] / len, Mathf.Clamp(0.0025f * Vector3.Distance(cp, pos[i]), 0.05f, 400f)));
            line.widthCurve = curve;
            line.widthMultiplier = 1f;
            if (!drawLogged)
            {
                drawLogged = true;
                Debug.Log("[AutoLab] traj draw: " + track.Length + " points, length " + len.ToString("F0") + " m");
            }
            if (pad != null)
            {
                Vector3 a = body.GetWorldSurfacePosition(pad[0], pad[1], pad[2]);
                Vector3 b = body.GetWorldSurfacePosition(pad[0], pad[1], pad[2] + 300);
                marker.positionCount = 2;
                marker.SetPosition(0, a);
                marker.SetPosition(1, b);
                marker.widthMultiplier = Mathf.Max(0.3f, 0.005f * Vector3.Distance(cp, a));
            }
            else marker.positionCount = 0;
        }

        void OnDestroy()
        {
            InputLockManager.RemoveControlLock(LOCK);
            if (line != null) Destroy(line.gameObject);
            if (marker != null) Destroy(marker.gameObject);
        }
    }

    // Camera director: GameData/AutoLab/shot.txt selects the shot (see the director shot list)
    //   chase                      : KSP's own flight camera (default)
    //   ground <lat> <lon> <alt> [fov] [frame_m] [min_fov]: a fixed camera on the ground that tracks the stage (long
    //                                lens); with frame_m > 0 the lens zooms so ~frame_m metres around the stage fill
    //                                the picture HEIGHT (FlightCamera.SetFoV sets Unity's vertical fieldOfView, no
    //                                clamp: Assembly-CSharp IL 2026-10-21), fov never wider than [fov], never narrower
    //                                than [min_fov] (default 1.2 deg = every shot string written before fix round 2)
    //   onboard <right> <up> <out> [fov]: a camera mounted on the hull (vessel axes, m), looking down along it
    //   fixed <lat> <lon> <alt> <tlat> <tlon> <talt> [fov]: a camera fixed in the world aimed at a fixed point (S10 drone)
    //   orbit <clat> <clon> <calt> <radius> <up> <az0> <rate> [fov]: V14 S10 drone (recording only): circles the point
    //                                (clat, clon) at `radius` m, `up` m above calt; its azimuth (deg from north,
    //                                clockwise) is az0 + rate * (game s since the cut) (RecMath.OrbitAz, every frame);
    //                                always aimed at the vessel's CoM; fixed fov (no zoom); the cut is hard
    //   rel <dist> <elev> <az> [fov]: a camera riding with the stage at dist m, elev deg above its local horizon,
    //                                az deg round from straight behind the horizontal velocity (90 = right side,
    //                                180 = ahead); parameter changes within one rel shot are eased (slow moves),
    //                                a change of shot type is a hard cut
    //   optional tokens (recording v2 fix round 2; old strings unchanged): "sun" (rel only) = az is counted from the
    //   sun's direction instead of from straight behind (camera on the lit side, az 0 = sun behind the camera);
    //   "cut" = a new shot string carrying it is a hard cut even when the shot type stays the same
    // While a director shot is active the stock camera update is suspended and CameraAim stands aside.
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class CameraDirector : MonoBehaviour
    {
        public static bool Active;
        public static string Current = "chase";     // shot string on air (Recorder v2 index.csv)
        float next;
        string[] shot = new string[] { "chase" };
        bool suspended;
        float baseFov = -1f;
        string lastKind = "";
        string lastKey = "";
        float sDist, sElev, sAz, sFov;
        double orbitUt0;                    // UT of the cut to the current orbit shot (its t = 0)
        Vector3 relFwd = Vector3.zero;      // smoothed travel direction, body-fixed axes

        static string F(string n) { return Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/" + n); }
        static float P(string[] a, int i, float d)
        {
            float v;
            return a.Length > i && float.TryParse(a[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out v) ? v : d;
        }

        static double PD(string[] a, int i, double d)
        {   // double precision for world positions (a float lat/lon is ~1 m off)
            double v;
            return a.Length > i && double.TryParse(a[i], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out v) ? v : d;
        }

        // ease towards the target (time constant tau, s); a new shot type starts at the target (hard cut)
        static float Ease(ref float cur, float target, float tau, bool cut)
        {
            cur = cut ? target : cur + (target - cur) * (1f - Mathf.Exp(-Time.deltaTime / tau));
            return cur;
        }

        void LateUpdate()
        {
            try
            {
                if (Time.realtimeSinceStartup >= next)
                {
                    next = Time.realtimeSinceStartup + 0.2f;
                    string f = F("shot.txt");
                    shot = File.Exists(f) ? File.ReadAllText(f).Trim().Split(' ') : new string[] { "chase" };
                    Current = string.Join(" ", shot).Replace(',', ' ');
                }
                FlightCamera fc = FlightCamera.fetch;
                Vessel v = FlightGlobals.ActiveVessel;
                if (fc == null || v == null) return;
                if (baseFov < 0) baseFov = fc.FieldOfView;
                string kind = shot.Length > 0 ? shot[0] : "chase";
                bool director = kind == "ground" || kind == "onboard" || kind == "rel" || kind == "fixed" || kind == "orbit";
                if (!director)
                {
                    if (suspended) { fc.ActivateUpdate(); fc.SetFoV(baseFov); suspended = false; }
                    Active = false;
                    lastKind = kind;
                    return;
                }
                if (!suspended) { fc.DeactivateUpdate(); suspended = true; }
                Active = true;
                string key = string.Join(" ", shot);
                bool cut = kind != lastKind || (key != lastKey && System.Array.IndexOf(shot, "cut") >= 0);
                lastKind = kind;
                lastKey = key;
                Vector3 up = (v.CoM - v.mainBody.position).normalized;
                Vector3 pos, look, camUp = up;
                float fov;
                if (kind == "ground")
                {
                    pos = v.mainBody.GetWorldSurfacePosition(P(shot, 1, 0), P(shot, 2, 0), P(shot, 3, 0));
                    look = (Vector3)v.CoM - pos;
                    float fmax = P(shot, 4, 20f), frameM = P(shot, 5, 0f);
                    fov = fmax;
                    if (frameM > 0f)
                        fov = Ease(ref sFov, Mathf.Clamp(2f * Mathf.Atan(0.5f * frameM / Mathf.Max(look.magnitude, 1f)) * Mathf.Rad2Deg, Mathf.Min(P(shot, 6, 1.2f), fmax), fmax), 0.6f, cut);
                    else
                        sFov = fmax;     // a later switch to the auto-zoom lens (same camera) eases from this fov
                }
                else if (kind == "fixed")
                {   // a camera fixed in the world looking at a fixed point (V10 drone view S10); hard cut, no easing
                    pos = v.mainBody.GetWorldSurfacePosition(PD(shot, 1, 0), PD(shot, 2, 0), PD(shot, 3, 0));
                    Vector3 tgt = v.mainBody.GetWorldSurfacePosition(PD(shot, 4, 0), PD(shot, 5, 0), PD(shot, 6, 0));
                    look = tgt - pos;
                    camUp = (pos - (Vector3)v.mainBody.position).normalized;
                    fov = P(shot, 7, 50f);
                }
                else if (kind == "orbit")
                {   // V14 S10 drone: circles the landing point, aimed at the vessel every frame, fixed fov (no zoom).
                    // Position from lat / lon / alt (no axis-handedness assumption, as "fixed"); the azimuth advances
                    // with game time from the cut (smooth per frame, frame-rate independent; stops while paused)
                    double now = Planetarium.GetUniversalTime();
                    if (cut) orbitUt0 = now;
                    double clat = PD(shot, 1, 0), clon = PD(shot, 2, 0), lat, lon;
                    double az = RecMath.OrbitAz(PD(shot, 6, 0), PD(shot, 7, 10), now - orbitUt0);
                    RecMath.OrbitLatLon(clat, clon, v.mainBody.Radius, PD(shot, 4, 70), az, out lat, out lon);
                    pos = v.mainBody.GetWorldSurfacePosition(lat, lon, PD(shot, 3, 0) + PD(shot, 5, 30));
                    look = (Vector3)v.CoM - pos;
                    camUp = (pos - (Vector3)v.mainBody.position).normalized;
                    fov = P(shot, 8, 50f);
                }
                else if (kind == "rel")
                {
                    Transform bt = v.mainBody.transform;
                    Vector3 vel = v.altitude < 36000.0 ? (Vector3)v.srf_velocity : (Vector3)v.obt_velocity;
                    Vector3 hv = Vector3.ProjectOnPlane(vel, up);
                    if (hv.magnitude > 3f)
                    {   // body-fixed axes: no jump when KSP switches its world frame (rotating <-> inertial)
                        Vector3 hl = bt.InverseTransformDirection(hv.normalized);
                        relFwd = relFwd == Vector3.zero ? hl : Vector3.Slerp(relFwd, hl, 1f - Mathf.Exp(-Time.deltaTime / 1.5f));
                    }
                    if (relFwd == Vector3.zero)                  // no horizontal motion yet: the travel direction is north
                        relFwd = bt.InverseTransformDirection(Vector3.ProjectOnPlane(bt.up, up).normalized);
                    Vector3 fwd = Vector3.ProjectOnPlane(bt.TransformDirection(relFwd), up).normalized;
                    Vector3 right = Vector3.Cross(up, fwd);
                    float d = Ease(ref sDist, P(shot, 1, 60f), 2.5f, cut);
                    float el = Ease(ref sElev, P(shot, 2, 8f), 2.5f, cut) * Mathf.Deg2Rad;
                    float azT = P(shot, 3, 20f);
                    if (System.Array.IndexOf(shot, "sun") >= 0 && Planetarium.fetch != null && Planetarium.fetch.Sun != null)
                    {   // az from the sun's direction (all vectors in Unity's own frame: no handedness assumption)
                        Vector3 sun = Vector3.ProjectOnPlane((Vector3)(Planetarium.fetch.Sun.position - (Vector3d)v.CoM), up);
                        if (sun.sqrMagnitude > 1e-6f)
                        {
                            sun.Normalize();
                            azT += Mathf.Atan2(Vector3.Dot(sun, right), -Vector3.Dot(sun, fwd)) * Mathf.Rad2Deg;
                        }
                        if (!cut) azT = sAz + Mathf.DeltaAngle(sAz, azT);     // ease the short way round
                    }
                    float az = Ease(ref sAz, azT, 2.5f, cut) * Mathf.Deg2Rad;
                    Vector3 off = Mathf.Cos(el) * (-Mathf.Cos(az) * fwd + Mathf.Sin(az) * right) + Mathf.Sin(el) * up;
                    pos = (Vector3)v.CoM + off * d;
                    look = (Vector3)v.CoM - pos;
                    fov = P(shot, 4, 60f);
                }
                else
                {
                    Transform t = v.ReferenceTransform != null ? v.ReferenceTransform : v.transform;
                    Vector3 ax = t.up, right = t.right, outv = t.forward;   // up = long axis (nose)
                    pos = (Vector3)v.CoM + right * P(shot, 1, 0) + ax * P(shot, 2, 0) + outv * P(shot, 3, 3f);
                    look = (-ax * 0.94f + outv * -0.34f).normalized;           // down the hull, slightly inwards
                    camUp = outv;
                    fov = P(shot, 4, 70f);
                }
                fc.transform.position = pos;
                fc.transform.rotation = Quaternion.LookRotation(look, camUp);
                fc.SetFoV(fov);
            }
            catch (Exception e) { Debug.LogWarning("[AutoLab] director " + e.Message); }
        }

        void OnDestroy()
        {
            if (suspended && FlightCamera.fetch != null) FlightCamera.fetch.ActivateUpdate();
        }
    }

    // Presentation camera aiming. kspilot writes GameData/AutoLab/cam.txt = blend alpha (0..1); with
    // alpha > 0 the free camera is steered (closed loop on camHdg/camPitch, so no assumption about the
    // heading convention) until its line of sight runs from the camera through the stage to the pad
    // given by the "pad" line of traj.txt; alpha blends from the level view (yaw kept, 10 deg down).
    [KSPAddon(KSPAddon.Startup.Flight, false)]
    public class CameraAim : MonoBehaviour
    {
        float next, alpha;
        double[] pad;
        float yaw0 = float.NaN, hdgSign;
        float prevYaw = float.NaN, lastStep;

        static string F(string n) { return Path.Combine(KSPUtil.ApplicationRootPath, "GameData/AutoLab/" + n); }

        void LateUpdate()
        {
            try
            {
                if (Time.realtimeSinceStartup >= next)
                {
                    next = Time.realtimeSinceStartup + 0.2f;
                    alpha = 0f;
                    if (File.Exists(F("cam.txt")))
                        float.TryParse(File.ReadAllText(F("cam.txt")).Trim(), System.Globalization.NumberStyles.Float,
                                       System.Globalization.CultureInfo.InvariantCulture, out alpha);
                    pad = null;
                    if (File.Exists(F("traj.txt")))
                        foreach (string ln in File.ReadLines(F("traj.txt")))   // lazy: stops at the "pad" line
                        {
                            string[] a = ln.Trim().Split(' ');
                            if (a.Length == 4 && a[0] == "pad")
                            {
                                pad = new[] { double.Parse(a[1], System.Globalization.CultureInfo.InvariantCulture),
                                              double.Parse(a[2], System.Globalization.CultureInfo.InvariantCulture),
                                              double.Parse(a[3], System.Globalization.CultureInfo.InvariantCulture) };
                                break;
                            }
                        }
                }
                FlightCamera fc = FlightCamera.fetch;
                Vessel v = FlightGlobals.ActiveVessel;
                if (fc == null || v == null || fc.mainCamera == null || alpha <= 0f || pad == null || CameraDirector.Active)
                {
                    yaw0 = float.NaN; prevYaw = float.NaN;
                    return;
                }
                Vector3 up = (v.CoM - v.mainBody.position).normalized;
                Vector3 north = Vector3.ProjectOnPlane(v.mainBody.transform.up, up).normalized;
                Vector3 fwd = fc.mainCamera.transform.forward;
                Vector3 toPad = ((Vector3)v.mainBody.GetWorldSurfacePosition(pad[0], pad[1], pad[2]) - (Vector3)v.CoM).normalized;
                float yawNow = Vector3.SignedAngle(north, Vector3.ProjectOnPlane(fwd, up), up);
                float yawPad = Vector3.SignedAngle(north, Vector3.ProjectOnPlane(toPad, up), up);
                float elevPad = 90f - Vector3.Angle(up, toPad);
                if (float.IsNaN(yaw0)) yaw0 = yawNow;
                // side-on view: look 40 deg off the stage->pad line (the stage must not hide the pad) and
                // never steeper than 45 deg down, so stage and pad are both in frame
                float side = Mathf.Abs(Mathf.DeltaAngle(yaw0, yawPad + 40f)) <= Mathf.Abs(Mathf.DeltaAngle(yaw0, yawPad - 40f)) ? 40f : -40f;
                float yawT = yaw0 + alpha * Mathf.DeltaAngle(yaw0, yawPad + side);
                float elevT = Mathf.Lerp(-10f, Mathf.Clamp(elevPad, -45f, -10f), alpha);
                // heading: learn the sign of d(yaw)/d(camHdg) from the previous step, then step towards the target
                if (!float.IsNaN(prevYaw) && Mathf.Abs(lastStep) > 1e-4f)
                {
                    float dy = Mathf.DeltaAngle(prevYaw, yawNow);
                    if (Mathf.Abs(dy) > 0.01f) hdgSign = Mathf.Sign(dy) * Mathf.Sign(lastStep);
                }
                float err = Mathf.DeltaAngle(yawNow, yawT);
                lastStep = (hdgSign == 0f ? 1f : hdgSign) * Mathf.Clamp(err * 0.15f, -3f, 3f) * Mathf.Deg2Rad;
                fc.camHdg += lastStep;
                prevYaw = yawNow;
                // pitch: camPitch > 0 looks down (elevation of the line of sight ~ -camPitch)
                float target = Mathf.Clamp(-elevT * Mathf.Deg2Rad, fc.minPitch, fc.maxPitch);
                fc.camPitch += (target - fc.camPitch) * 0.15f;
            }
            catch (Exception e) { Debug.LogWarning("[AutoLab] camaim " + e.Message); alpha = 0f; }
        }
    }

    [KSPAddon(KSPAddon.Startup.MainMenu, false)]
    public class AutoLoader : MonoBehaviour
    {
        const string SaveName = "AutoPilotLab";
        static bool done;
        int frames;

        void Update()
        {
            if (done) return;
            // let the main menu finish initialising
            if (++frames < 60) return;
            done = true;
            try
            {
                string dir = Path.Combine(KSPUtil.ApplicationRootPath, "saves/" + SaveName);
                if (!File.Exists(Path.Combine(dir, "persistent.sfs")))
                {
                    Debug.Log("[AutoLab] creating sandbox save " + SaveName);
                    Directory.CreateDirectory(dir);
                    Game g = new Game();
                    g.Mode = Game.Modes.SANDBOX;
                    g.Title = SaveName;
                    g.Description = "kspilot automated test bench";
                    g.flagURL = "Squad/Flags/default";
                    g.Parameters = new GameParameters();
                    g.startScene = GameScenes.SPACECENTER;
                    g.CrewRoster = KerbalRoster.GenerateInitialCrewRoster(Game.Modes.SANDBOX);
                    g.flightState = new FlightState();
                    g.flightState.universalTime = 0;
                    HighLogic.CurrentGame = g;
                    HighLogic.SaveFolder = SaveName;
                    GamePersistence.SaveGame(g, "persistent", SaveName, SaveMode.OVERWRITE);
                }
                Game game = GamePersistence.LoadGame("persistent", SaveName, true, false);
                if (game == null) { Debug.LogError("[AutoLab] failed to load save"); return; }
                HighLogic.CurrentGame = game;
                HighLogic.SaveFolder = SaveName;
                game.startScene = GameScenes.SPACECENTER;
                game.Start();
                Debug.Log("[AutoLab] save loaded, entering space center");
            }
            catch (Exception e)
            {
                Debug.LogError("[AutoLab] " + e);
            }
        }
    }
}
