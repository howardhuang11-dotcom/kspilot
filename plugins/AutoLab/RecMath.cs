// Pure math of the recording-only presentation features (no Unity / KSP types, so tests/cs_recmath can compile and
// check it outside the game; mirrored by kspilot/showrun.py drone_orbit_* / panel_reveal, tests/test_video_r6.py).
//   V14 (requirement 2026-10-21): the S10 drone ORBITS the landing point (AutoLab.cs CameraDirector "orbit" shot)
//   V15 (requirement 2026-10-21): the HUD data panels are hidden before lift-off in a recording (HudV2.cs)
using System;

namespace AutoLab
{
    public static class RecMath
    {
        /// <summary>Camera azimuth (deg from north, clockwise, seen from the orbit centre) t game s after the cut.</summary>
        public static double OrbitAz(double az0, double rateDegS, double t)
        {
            double a = (az0 + rateDegS * t) % 360.0;
            return a < 0.0 ? a + 360.0 : a;
        }

        /// <summary>Latitude / longitude (deg) of the point `radius` m from (clat, clon) at azimuth azDeg on a sphere of
        /// radius R (small-offset formula, as kspilot/showrun.drone_pose: exact to mm at 70 m).</summary>
        public static void OrbitLatLon(double clat, double clon, double R, double radius, double azDeg, out double lat, out double lon)
        {
            double a = azDeg * Math.PI / 180.0;
            double n = radius * Math.Cos(a), e = radius * Math.Sin(a);
            lat = clat + n / R * 180.0 / Math.PI;
            lon = clon + e / (R * Math.Cos(clat * Math.PI / 180.0)) * 180.0 / Math.PI;
        }

        /// <summary>Smoothstep 0..1 (HudV2.Ease).</summary>
        public static float Ease(float x)
        {
            x = x < 0f ? 0f : x > 1f ? 1f : x;
            return x * x * (3f - 2f * x);
        }

        /// <summary>V15: 0 (hidden) .. 1 (shown) for the HUD data panels. Only a recording HUD (rec=full) with a planned
        /// lift-off (cd=) hides them, until the lift-off UT t0 is known; they then slide / fade in over dur s. Every other
        /// HUD (no recording, a checkpoint recording without a countdown): 1, unchanged.</summary>
        public static float PanelReveal(bool recFull, bool haveCd, bool haveT0, double ut, double t0, double dur)
        {
            if (!recFull || !haveCd) return 1f;
            if (!haveT0) return 0f;
            return Ease((float)((ut - t0) / Math.Max(dur, 1e-6)));
        }
    }
}
