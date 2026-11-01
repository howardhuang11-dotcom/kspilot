// Prints RecMath values for a fixed case list (tests/test_video_r6.py computes the same cases with kspilot/showrun.py).
using System;
using System.Globalization;
using AutoLab;

static class Program
{
    static string F(double x) { return x.ToString("R", CultureInfo.InvariantCulture); }

    static void Main()
    {
        double[][] az = { new[] { 330.0, 10.0, 0.0 }, new[] { 330.0, 10.0, 3.5 }, new[] { 350.0, 10.0, 24.0 },
                          new[] { 10.0, -4.0, 5.0 }, new[] { 0.0, 10.0, -1.0 } };
        foreach (var c in az) Console.WriteLine("az " + F(RecMath.OrbitAz(c[0], c[1], c[2])));
        foreach (double a in new[] { 0.0, 90.0, 247.0, 330.0 })
        {
            double lat, lon;
            RecMath.OrbitLatLon(-0.0972079, -74.557673, 600000.0, 70.0, a, out lat, out lon);
            Console.WriteLine("ll " + F(lat) + " " + F(lon));
        }
        // (recFull, haveCd, haveT0, ut, t0)
        object[][] pr = { new object[] { false, true, false, 100.0, 0.0 }, new object[] { true, false, false, 100.0, 0.0 },
                          new object[] { true, true, false, 100.0, 0.0 }, new object[] { true, true, true, 100.0, 100.0 },
                          new object[] { true, true, true, 100.4, 100.0 }, new object[] { true, true, true, 100.8, 100.0 },
                          new object[] { true, true, true, 5000.0, 100.0 } };
        foreach (var c in pr)
            Console.WriteLine("pr " + F(RecMath.PanelReveal((bool)c[0], (bool)c[1], (bool)c[2], (double)c[3], (double)c[4], 0.8)));
    }
}
