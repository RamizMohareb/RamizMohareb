using Autodesk.DesignScript.Runtime;
using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;

namespace Sofistik
{
    /// <summary>
    /// Simple beam on two supports: hinge at the start, roller at the end,
    /// rectangular cross section. Closed-form reactions, shear, moment,
    /// stresses and deflection, plus a SOFiLOAD writer for the same beam.
    ///
    /// Units: L, b, h, a in [mm]; w in [kN/m]; P in [kN]; E in [N/mm2].
    /// 1 kN/m is exactly 1 N/mm, so w is used as N/mm without conversion.
    /// Results: forces in [kN], moments in [kNm], stresses in [N/mm2],
    /// deflection in [mm].
    /// </summary>
    public class SimpleBeam
    {
        /// <summary>
        /// Rectangular section properties.
        /// </summary>
        [MultiReturn(new[] { "Area", "Iy", "Wy", "Ay" })] // DesignScript.Runtime
        public static Dictionary<string, object> Rectangular_Section(double b, double h)
        {
            if (About.About.WebUtilities.GetFromUrl("Rectangular_Section") == "OK")
            {
                double area = b * h;

                return new Dictionary<string, object>
                {
                    { "Area", area },                   // [mm2]
                    { "Iy", b * h * h * h / 12.0 },     // [mm4] second moment of area
                    { "Wy", b * h * h / 6.0 },          // [mm3] elastic section modulus
                    { "Ay", area * 5.0 / 6.0 },         // [mm2] shear area of a rectangle
                };
            }

            else
            {
                return null;
            }
        }

        /// <summary>
        /// Hinge at x = 0, roller at x = L. Uniform load w over the full span
        /// plus an optional point load P at distance a from the hinge.
        /// </summary>
        [MultiReturn(new[] { "Ra", "Rb", "Vmax", "Mmax", "Mmax_x", "Sigma", "Tau", "Deflection", "Wy", "Iy" })] // DesignScript.Runtime
        public static Dictionary<string, object> Analyse_Hinge_Roller(
            double L,
            double b,
            double h,
            double w,
            double P = 0.0,
            double a = 0.0,
            double E = 33000.0,
            int Steps = 200)
        {
            if (About.About.WebUtilities.GetFromUrl("Analyse_Hinge_Roller") == "OK")
            {
                if (L <= 0.0 || b <= 0.0 || h <= 0.0)
                {
                    throw new ArgumentException("L, b and h must be greater than zero.");
                }

                if (a < 0.0 || a > L)
                {
                    throw new ArgumentException("The point load must sit on the span: 0 <= a <= L.");
                }

                if (Steps < 2)
                {
                    Steps = 2;
                }

                double pointLoad = P * 1000.0;              // [kN] -> [N]
                double area = b * h;
                double iy = b * h * h * h / 12.0;           // [mm4]
                double wy = b * h * h / 6.0;                // [mm3]

                // Vertical equilibrium and moment about the roller.
                double ra = w * L / 2.0 + pointLoad * (L - a) / L;   // [N]
                double rb = w * L / 2.0 + pointLoad * a / L;         // [N]

                // Walk the span for the envelope. The stations include both
                // sides of the point load, where the shear jumps.
                double vmax = 0.0;
                double mmax = 0.0;
                double mmaxX = 0.0;

                List<double> stations = new List<double>();
                for (int i = 0; i <= Steps; i++)
                {
                    stations.Add(L * i / Steps);
                }

                // Station where the shear of the uniform load alone changes sign.
                if (w > 0.0)
                {
                    double xv = ra / w;
                    if (xv > 0.0 && xv < L)
                    {
                        stations.Add(xv);
                    }
                }

                if (pointLoad != 0.0)
                {
                    stations.Add(a);
                }

                foreach (double x in stations)
                {
                    double v = ra - w * x - (x > a ? pointLoad : 0.0);
                    double m = ra * x - w * x * x / 2.0 - (x > a ? pointLoad * (x - a) : 0.0);

                    if (Math.Abs(v) > Math.Abs(vmax))
                    {
                        vmax = v;
                    }

                    if (Math.Abs(m) > Math.Abs(mmax))
                    {
                        mmax = m;
                        mmaxX = x;
                    }
                }

                // The shear just left of the point load is not sampled above
                // when a coincides with a station, so check both ends as well.
                if (Math.Abs(ra) > Math.Abs(vmax))
                {
                    vmax = ra;
                }

                if (Math.Abs(rb) > Math.Abs(vmax))
                {
                    vmax = -rb;
                }

                // Midspan deflection, superposed. For the point load the
                // shorter of the two lever arms gives the symmetric form.
                double deflection = 5.0 * w * Math.Pow(L, 4) / (384.0 * E * iy);

                if (pointLoad != 0.0)
                {
                    double bMin = Math.Min(a, L - a);
                    deflection += pointLoad * bMin * (3.0 * L * L - 4.0 * bMin * bMin) / (48.0 * E * iy);
                }

                return new Dictionary<string, object>
                {
                    { "Ra", ra / 1000.0 },                          // [kN]
                    { "Rb", rb / 1000.0 },                          // [kN]
                    { "Vmax", vmax / 1000.0 },                      // [kN]
                    { "Mmax", mmax / 1.0e6 },                       // [kNm]
                    { "Mmax_x", mmaxX },                            // [mm]
                    { "Sigma", mmax / wy },                         // [N/mm2] extreme fibre
                    { "Tau", 1.5 * Math.Abs(vmax) / area },         // [N/mm2] rectangle, 1.5*V/A
                    { "Deflection", deflection },                   // [mm] at midspan
                    { "Wy", wy },                                   // [mm3]
                    { "Iy", iy },                                   // [mm4]
                };
            }

            else
            {
                return null;
            }
        }

        /// <summary>
        /// Shear and moment at a single station x, measured from the hinge.
        /// </summary>
        [MultiReturn(new[] { "x", "V", "M", "Sigma" })] // DesignScript.Runtime
        public static Dictionary<string, object> Internal_Forces_At(
            double L,
            double b,
            double h,
            double w,
            double x,
            double P = 0.0,
            double a = 0.0)
        {
            if (About.About.WebUtilities.GetFromUrl("Internal_Forces_At") == "OK")
            {
                double pointLoad = P * 1000.0;
                double wy = b * h * h / 6.0;
                double ra = w * L / 2.0 + pointLoad * (L - a) / L;

                double v = ra - w * x - (x > a ? pointLoad : 0.0);
                double m = ra * x - w * x * x / 2.0 - (x > a ? pointLoad * (x - a) : 0.0);

                return new Dictionary<string, object>
                {
                    { "x", x },                 // [mm]
                    { "V", v / 1000.0 },        // [kN]
                    { "M", m / 1.0e6 },         // [kNm]
                    { "Sigma", m / wy },        // [N/mm2]
                };
            }

            else
            {
                return null;
            }
        }

        /// <summary>
        /// Writes a SOFiLOAD block for the same beam, appended to an existing
        /// Teddy file in the style of Sofistik_Teddy. Lengths are written in
        /// metres and loads in kN, which is what the other writers in this
        /// project use.
        ///
        /// The keywords below follow the usual SOFiLOAD records. Check them
        /// against the SOFiSTiK release in use before relying on the output:
        /// nothing else in this project writes loads, so there was no existing
        /// block to copy.
        /// </summary>
        [MultiReturn(new[] { "HEAD", "LoadCase", "FilePath" })] // DesignScript.Runtime
        public static Dictionary<string, object> Set_Load_SOFILOAD(
            string filePath,
            string HEAD,
            int LoadCase,
            string LoadCase_Title,
            int Structural_Line_ID,
            double w,
            double Self_Weight_Factor = 0.0,
            int Structural_Point_ID = 0,
            double P = 0.0)
        {
            if (About.About.WebUtilities.GetFromUrl("Set_Load_SOFILOAD") == "OK")
            {
                List<string> usedLine = new List<string>();

                usedLine.Add("+PROG SOFILOAD");
                usedLine.Add("HEAD " + HEAD);
                usedLine.Add("LC " + LoadCase.ToString()
                    + " TITL '" + LoadCase_Title + "'"
                    + " FACD " + Self_Weight_Factor.ToString("0.000", CultureInfo.InvariantCulture));

                if (w != 0.0)
                {
                    usedLine.Add("LLN " + Structural_Line_ID.ToString()
                        + " TYPE PZZ"
                        + " PA " + w.ToString("0.000", CultureInfo.InvariantCulture)
                        + " PE " + w.ToString("0.000", CultureInfo.InvariantCulture));
                }

                if (P != 0.0 && Structural_Point_ID != 0)
                {
                    usedLine.Add("NODE " + Structural_Point_ID.ToString()
                        + " TYPE PZZ"
                        + " P1 " + P.ToString("0.000", CultureInfo.InvariantCulture));
                }

                usedLine.Add("END");

                StreamWriter sww = new StreamWriter(filePath, append: true);
                foreach (var line in usedLine)
                {
                    sww.WriteLine(line);
                }
                sww.Close();

                return new Dictionary<string, object>
                {
                    { "HEAD", HEAD },
                    { "LoadCase", LoadCase },
                    { "FilePath", filePath },
                };
            }

            else
            {
                return null;
            }
        }
    }
}
