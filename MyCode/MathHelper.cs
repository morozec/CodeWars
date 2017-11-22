using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public static class MathHelper
    {
        private const double Tolerance = 1E-3;
        

        public static double GetAnlge(Vector v1, Vector v2)
        {
            var scalarMult = v1.V.X * v2.V.X + v1.V.Y * v2.V.Y;
            var cos = Math.Round(scalarMult / v1.Length / v2.Length, 5);
            var angle = Math.Acos(cos);
            var rotate = Rotate(v1.P1, v1.P2, v2.P2);
            return rotate < 0 ? angle : -angle;
        }

        /// <summary>
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns>+, если точка справа. -, если точка слева</returns>
        private static double Rotate(Point a, Point b, Point c)
        {
            //-, ибо другое расположение осей x-y
            return -((b.X - a.X) * (c.Y - b.Y) - (b.Y - a.Y) * (c.X - b.X));
        }
        

        public static Point GetNearestRectangleCrossPoint(Point sourcePoint, Rectangle rectangle, Point destPoint)
        {
            Point minCp = null;
            var minDist = double.MaxValue;
            for (var i = 0; i < rectangle.Points.Count; ++i)
            {
                var a0 = rectangle.Points[i];
                var a1 = rectangle.Points[i < rectangle.Points.Count - 1 ? i + 1 : 0];
                var cp = GetCrossPoint(sourcePoint, destPoint, a0, a1);

                bool isOutsideAnchor = IsOutsideAnchor(a0, a1, cp);

                if (isOutsideAnchor) continue; // точка снаружи отрезка границы прямоугольника

                isOutsideAnchor = IsOutsideAnchor(sourcePoint, destPoint, cp);
                if (isOutsideAnchor) continue; //находится за моим центром или центром врага

                var dist = sourcePoint.GetDistance(cp);
                if (dist < minDist)
                {
                    minDist = dist;
                    minCp = cp;
                }
            }

            if (minCp == null) throw new Exception("No cross point");

            return minCp;
        }

        private static bool IsOutsideAnchor(Point a0, Point a1, Point p)
        {
            if (p.GetDistance(a0) < Tolerance || p.GetDistance(a1) < Tolerance) return false;

            bool isOutsideAnchor;
            if (Math.Abs(a0.X - a1.X) < Tolerance)
            {
                isOutsideAnchor = a0.Y - p.Y > Tolerance && a1.Y - p.Y > Tolerance ||
                                  a0.Y - p.Y < Tolerance && a1.Y - p.Y < Tolerance;
            }
            else if (Math.Abs(a0.Y - a1.Y) < Tolerance)
            {
                isOutsideAnchor = a0.X - p.X > Tolerance && a1.X - p.X > Tolerance ||
                                  a0.X - p.X < Tolerance && a1.X - p.X < Tolerance;
            }
            else
            {
                var isOutsideX = a0.X - p.X > Tolerance && a1.X - p.X > Tolerance ||
                                 a0.X - p.X < Tolerance && a1.X - p.X < Tolerance;
                var isOutsideY = a0.Y - p.Y > Tolerance && a1.Y - p.Y > Tolerance ||
                                 a0.Y - p.Y < Tolerance && a1.Y - p.Y < Tolerance;

                isOutsideAnchor = isOutsideX && isOutsideY;
            }
            return isOutsideAnchor;
        }

        //public static double GetPointAnchorDistance(Point p, Point a0, Point a1)
        //{
        //    return 
        //        Math.Abs(((a0.Y - a1.Y)*p.X + (a1.X - a0.X)*p.Y + (a0.X*a1.Y - a1.X*a0.Y))/
        //                 Math.Sqrt((a1.X - a0.X)*(a1.X - a0.X) + (a1.Y - a0.Y)*(a1.Y - a0.Y)));


        //}

        public static Point GetCrossPoint(Point p1, Point p2, Point p3, Point p4)
        {
            return new Point(
                ((p1.X * p2.Y - p1.Y*p2.X)*(p3.X - p4.X) - (p1.X - p2.X)*(p3.X*p4.Y-p3.Y*p4.X))/((p1.X-p2.X)*(p3.Y - p4.Y)-(p1.Y-p2.Y)*(p3.X-p4.X)),
                ((p1.X * p2.Y - p1.Y*p2.X)*(p3.Y - p4.Y) - (p1.Y - p2.Y)*(p3.X*p4.Y-p3.Y*p4.X))/((p1.X-p2.X)*(p3.Y - p4.Y)-(p1.Y-p2.Y)*(p3.X-p4.X))
                );
        }
        

        public static Rectangle GetJarvisRectangle(IList<Point> a)
        {
            var n = a.Count; //число точек

            if (n == 1)
                return new Rectangle()
                {
                    Points = new List<Point>()
                    {
                        new Point(a.Single().X - Tolerance, a.Single().Y - Tolerance),
                        new Point(a.Single().X - Tolerance, a.Single().Y + Tolerance),
                        new Point(a.Single().X + Tolerance, a.Single().Y + Tolerance),
                        new Point(a.Single().X + Tolerance, a.Single().Y - Tolerance),
                    }
                };

            if (n == 2)
                return new Rectangle() {Points = new List<Point>()
                {
                    new Point(a.Min(po => po.X), a.Min(po => po.Y)),
                    new Point(a.Min(po => po.X), a.Max(po => po.Y)),
                    new Point(a.Max(po => po.X), a.Max(po => po.Y)),
                    new Point(a.Max(po => po.X), a.Min(po => po.Y)),
                }};

            var p = new List<int>(); //список номеров точек

            for (var i = 0; i < n; ++i)
            {
                p.Add(i);
            }
            for (var i = 1; i < n; ++i)
            {
                if (a[p[i]].X < a[p[0]].X)
                {
                    var c = p[i];
                    p[i] = p[0];
                    p[0] = c;
                }
            }

            //Джарвис
            var h = new List<int> { p[0] };
            p.RemoveAt(0);
            p.Add(h[0]);

            while (true)
            {
                var right = 0;
                for (var i = 1; i < p.Count; ++i)
                {

                    if (Rotate(a[h.Last()], a[p[right]], a[p[i]]) < 0)
                    {
                        right = i;
                    }
                }
                if (p[right] == h[0]) break;
                else
                {
                    h.Add(p[right]);
                    p.RemoveAt(right);
                }
            }



            //Грэхем
            //for (var i = 2; i < n; ++i)
            //{
            //    var j = i;
            //    while (j > 1 && Rotate(a[p[0]], a[p[j - 1]], a[p[j]]) < 0)
            //    {
            //        var c = p[j];
            //        p[j] = p[j-1];
            //        p[j-1] = c;
            //        j -= 1;
            //    }
            //}

            //var h = new List<int>() {p[0], p[1]};
            //for (var i = 2; i < n; ++i)
            //{
            //    while (Rotate(a[h[h.Count - 2]], a[h[h.Count - 1]], a[p[i]]) < 0)
            //    {
            //        h.RemoveAt(h.Count - 1);
            //    }
            //    h.Add(p[i]);
            //}


            var rect = new Rectangle() { Points = new List<Point>() };
            for (var i = 0; i < h.Count; ++i)
            {
                rect.Points.Add(a[h[i]]);
            }


            SmoothRectangle(rect);
            return rect;

        }

        private static void SmoothRectangle(Rectangle rectangle)
        {
            var hasChanges = true;
            while (hasChanges)
            {
                hasChanges = false;
                for (var i = rectangle.Points.Count - 1; i >= 0; --i)
                {

                    var p = rectangle.Points[i];
                    var p0 = rectangle.Points[i > 0 ? i - 1 : rectangle.Points.Count - 1];
                    var p1 = rectangle.Points[i < rectangle.Points.Count - 1 ? i + 1 : 0];

                    var angle = GetAnlge(new Vector(p0, p), new Vector(p0, p1));
                    if (Math.Abs(angle) < Tolerance)
                    {
                        rectangle.Points.RemoveAt(i);
                        hasChanges = true;
                    }
                }
            }
        }


    }
}
