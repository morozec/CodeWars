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
            var cos = scalarMult / v1.Length / v2.Length;
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

        private static bool Intersect(Point a, Point b, Point c, Point d)
        {
            return Rotate(a, b, c) * Rotate(a, b, d) <= 0 && Rotate(c, d, a) * Rotate(c, d, b) < 0;
        }

        private static bool PointLoc(Rectangle rect, Point a)
        {
            var n = rect.Points.Count;
            if (Rotate(rect.Points[0], rect.Points[1], a) < 0 || Rotate(rect.Points[0], rect.Points[n - 1], a) > 0)
                return false;
            var p = 1;
            var r = n - 1;
            var q = 0;
            while (r - p > 1)
            {
                q = (p + r) / 2;
                if (Rotate(rect.Points[0], rect.Points[q], a) < 0)
                {
                    r = q;
                }
                else
                {
                    p = q;
                }
            }

            return !Intersect(rect.Points[0], a, rect.Points[p], rect.Points[r]);
        }


        public static bool IsIntersects(Rectangle movingRectangle, Rectangle stayingRectangle, Point destPoint)
        {
            var points = stayingRectangle.Points.ToList();
            points.Add(new Point(points.Average(p => p.X), points.Average(p => p.Y)));

            var pathRectangle = new Rectangle { Points = new List<Point>() };
            if (destPoint.X > 0 && destPoint.Y > 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
            }
            else if (destPoint.X < 0 && destPoint.Y > 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X + destPoint.X,
                    movingRectangle.Points[0].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
            }
            else if (destPoint.X > 0 && destPoint.Y < 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
            }
            else if (destPoint.X < 0 && destPoint.Y < 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
            }

            foreach (var point in points)
            {
                if (PointLoc(pathRectangle, point)) return true;
            }
            return false;
        }

        /// <summary>
        ///     Находит дальнюю точку пересчечения окружности и прямой, проходящей через центр окружности
        /// </summary>
        /// <param name="a">центр окружности</param>
        /// <param name="b">вторая точка прямой</param>
        /// <param name="c">точка на краю окружности</param>
        /// <returns></returns>
        public static Point GetLineCircleBehindCrossPoint(Point a, Point b, Point c)
        {
            var radius = a.GetDistance(c);
            var x1 = a.X + radius / Math.Sqrt(1 + Math.Pow((b.Y - a.Y) / (b.X - a.X), 2));
            var x2 = a.X - radius / Math.Sqrt(1 + Math.Pow((b.Y - a.Y) / (b.X - a.X), 2));

            var y1 = (x1 - a.X) * (b.Y - a.Y) / (b.X - a.X) + a.Y;
            var y2 = (x2 - a.X) * (b.Y - a.Y) / (b.X - a.X) + a.Y;

            var p1 = new Point(x1, y1);
            var p2 = new Point(x2, y2);

            return b.GetDistance(p1) > b.GetDistance(p2) ? p1 : p2;
        }

        public static Point GetNearestRectangleCrossPoint(Point p, Rectangle rectangle, Point rectangeCenter)
        {
            Point minCp = null;
            var minDist = double.MaxValue;
            for (var i = 0; i < rectangle.Points.Count; ++i)
            {
                var a0 = rectangle.Points[i];
                var a1 = rectangle.Points[i < rectangle.Points.Count - 1 ? i + 1 : 0];
                var cp = GetCrossPoint(p, rectangeCenter, a0, a1);

                bool isOutsideAnchor;
                if (Math.Abs(a0.X - a1.X) < Tolerance)
                {
                    isOutsideAnchor = a0.Y - cp.Y > Tolerance && a1.Y - cp.Y > Tolerance ||
                                      a0.Y - cp.Y < Tolerance && a1.Y - cp.Y < Tolerance;
                }
                else if (Math.Abs(a0.Y - a1.Y) < Tolerance)
                {
                    isOutsideAnchor = a0.X - cp.X > Tolerance && a1.X - cp.X > Tolerance ||
                                      a0.X - cp.X < Tolerance && a1.X - cp.X < Tolerance;
                }
                else
                {
                    var isOutsideX = a0.X - cp.X > Tolerance && a1.X - cp.X > Tolerance ||
                                     a0.X - cp.X < Tolerance && a1.X - cp.X < Tolerance;
                    var isOutsideY = a0.Y - cp.Y > Tolerance && a1.Y - cp.Y > Tolerance ||
                                     a0.Y - cp.Y < Tolerance && a1.Y - cp.Y < Tolerance;

                    isOutsideAnchor = isOutsideX && isOutsideY;
                }

                if (isOutsideAnchor) continue; // точка снаружи отрезка границы прямоугольника

                var dist = p.GetDistance(cp);
                if (dist < minDist)
                {
                    minDist = dist;
                    minCp = cp;
                }
            }

            if (minCp == null) throw new Exception("No cross point");

            return minCp;
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

    }
}
