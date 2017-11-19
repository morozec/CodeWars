using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class Rectangle
    {
        private const double Tolerance = 1E-3;
        public IList<Point> Points { get; set; }


        public double TurnAngle
        {
            get
            {
                Vector maxSide = null;
                var maxLength = 0d;
                for (var i = 0; i < Points.Count; ++i)
                {
                    var p1 = Points[i];
                    var p2 = i < Points.Count - 1 ? Points[i + 1] : Points[0];
                    var v = new Vector(p1, p2);
                    if (v.Length > maxLength)
                    {
                        maxLength = v.Length;
                        maxSide = v;
                    }
                }

                var midMaxSidePoint = new Point((maxSide.P1.X + maxSide.P2.X)/2, (maxSide.P1.Y + maxSide.P2.Y)/2);
                var centerPoint = new Point(Points.Average(p => p.X), Points.Average(p => p.Y));
                var resVector = new Vector(centerPoint, midMaxSidePoint);

                return Math.Abs(resVector.V.X) < Tolerance ? Math.PI/2 : Math.Atan(resVector.V.Y/resVector.V.X);
            }
        }
    }
}
