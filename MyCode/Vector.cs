using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class Vector
    {
        public Point P1 { get; set; }
        public Point P2 { get; set; }

        public Vector(Point p1, Point p2)
        {
            P1 = p1;
            P2 = p2;
        }

        public Point V
        {
            get { return new Point(P2.X - P1.X, P2.Y - P1.Y); }
        }

        public double Length
        {
            get { return Math.Sqrt(V.X * V.X + V.Y * V.Y); }
        }

    }
}
