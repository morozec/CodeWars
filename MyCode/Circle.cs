using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class Circle
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double R { get; set; }

        public Circle(double x, double y, double r)
        {
            X = x;
            Y = y;
            R = r;
        }

        public bool IsInside(double x, double y)
        {
            return (x - X) * (x - X) + (y - Y) * (y - Y) <= R * R;
        }
    }
}
