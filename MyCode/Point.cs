using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class Point
    {
        public double X { get; set; }
        public double Y { get; set; }

        public Point(double x, double y)
        {
            X = x;
            Y = y;
        }

        public Point(Point p)
        {
            X = p.X;
            Y = p.Y;
        }

        public double GetSquareDistance(Point p)
        {
            return GetSquareDistance(p.X, p.Y);
        }

        public double GetSquareDistance(double x, double y)
        {
            return (X - x) * (X - x) + (Y - y) * (Y - y);
        }

        public double GetDistance(Point p)
        {
            return GetDistance(p.X, p.Y);
        }

        public double GetDistance(double x, double y)
        {
            return Math.Sqrt((X - x) * (X - x) + (Y - y) * (Y - y));
        }
    }
}
