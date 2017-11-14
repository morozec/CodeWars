using System;
using System.Collections.Generic;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;
using IPA.AStar;

namespace Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.AStar
{
    public class ASquare : APoint
    {

        public Dictionary<ASquare, double> AdditionalAngleCoeffs { get; set; }

        public Dictionary<ASquare, double> Angles { get; set; }

        private const double Eps = 1E-6;

        /// <summary>
        /// Длина стороны квадрата
        /// </summary>
        public double Side { get; set; }
        /// <summary>
        /// Координата x левого верхнего угла квадрата
        /// </summary>
        public double X { get; set; }
        /// <summary>
        /// Координата y левого верхнего улга квадрата
        /// </summary>
        public double Y { get; set; }
        /// <summary>
        /// "Вес" квадрата
        /// </summary>
        public double Weight { get; set; }
        /// <summary>
        /// Имя квадрата (для удобства идентификации)
        /// </summary>
        public string Name { get; set; }

        public IEnumerable<ASquare> Neighbors { get; set; } 

        public double CenterX { get { return X + Side/2; } }
        public double CenterY { get { return Y + Side/2; } }
       

        public ASquare(double side, double x, double y, double weight, string name)
        {
            Side = side;
            X = x;
            Y = y;
            Weight = weight;
            Name = name;
            AdditionalAngleCoeffs = new Dictionary<ASquare, double>();
            Angles = new Dictionary<ASquare, double>();
        }

        public override IEnumerable<APoint> GetNeighbors(IEnumerable<APoint> points)
        {
            return Neighbors;
        }

        public override double GetHeuristicCost(APoint goal)
        {
            //return GetManhattanDistance(this, (ASquare)goal);
            return GetEuclidDistance(this, (ASquare)goal);
            //if (AdditionalAngleCoeffs.ContainsKey(goal as ASquare))
            //{
            //    res += AdditionalAngleCoeffs[goal as ASquare];
            //}
            //return res;


        }

        public override double GetCost(APoint goal)
        {
            var dist = GetEuclidDistance(this, goal as ASquare);
            return (Weight + (goal as ASquare).Weight) * dist;
        }

        private static double GetEuclidDistance(ASquare a, ASquare b)
        {
            return Math.Sqrt((a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y));
        }

        private static double GetManhattanDistance(ASquare a, ASquare b)
        {
            var dx = Math.Abs(a.X - b.X);
            var dy = Math.Abs(a.Y - b.Y);
            return dx + dy;
        }

        public override string ToString()
        {
            return "X: " + X + "; Y: " + Y;
        } 
    }
}
