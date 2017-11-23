using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class RotationContainer
    {
        public Point AfterRotationPoint { get; set; }
        public Point PrepareRotationPoint { get; set; }
        public Point RotationCenterPoint { get; set; }
        public double RotationAngle { get; set; }

        public bool IsFarFromBroder(double width, double height, double farBorderDistance)
        {
            var isFarFromBorder = AfterRotationPoint.X >= farBorderDistance && AfterRotationPoint.Y >= farBorderDistance &&
                                  AfterRotationPoint.X <= width - farBorderDistance &&
                                  AfterRotationPoint.Y <= height - farBorderDistance;
            return isFarFromBorder;
        }
    }
}
