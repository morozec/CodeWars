using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class VariantContainer : IComparable
    {
        private const double Tolerance = 1E-3;
        public IList<PointVehilceTypeContainer> PointVehilceTypeContainers { get; set; }
        
        public int CompareTo(object obj)
        {
            var otherVc = obj as VariantContainer;
            var maxDist = PointVehilceTypeContainers.Max(p => p.Distance);
            var otherMaxDist = otherVc.PointVehilceTypeContainers.Max(p => p.Distance);

            var zeroDistCount = PointVehilceTypeContainers.Count(p => Math.Abs(p.Distance) < Tolerance);
            var otherZeroDistCount = otherVc.PointVehilceTypeContainers.Count(p => Math.Abs(p.Distance) < Tolerance);

            if (zeroDistCount > otherZeroDistCount) return -1;
            else if (zeroDistCount < otherZeroDistCount) return 1;

            if (Math.Abs(maxDist - otherMaxDist) < Tolerance)
            {
                var sumDist = PointVehilceTypeContainers.Sum(p => p.Distance);
                var otherSumDist = otherVc.PointVehilceTypeContainers.Sum(p => p.Distance);
                if (Math.Abs(sumDist - otherSumDist) < Tolerance)
                {
                    return 0;
                }
                return sumDist > otherSumDist ? 1 : -1;
            }
            else
            {
                return maxDist > otherMaxDist ? 1 : -1;
            }
            
        }
    }
}
