using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class PointVehilceTypeContainer
    {
        public int PointIndex { get; set; }
        public VehicleType VehicleType { get; set; }
        public double Distance { get; set; }

        public PointVehilceTypeContainer(int pointIndex, VehicleType vehicleType, double distance)
        {
            PointIndex = pointIndex;
            VehicleType = vehicleType;
            Distance = distance;
        }
    }
}
