using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    /// <summary>
    /// Класс для работы с потенциальными полями
    /// </summary>
    public static class PotentialFieldsHelper
    {
        private const double Tolerance = 1E-3;
        private const double CloseBorderDist = 0d;
        private const double OrbitalWidth = 25d;

        public static Point GetAllyGroupRepulsiveFunction(IList<Vehicle> thisVehicles, IList<Vehicle> otherVehicles, double coeff)
        {
            var isThisGround = MyStrategy.IsGroundGroup(thisVehicles);
            var isThisAir = MyStrategy.IsAirGroup(thisVehicles);
            var isOtherGround = MyStrategy.IsGroundGroup(otherVehicles);
            var isOtherAir = MyStrategy.IsAirGroup(otherVehicles);

            if (isThisGround && !isOtherGround || isThisAir && !isOtherAir || isOtherGround && !isThisGround ||
                isOtherAir && !isThisAir) return new Point(0d, 0d);

            var myCenter = MyStrategy.GetVehiclesCenter(thisVehicles);
            var myRadius = MyStrategy.GetSandvichRadius(thisVehicles);
            var otherCenter = MyStrategy.GetVehiclesCenter(otherVehicles);
            var otherRadius = MyStrategy.GetSandvichRadius(otherVehicles);

            var centersDist = myCenter.GetDistance(otherCenter);
            if (centersDist > myRadius + otherRadius + MyStrategy.EnemyDangerousRadius) return new Point(0d, 0d);

            //Debug.circle(myCenter.X, myCenter.Y, myRadius + otherRadius + EnemyDangerousRadius, 0x00FF00);

            double x, y;
            if (centersDist < myRadius + otherRadius)
            {
                x = coeff * (myCenter.X - otherCenter.X) / centersDist;
                y = coeff * (myCenter.Y - otherCenter.Y) / centersDist;
            }
            else
            {
                x = 2 * coeff * (myCenter.X - otherCenter.X) * (1 / centersDist - 1 / (myRadius + otherRadius + MyStrategy.EnemyDangerousRadius));
                y = 2 * coeff * (myCenter.Y - otherCenter.Y) * (1 / centersDist - 1 / (myRadius + otherRadius + MyStrategy.EnemyDangerousRadius));
            }

            return new Point(x, y);
        }


        public static Point GetAllyNoGroupRepulsiveFunction(IList<Vehicle> thisVehicles, IList<Vehicle> otherVehicles, double coeff)
        {
            var isThisGround = MyStrategy.IsGroundGroup(thisVehicles);
            var isThisAir = MyStrategy.IsAirGroup(thisVehicles);
            var myCenter = MyStrategy.GetVehiclesCenter(thisVehicles);
            var myRadius = MyStrategy.GetSandvichRadius(thisVehicles);

            var resPoint = new Point(0d, 0d);

            foreach (var v in otherVehicles)
            {
                var isGroundVehicle = v.Type == VehicleType.Arrv || v.Type == VehicleType.Ifv ||
                                      v.Type == VehicleType.Tank;
                if (isThisGround && !isGroundVehicle) continue;
                if (isThisAir && isGroundVehicle) continue;

                var centersDist = myCenter.GetDistance(v.X, v.Y);
                if (centersDist > myRadius + MyStrategy.EnemyDangerousRadius) continue;

                double x, y;
                if (centersDist < myRadius)
                {
                    x = coeff * (myCenter.X - v.X) / centersDist;
                    y = coeff * (myCenter.Y - v.Y) / centersDist;
                }
                else
                {
                    x = 2 * coeff * (myCenter.X - v.X) * (1 / centersDist - 1 / (myRadius + MyStrategy.EnemyDangerousRadius));
                    y = 2 * coeff * (myCenter.Y - v.Y) * (1 / centersDist - 1 / (myRadius + MyStrategy.EnemyDangerousRadius));
                }

                resPoint = new Point(resPoint.X + x, resPoint.Y + y);
            }

            return resPoint;
        }


        public static Point GetBorderRepulsiveFunction(IList<Vehicle> vehicles, double worldWidth, double worldHeight)
        {
            var resPoint = new Point(0d, 0d);

            var center = MyStrategy.GetVehiclesCenter(vehicles);
            var radius = MyStrategy.GetSandvichRadius(vehicles);

            if (center.X - radius < CloseBorderDist) resPoint = new Point(resPoint.X + 1d, resPoint.Y);
            if (center.Y - radius < CloseBorderDist) resPoint = new Point(resPoint.X, resPoint.Y + 1d);
            if (center.X + radius > worldWidth - CloseBorderDist) resPoint = new Point(resPoint.X - 1d, resPoint.Y);
            if (center.Y + radius > worldHeight - CloseBorderDist) resPoint = new Point(resPoint.X, resPoint.Y - 1d);

            return resPoint;
        }

        public static Point GetAttractiveRadiusFunction(Point destPoint, double coeff, double radius, double x, double y)
        {
            var dist = destPoint.GetDistance(x, y);
            if (Math.Abs(dist - radius) < OrbitalWidth) return new Point(0d, 0d);
            if (dist > radius) return new Point(-coeff * (x - destPoint.X) / dist, -coeff * (y - destPoint.Y) / dist);

            double resX, resY;
            if (dist < radius / 2)
            {
                //TODO: деление на 0, если находимся в цетре круга
                resX = coeff * (x - destPoint.X) / dist;
                resY = coeff * (y - destPoint.Y) / dist;
            }
            else
            {
                resX = 2 * coeff * (x - destPoint.X) * (1 / dist - 1 / radius);
                resY = 2 * coeff * (y - destPoint.Y) * (1 / dist - 1 / radius);
            }

            return new Point(resX, resY);
        }

        public static Point GetAttractiveFunction(Point destPoint, double coeff, double x, double y)
        {
            var dist = destPoint.GetDistance(x, y);
            if (Math.Abs(dist) < Tolerance) return new Point(0d, 0d);
            return new Point(-coeff * (x - destPoint.X) / dist, -coeff * (y - destPoint.Y) / dist);
        }

        //public static Point GetEnemyVehicleRepulsiveFunction(Vehicle enemyVehicle, double coeff, IList<Vehicle> vehicles, bool isGroundAttak)
        //{
        //    var radius = GetActualShootingDistance(enemyVehicle, isGroundAttak) + MyStrategy.EnemyDangerousRadius;
        //    var centerPoint = MyStrategy.GetVehiclesCenter(vehicles);
        //    var dist = centerPoint.GetDistance(enemyVehicle.X, enemyVehicle.Y);
        //    if (dist > radius) return new Point(0d, 0d);
        //    var x = coeff * (centerPoint.X - enemyVehicle.X) * (1 / dist - 1 / radius) / Math.Pow(dist, 3d);
        //    var y = coeff * (centerPoint.Y - enemyVehicle.Y) * (1 / dist - 1 / radius) / Math.Pow(dist, 3d);
        //    return new Point(x, y);
        //}

        public static Point GetEnemyGroupRepulsiveFunction(GroupContainer groupContainer, double coeff,
            IList<Vehicle> vehicles)
        {
            var enemyRectangle =
                MathHelper.GetJarvisRectangle(groupContainer.Vehicles.Select(v => new Point(v.X, v.Y)).ToList());
            var myCenter = MyStrategy.GetVehiclesCenter(vehicles);

            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(myCenter, enemyRectangle, groupContainer.Center);
            var myCenterDist = myCenter.GetDistance(groupContainer.Center);
            var radius = groupContainer.Center.GetDistance(enemyCp) + MyStrategy.EnemyDangerousRadius;

            if (myCenterDist > radius) return new Point(0d, 0d);

            double x, y;
            if (myCenterDist < radius / 2)
            {
                x = coeff * (myCenter.X - groupContainer.Center.X) / myCenterDist;
                y = coeff * (myCenter.Y - groupContainer.Center.Y) / myCenterDist;
            }
            else
            {
                x = 2 * coeff * (myCenter.X - groupContainer.Center.X) * (1 / myCenterDist - 1 / radius);
                y = 2 * coeff * (myCenter.Y - groupContainer.Center.Y) * (1 / myCenterDist - 1 / radius);
            }

            return new Point(x, y);
        }

       
    }
}
