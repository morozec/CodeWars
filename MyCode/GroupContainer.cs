using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class GroupContainer
    {
        public List<Vehicle> Vehicles { get; set; }
        public Point Center { get; set; }

        public GroupContainer()
        {
            Vehicles = new List<Vehicle>();
        }

        public void AddVehicle(Vehicle vehicle)
        {
            Vehicles.Add(vehicle);
            Center = new Point(Vehicles.Select(v => v.X).Average(), Vehicles.Select(v => v.Y).Average());
        }

        public void AddGroupContainer(GroupContainer gc)
        {
            Vehicles.AddRange(gc.Vehicles);
            Center = new Point(Vehicles.Select(v => v.X).Average(), Vehicles.Select(v => v.Y).Average());
        }
    }
}
