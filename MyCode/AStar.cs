using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.AStar;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;
using IPA.AStar;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode
{
    public class AStar
    {
        public static double SquareSize = 74;
        public static double BigWeight = 999999;

        private const double _startX = 8;
        private const double _startY = 8;

        private ASquare[,] _table;
        private IList<ASquare> _squares;
        private ASquare _startSquare;
        private int _n;
        private int _m;

        public AStar(World world)
        {
            _squares = new List<ASquare>();
            _n = (int)(world.Width / SquareSize);
            _m = (int)(world.Height / SquareSize);

            _table = new ASquare[_n, _m];

            for (var i = 0; i < _n; ++i)
            {
                for (var j = 0; j < _m; ++j)
                {
                    var square = new ASquare(
                        SquareSize,
                        _startX + i * SquareSize,
                        _startY + j * SquareSize,
                        1d,
                        i + ":" + j);

                    _squares.Add(square);

                    _table[i, j] = square;
                }
            }

            for (var i = 0; i < _n; ++i)
            {
                for (var j = 0; j < _m; ++j)
                {
                    var neighbors = new List<ASquare>();
                    if (i != 0)
                    {
                        neighbors.Add(_table[i - 1, j]);
                    }
                    if (i != _n - 1)
                    {
                        neighbors.Add(_table[i + 1, j]);
                    }
                    if (j != 0)
                    {
                        neighbors.Add(_table[i, j - 1]);
                    }
                    if (j != _m - 1)
                    {
                        neighbors.Add(_table[i, j + 1]);
                    }

                    //if (i != 0 && j != 0)
                    //{
                    //    neighbors.Add(_table[i - 1, j - 1]);
                    //}

                    //if (i != _n - 1 && j != _m - 1)
                    //{
                    //    neighbors.Add(_table[i + 1, j + 1]);
                    //}

                    //if (i != 0 && j != _m - 1)
                    //{
                    //    neighbors.Add(_table[i - 1, j + 1]);
                    //}

                    //if (i != _n - 1 && j != 0)
                    //{
                    //    neighbors.Add(_table[i + 1, j - 1]);
                    //}

                    var square = _table[i, j];
                    square.Neighbors = neighbors;
                }
            }
        }

        public void UpdateDynamicAStar(
            int groupIndex, IList<Vehicle> vehicles, IDictionary<int, VehicleType> reversedGroupIndexes, MyStrategy ms)
        {
            var centerX = vehicles.Average(v => v.X);
            var centerY = vehicles.Average(v => v.Y);

            var myLeftX = centerX - SquareSize / 2;
            var leftN = (int)(myLeftX / SquareSize);

            var myTopY = centerY - SquareSize / 2;
            var topM = (int)(myTopY / SquareSize);

            for (var i = 0; i < _n; ++i)
            {
                for (var j = 0; j < _m; ++j)
                {
                    var square = _table[i, j];
                    square.X = _startX + i * SquareSize;
                    square.Y = _startY + j * SquareSize;
                    square.Weight = 1d;
                }
            }

            _startSquare = _table[leftN, topM];

            foreach (var index in reversedGroupIndexes.Keys)
            {
                if (index == groupIndex) continue;
                if (!MyStrategy.IsSameTypes(reversedGroupIndexes[groupIndex], reversedGroupIndexes[index]))
                    continue; // они друг другу не мешают

                var currVeichles = ms.GetVehicles(index, MyStrategy.Ownership.ALLY);
                if (!currVeichles.Any()) continue; // все сдохли

                var currCenterX = currVeichles.Average(v => v.X);
                var currCenterY = currVeichles.Average(v => v.Y);

                var currLeftX = currCenterX - SquareSize / 2;
                var currLeftN = (int)(currLeftX / SquareSize);

                var currTopY = currCenterY - SquareSize / 2;
                var currTopM = (int)(currTopY / SquareSize);

                _table[currLeftN, currTopM].Weight = BigWeight;
            }


            //if (leftN > 0 && _table[leftN - 1, topM].Weight == BigWeight)
            //{
            //    if (topM > 0) _table[leftN - 1, topM - 1].Weight = BigWeight;
            //    if (topM < _m - 1) _table[leftN - 1, topM + 1].Weight = BigWeight;
            //}

            //if (topM > 0 && _table[leftN, topM - 1].Weight == BigWeight)
            //{
            //    if (leftN > 0) _table[leftN - 1, topM - 1].Weight = BigWeight;
            //    if (leftN < _n - 1) _table[leftN + 1, topM - 1].Weight = BigWeight;
            //}

            //if (leftN < _n - 1 && _table[leftN + 1, topM].Weight == BigWeight)
            //{
            //    if (topM > 0) _table[leftN + 1, topM - 1].Weight = BigWeight;
            //    if (topM < _m - 1) _table[leftN + 1, topM + 1].Weight = BigWeight;
            //}

            //if (topM < _m - 1 && _table[leftN, topM + 1].Weight == BigWeight)
            //{
            //    if (leftN > 0) _table[leftN - 1, topM + 1].Weight = BigWeight;
            //    if (leftN < _n - 1) _table[leftN + 1, topM + 1].Weight = BigWeight;
            //}
        }

        public int GetSquareI(double x)
        {
            var res = (int)((x - _startX) / SquareSize);
            if (res < 0) return 0;
            if (res > _n - 1) return _n - 1;
            return res;
        }

        public int GetSquareJ(double y)
        {
            var res = (int)((y - _startY) / SquareSize);
            if (res < 0) return 0;
            if (res > _m - 1) return _m - 1;
            return res;
        }

        public IList<APoint> GetPath(int startI, int startJ, int goalI, int goalJ)
        {
            var path = Calculator.GetPath(_table[startI, startJ], _table[goalI, goalJ], _squares);
            return path;
        }

        public ASquare GetSupportAPoint(int goalI, int goalJ)
        {
            ASquare resSquare = null;
            if (goalI > 0 && _table[goalI - 1, goalJ].Weight < BigWeight)
            {
                resSquare = _table[goalI - 1, goalJ];
            }
            else if (goalJ > 0 && _table[goalI, goalJ - 1].Weight < BigWeight)
            {
                resSquare = _table[goalI, goalJ - 1];
            }
            else if (goalI > 0 && goalJ > 0 && _table[goalI - 1, goalJ - 1].Weight < BigWeight)
            {
                resSquare = _table[goalI - 1, goalJ - 1];
            }
            return resSquare;
        }

        public IList<APoint> GetStraightPath(IList<APoint> path)
        {
            var result = new List<APoint>() {path[0]};
         
            var dx = (path[1] as ASquare).X - (path[0] as ASquare).X;
            var dy = (path[1] as ASquare).Y - (path[0] as ASquare).Y;

            for (var i = 2; i < path.Count; ++i)
            {
                var currDx = (path[i] as ASquare).X - (path[i - 1] as ASquare).X;
                var currDy = (path[i] as ASquare).Y - (path[i - 1] as ASquare).Y;
                if (dx != currDx || dy != currDy)
                {
                    result.Add(path[i-1]);
                    dx = currDx;
                    dy = currDy;
                }
            }

            result.Add(path[path.Count - 1]);
            return result;
        }
    }
}

