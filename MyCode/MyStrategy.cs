using System;
using System.Collections.Generic;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.AStar;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode;
using IPA.AStar;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        public enum Ownership
        {
            ANY,
            ALLY,
            ENEMY
        }

        //static MyStrategy()
        //{
        //    Debug.connect("localhost", 13579);
        //}

        private const double Tolerance = 1E-3;
        private const int MoveToEnemyTicks = 10;

        private const double FarBorderDistance = 100;
        private const double ShootingDistance = 64d;

        private const int VehiclesCountAdvantage = 100;
        private const double VehiclesCoeffAdvantage = 2;

        private const double NuclearStrikeDistDelta = 30;

        private const double GroupMaxRadius = 15;

        private const double MaxAngle = Math.PI/180*2;

        private const double NuclearCompressionFactor = 0.1d;

        private const double SquardDelta = 6;
        private const double PrepareCompressinFactor = 0.75;
        private const double MoveEnemyPointTolerance = 10d;

        private const double MinNuclearStrikeCount = 10;

        private readonly IList<Point> _airKeyPoints = new List<Point>
        {
            new Point(119, 193),
            new Point(193, 193)
        };

        private readonly Queue<Action<Move>> _delayedMoves = new Queue<Action<Move>>();

        private readonly IList<Point> _groundKeyPoints = new List<Point>
        {
            new Point(45, 119),
            new Point(119, 119),
            new Point(193, 119)
        };

        private readonly IDictionary<VehicleType, int> _groupIndexes = new Dictionary<VehicleType, int>();

        private readonly IDictionary<VehicleType, IList<VehicleType>> _preferredVehicleTypes =
            new Dictionary<VehicleType, IList<VehicleType>>
            {
                {VehicleType.Fighter, new List<VehicleType> {VehicleType.Helicopter, VehicleType.Fighter}},
                {
                    VehicleType.Helicopter,
                    new List<VehicleType>
                    {
                        VehicleType.Tank,
                        VehicleType.Arrv,
                        VehicleType.Helicopter,
                        VehicleType.Ifv,
                        VehicleType.Fighter
                    }
                },
                {
                    VehicleType.Tank,
                    new List<VehicleType>
                    {
                        VehicleType.Ifv,
                        VehicleType.Arrv,
                        VehicleType.Tank,
                        VehicleType.Helicopter
                    }
                },
                {
                    VehicleType.Ifv,
                    new List<VehicleType>
                    {
                        VehicleType.Helicopter,
                        VehicleType.Arrv,
                        VehicleType.Ifv,
                        VehicleType.Fighter,
                        VehicleType.Tank
                    }
                }
            };

        private readonly IDictionary<int, VehicleType> _reversedGroupIndexes = new Dictionary<int, VehicleType>();
        private readonly IDictionary<long, int> _updateTickByVehicleId = new Dictionary<long, int>();

        private readonly IDictionary<long, Vehicle> _vehicleById = new Dictionary<long, Vehicle>();

        private IDictionary<VehicleType, IList<APoint>> _airAStarPathes = new Dictionary<VehicleType, IList<APoint>>();
        private IDictionary<VehicleType, int> _airPathIndexes = new Dictionary<VehicleType, int>();
        private IDictionary<int, VehicleType> _airPointsVehicleTypes;

        private AStar _aStar;
        

        private TornadoAction _currentTornadoAction = TornadoAction.MoveCenter;
        private Player _enemy;

        private double _enemyNuclearStrikeX;
        private double _enemyNuclearStrikeY;

        //private Point _enemyPoint;
        private Game _game;

        private IDictionary<VehicleType, IList<APoint>> _groundAStarPathes= new Dictionary<VehicleType, IList<APoint>>();
        private IDictionary<VehicleType, int> _groundPathIndexes = new Dictionary<VehicleType, int>();
        private IDictionary<int, VehicleType> _groundPointsVehicleTypes;

        private int _groupIndex = 1;
        private bool _isAirGroupsSet;
        private bool _isGroudGroupsSet;
        private bool _isVerticalCenterMove = true;


        private Player _me;
        private Move _move;


        private Random _random;

        private readonly IDictionary<int, SandvichAction> _sandvichActions = new Dictionary<int, SandvichAction>()
        {
            {1, SandvichAction.AStarMove },
            {2, SandvichAction.AStarMove },
        };
        private readonly IDictionary<int, bool> _isGroupCompressed = new Dictionary<int, bool>()
        {
            {1, false },
            {2, false },
        };

        private readonly IDictionary<int, double> _groupEndMovementTime = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

        //private readonly IDictionary<int, double> _currentGroupAngle = new Dictionary<int, double>()
        //{
        //    {1, 0d},
        //    {2, 0d},
        //};

        private readonly IDictionary<int, Point> _currentMoveEnemyPoint = new Dictionary<int, Point>()
        {
            {1, new Point(0d, 0d)},
            {2, new Point(0d, 0d)},
        };

        private int _selectedGroupId = -1;

        private TerrainType[][] _terrainTypeByCellXY;
        private double _tornadoRadius = 100;

        private int _tornadoVehiclesCount = 500;
        private WeatherType[][] _weatherTypeByCellXY;
        private World _world;

        

        /// <summary>
        ///     �������� ����� ���������, �������������� ���������� ������. ���������� ������ ���.
        /// </summary>
        /// <param name="me"></param>
        /// <param name="world"></param>
        /// <param name="game"></param>
        /// <param name="move"></param>
        public void Move(Player me, World world, Game game, Move move)
        {
            //Debug.beginPost();
            //Debug.circle(200, 200, 10, 150);

            InitializeStrategy(world, game);
            InitializeTick(me, world, game, move);

            if (_world.TickIndex == 0)
            {
                //SetGroudGroups();
                //SetAirGroups();
                AirVehiclesInit();
                GroundVehiclesInit();
            }
            else
            {
                if (me.RemainingActionCooldownTicks > 0) return;
                if (ExecuteDelayedMove()) return;
                SandvichMove(1, IsGroundAStarMoveFinished, GroundShift, GroundCompress);
                SandvichMove(2, IsAirAStarMoveFinished, AirShift, AirCompress);
            }

            ExecuteDelayedMove();

            //Debug.endPost();
        }

        private void GroundVehiclesInit()
        {
            SetGroudGroups();

            var vehicles = new Dictionary<VehicleType, IList<Vehicle>>
                    {
                        {VehicleType.Arrv, GetVehicles(Ownership.ALLY, VehicleType.Arrv)},
                        {VehicleType.Ifv, GetVehicles(Ownership.ALLY, VehicleType.Ifv)},
                        {VehicleType.Tank, GetVehicles(Ownership.ALLY, VehicleType.Tank)}
                    };

            var variants = new List<VariantContainer>()
            {
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetDistance(_groundKeyPoints[2])),
                    },
                },
            };

            variants.Sort();
            var bestVariant = variants.First();
            _groundPointsVehicleTypes = new Dictionary<int, VehicleType>();
            _groundPointsVehicleTypes.Add(0, bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 0).VehicleType);
            _groundPointsVehicleTypes.Add(1, bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 1).VehicleType);
            _groundPointsVehicleTypes.Add(2, bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 2).VehicleType);


            //for (var i = 0; i < 3; ++ i)
            //{
            //    foreach (var key0 in vehicles.Keys)
            //    {
            //        //pointVtContainers.Add(new PointVehilceTypeContainer(i, key,
            //        //    GetVehiclesCenter(vehicles[key]).GetDistance(_groundKeyPoints[i])));
            //        var gotKeysTmp = new List<VehicleType>() { key0 };
            //        for (var j = 1; j < 3; ++j)
            //        {
            //            foreach (var key1 in vehicles.Keys.Where(k => !gotKeysTmp.Contains(k)))
            //            {

            //            }
            //        }
            //    }
            //}
            //pointVtContainers = pointVtContainers.OrderBy(c => c.Distance).ToList();
            //var gotPoints = new List<int>();
            //var gotKeys = new List<VehicleType>();
            //_groundPointsVehicleTypes = new Dictionary<int, VehicleType>();
            //foreach (var item in pointVtContainers)
            //{
            //    if (gotPoints.Contains(item.PointIndex) || gotKeys.Contains(item.VehicleType)) continue;
            //    _groundPointsVehicleTypes.Add(item.PointIndex, item.VehicleType);
            //    gotPoints.Add(item.PointIndex);
            //    gotKeys.Add(item.VehicleType);
            //}

            var needMove = false;
            foreach (var pointIndex in _groundPointsVehicleTypes.Keys)
            {
                var currentType = _groundPointsVehicleTypes[pointIndex];
                var currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                if (Math.Abs(GetVehiclesCenter(currentVehicles).GetDistance(_groundKeyPoints[pointIndex])) > Tolerance)
                {
                    GroundMakeMoveToKeyPoint(currentType, currentVehicles, pointIndex);
                    needMove = true;
                }
            }

            if (!needMove) Scale(GetGroudVehicles(Ownership.ALLY), 1);
        }

        private void AirVehiclesInit()
        {
            SetAirGroups();

            var vehicles = new Dictionary<VehicleType, IList<Vehicle>>
                    {
                        {VehicleType.Fighter, GetVehicles(Ownership.ALLY, VehicleType.Fighter)},
                        {VehicleType.Helicopter, GetVehicles(Ownership.ALLY, VehicleType.Helicopter)},
                    };

            var variants = new List<VariantContainer>()
            {
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Fighter,
                            GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetDistance(_airKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Helicopter,
                            GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetDistance(_airKeyPoints[1])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Helicopter,
                            GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetDistance(_airKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Fighter,
                            GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetDistance(_airKeyPoints[1])),
                    },
                },
            };

            variants.Sort();
            var bestVariant = variants.First();
            _airPointsVehicleTypes = new Dictionary<int, VehicleType>();
            _airPointsVehicleTypes.Add(0, bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 0).VehicleType);
            _airPointsVehicleTypes.Add(1, bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 1).VehicleType);

            var needMove = false;
            foreach (var pointIndex in _airPointsVehicleTypes.Keys)
            {
                var currentType = _airPointsVehicleTypes[pointIndex];
                var currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                if (Math.Abs(GetVehiclesCenter(currentVehicles).GetDistance(_airKeyPoints[pointIndex])) > Tolerance)
                {
                    AirMakeMoveToKeyPoint(currentType, currentVehicles, pointIndex);
                    needMove = true;
                }
            }

            if (!needMove) Scale(GetGroudVehicles(Ownership.ALLY), 2);
        }


        private void GroundMakeMoveToKeyPoint(VehicleType vehicleType, IList<Vehicle> vehicles, int pointIndex)
        {
            var startI =
                (int)
                    ((GetVehiclesCenter(vehicles).X - 8 + AStar.SquareSize/2)/AStar.SquareSize) - 1;
            var startJ =
                (int)
                    ((GetVehiclesCenter(vehicles).Y - 8 + AStar.SquareSize/2)/AStar.SquareSize) - 1;
            var path = _aStar.GetPath(startI, startJ, pointIndex, 1);
            _groundAStarPathes.Add(vehicleType, _aStar.GetStraightPath(path));
            _groundPathIndexes.Add(vehicleType, 1);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });

            var destX = (_groundAStarPathes[vehicleType][1] as ASquare).CenterX;
            var destY = (_groundAStarPathes[vehicleType][1] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;
                _groupEndMovementTime[1] = Math.Max(_groupEndMovementTime[1],
                    _world.TickIndex + dist/ GetMaxSpeed(1));
            });
        }

        private void AirMakeMoveToKeyPoint(VehicleType vehicleType, IList<Vehicle> vehicles, int pointIndex)
        {
            var startI =
                (int)
                    ((GetVehiclesCenter(vehicles).X - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var startJ =
                (int)
                    ((GetVehiclesCenter(vehicles).Y - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var path = _aStar.GetPath(startI, startJ, pointIndex + 1, 2);
            _airAStarPathes.Add(vehicleType, _aStar.GetStraightPath(path));
            _airPathIndexes.Add(vehicleType, 1);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });

            var destX = (_airAStarPathes[vehicleType][1] as ASquare).CenterX;
            var destY = (_airAStarPathes[vehicleType][1] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;
                _groupEndMovementTime[2] = Math.Max(_groupEndMovementTime[2],
                    _world.TickIndex + dist / GetMaxSpeed(2));
            });
        }

        private void Scale(IList<Vehicle> vehicles, int groupId)
        {
            //�������, ����� ����� ������ ���� ��������
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Group = groupId;
            });
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = GetMaxSpeed(groupId);
            });
            

            _sandvichActions[groupId] = SandvichAction.Scaling;

            var minY = vehicles.Min(v => v.Y);
            for (var i = 0; i < 5; ++i)
            {
                var y = minY + GetMaxSpeed(groupId) + SquardDelta*i;

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - Tolerance;
                    move.Bottom = y + Tolerance;
                    move.Left = vehicles.Min(v => v.X);
                    move.Right = vehicles.Max(v => v.X);
                });


                var moveY = SquardDelta*(5 - i);
                if (groupId == 1) moveY *= 2; //��� �������� �������

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = -moveY;

                    _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                        _world.TickIndex + moveY/GetMaxSpeed(groupId));
                });
            }

            var maxY = vehicles.Max(v => v.Y);
            for (var i = 0; i < 4; ++i)
            {
                var y = maxY + GetMaxSpeed(groupId) - SquardDelta*i;

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - Tolerance;
                    move.Bottom = y + Tolerance;
                    move.Left = vehicles.Min(v => v.X);
                    move.Right = vehicles.Max(v => v.X);
                });

                var moveY = SquardDelta*(4 - i);
                if (groupId == 1) moveY *= 2;

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = moveY;

                    _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                        _world.TickIndex + moveY/ GetMaxSpeed(groupId));
                });
            }
        }

        private delegate void Shift(int groupId);

        private void GroundShift(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Shifting;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[0];
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = -SquardDelta;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + SquardDelta/ GetMaxSpeed(groupId));
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[2];
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = SquardDelta;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + SquardDelta/GetMaxSpeed(groupId));
            });
        }

        private void AirShift(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Shifting; 

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = VehicleType.Fighter;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = -SquardDelta;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + SquardDelta/(_game.FighterSpeed*_game.RainWeatherSpeedFactor));
            });
        }

        private delegate void Compress(int groupId);

        private void GroundCompress(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Compressing;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[0];
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = AStar.SquareSize;
                move.Y = 0;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + AStar.SquareSize/ GetMaxSpeed(groupId));
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[2];
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = -AStar.SquareSize;
                move.Y = 0;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + AStar.SquareSize/ GetMaxSpeed(groupId));
            });
        }

        private void AirCompress(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Compressing;

            var helicoptersCenter = GetVehiclesCenter(GetVehicles(Ownership.ALLY, VehicleType.Helicopter));
            var figtersCenter = GetVehiclesCenter(GetVehicles(Ownership.ALLY, VehicleType.Fighter));

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Fighter;
            });

            var delta = helicoptersCenter.X - figtersCenter.X;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = delta;
                move.Y = 0;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + Math.Abs(delta)/(_game.FighterSpeed*_game.RainWeatherSpeedFactor));
            });
        }

        private void Compress2(double x, double y, double compressionCoeff, double time, int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Compressing2;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = compressionCoeff;
                move.X = x;
                move.Y = y;
                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId], _world.TickIndex + time);
            });
            _isGroupCompressed[groupId] = true;
        }

        private void RotateToEnemy(IList<Vehicle> vehicles, int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Rotating;

            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var rectangle = GetGroupRectangel(vehicles);
            var newAngle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, nearestGroup.Center));
            var turnAngle = newAngle - rectangle.TurnAngle;

            if (turnAngle > Math.PI) turnAngle -= 2*Math.PI;
            else if (turnAngle < -Math.PI) turnAngle += 2*Math.PI;

            if (turnAngle > Math.PI/2) turnAngle -= Math.PI;
            else if (turnAngle < -Math.PI/2) turnAngle += Math.PI;

            var radius = vehicles.Max(v => v.GetDistanceTo(centerPoint.X, centerPoint.Y));
            var speed = GetMaxSpeed(groupId);
            var angularSpeed = speed/radius;

            var turnTime = Math.Abs(turnAngle)/angularSpeed;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Rotate;
                move.X = centerPoint.X;
                move.Y = centerPoint.Y;
                move.Angle = turnAngle;
                _groupEndMovementTime[groupId] = _world.TickIndex + turnTime;
                move.MaxAngularSpeed = angularSpeed;
            });
        }

        private void MoveToEnemy(IList<Vehicle> vehicles, int groupId)
        {

            _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var isFarFromBorder = centerPoint.X >= FarBorderDistance && centerPoint.Y >= FarBorderDistance &&
                                  centerPoint.X <= _world.Width - FarBorderDistance &&
                                  centerPoint.Y <= _world.Height - FarBorderDistance;
            var hasAdvantege = HasAdvantage(groupId, nearestGroup);

            var rectangle = GetGroupRectangel(vehicles);

            var dx = isFarFromBorder && !hasAdvantege ? ShootingDistance*Math.Cos(rectangle.TurnAngle) : 0d;
            var dy = isFarFromBorder && !hasAdvantege ? ShootingDistance*Math.Sin(rectangle.TurnAngle) : 0d;


            var dist = centerPoint.GetDistance(nearestGroup.Center.X, nearestGroup.Center.Y);
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = nearestGroup.Center.X - centerPoint.X - dx;
                move.Y = nearestGroup.Center.Y - centerPoint.Y - dy;
                move.MaxSpeed = GetMaxSpeed(groupId);
                _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                _currentMoveEnemyPoint[groupId] = new Point(nearestGroup.Center.X, nearestGroup.Center.Y);
                //Math.Max(_endMovementTime,_world.TickIndex + dist / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });
        }

        private bool HasAdvantage(int groupId, GroupContainer enemyGroup)
        {
            var vehicles = GetVehicles(groupId, Ownership.ALLY);
            switch (groupId)
            {
                case 1:
                    return vehicles.Count - enemyGroup.Vehicles.Count >= VehiclesCountAdvantage ||
                              vehicles.Count * 1d / enemyGroup.Vehicles.Count >= VehiclesCoeffAdvantage;
                case 2:
                    var myWeight = vehicles.Count;
                    var enemyWeight = 0d;
                    foreach (var v in enemyGroup.Vehicles)
                    {
                        switch (v.Type)
                        {
                            case VehicleType.Fighter:
                                enemyWeight += 1;
                                break;
                            case VehicleType.Helicopter:
                                enemyWeight += 1;
                                break;
                            case VehicleType.Tank:
                                enemyWeight += 1;
                                break;
                            case VehicleType.Ifv:
                                enemyWeight += 3;
                                break;
                        }
                    }

                    return myWeight >= enemyWeight;

                default:
                    throw new Exception("Unknown group id");
            }
        }

        private bool MakeNuclearStrike(IList<Vehicle> vehicles, int groupId)
        {
            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            if (nearestGroup.Vehicles.Count < MinNuclearStrikeCount) return false;

            var canStrikeMyVehilces = vehicles.Where(v =>
                v.VisionRange >= v.GetDistanceTo(nearestGroup.Center.X, nearestGroup.Center.Y) + NuclearStrikeDistDelta);

            if (!canStrikeMyVehilces.Any()) return false;

            _sandvichActions[groupId] = SandvichAction.NuclearStrike;

            var faarestMyVehicles = canStrikeMyVehilces
                .OrderBy(v => v.GetDistanceTo(nearestGroup.Center.X, nearestGroup.Center.Y)).Last();

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.TacticalNuclearStrike;
                move.X = nearestGroup.Center.X;
                move.Y = nearestGroup.Center.Y;
                move.VehicleId = faarestMyVehicles.Id;
                _groupEndMovementTime[groupId] = _world.TickIndex + _game.TacticalNuclearStrikeDelay;
            });

            return true;
        }

        private void Uncompress(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Uncompress;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            _enemyNuclearStrikeX = _enemy.NextNuclearStrikeX;
            _enemyNuclearStrikeY = _enemy.NextNuclearStrikeY;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = 1/NuclearCompressionFactor;
                move.X = _enemy.NextNuclearStrikeX;
                move.Y = _enemy.NextNuclearStrikeY;
                _groupEndMovementTime[groupId] = _enemy.NextNuclearStrikeTickIndex;
            });
        }

        private delegate bool IsAStarMoveFinished(int groupId);

        private bool IsGroundAStarMoveFinished(int groupId)
        {
            //TODO: ������ ��� ��� ����� �������������

            var isFinished = true;
            foreach (var pointIndex in _groundPointsVehicleTypes.Keys)
            {
                var vehicleType = _groundPointsVehicleTypes[pointIndex];
                var currVehicles = GetVehicles(Ownership.ALLY, vehicleType);

                if (!currVehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                {
                    isFinished = false;
                    continue;
                }

                if (!_groundPathIndexes.ContainsKey(vehicleType)) continue;

                if (_groundPathIndexes[vehicleType] < _groundAStarPathes[vehicleType].Count - 1)
                {
                    isFinished = false;

                    _groundPathIndexes[vehicleType]++;

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Bottom = _world.Height;
                        move.Right = _world.Width;
                        move.VehicleType = vehicleType;
                    });

                    var destX = (_groundAStarPathes[vehicleType][_groundPathIndexes[vehicleType]] as ASquare).CenterX;
                    var destY = (_groundAStarPathes[vehicleType][_groundPathIndexes[vehicleType]] as ASquare).CenterY;
                    var dist = GetVehiclesCenter(currVehicles).GetDistance(destX, destY);

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = destX - GetVehiclesCenter(currVehicles).X;
                        move.Y = destY - GetVehiclesCenter(currVehicles).Y;

                        _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                            _world.TickIndex + dist / GetMaxSpeed(groupId));
                    });
                }
            }

            return isFinished;
        }

        private bool IsAirAStarMoveFinished(int groupId)
        {
            //TODO: ������ ��� ��� ����� �������������

            var isFinished = true;
            foreach (var pointIndex in _airPointsVehicleTypes.Keys)
            {
                var vehicleType = _airPointsVehicleTypes[pointIndex];
                var currVehicles = GetVehicles(Ownership.ALLY, vehicleType);

                if (!currVehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                {
                    isFinished = false;
                    continue;
                }

                if (!_airPathIndexes.ContainsKey(vehicleType)) continue;

                if (_airPathIndexes[vehicleType] < _airAStarPathes[vehicleType].Count - 1)
                {
                    isFinished = false;

                    _airPathIndexes[vehicleType]++;

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Bottom = _world.Height;
                        move.Right = _world.Width;
                        move.VehicleType = vehicleType;
                    });

                    var destX = (_airAStarPathes[vehicleType][_airPathIndexes[vehicleType]] as ASquare).CenterX;
                    var destY = (_airAStarPathes[vehicleType][_airPathIndexes[vehicleType]] as ASquare).CenterY;
                    var dist = GetVehiclesCenter(currVehicles).GetDistance(destX, destY);

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = destX - GetVehiclesCenter(currVehicles).X;
                        move.Y = destY - GetVehiclesCenter(currVehicles).Y;

                        _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                            _world.TickIndex + dist / GetMaxSpeed(groupId));
                    });
                }
            }

            return isFinished;
        }

        private double GetMaxSpeed(int groupId)
        {
            switch (groupId)
            {
                case 1:
                    return _game.TankSpeed * _game.SwampTerrainSpeedFactor;
                case 2:
                    return _game.HelicopterSpeed * _game.RainWeatherSpeedFactor;
                default:
                    throw new Exception("Unkonw group id");
            }
        }

        private bool NeedNuclearUncompress(IList<Vehicle> vehicles)
        {
            return
                vehicles.Any(
                    v =>
                        v.GetDistanceTo(_enemy.NextNuclearStrikeX, _enemy.NextNuclearStrikeY) <=
                        _game.TacticalNuclearStrikeRadius);
        }

        private Rectangle GetGroupRectangel(IList<Vehicle> vehicles)
        {
            var minX = vehicles.Min(v => v.X);
            var minY = vehicles.Min(v => v.Y);

            var maxX = vehicles.Max(v => v.X);
            var maxY = vehicles.Max(v => v.Y);

            var minXVehicles = vehicles.Where(v => Math.Abs(v.X - minX) < Tolerance);
            var minYVehicles = vehicles.Where(v => Math.Abs(v.Y - minY) < Tolerance);
            var maxXVehicles = vehicles.Where(v => Math.Abs(v.X - maxX) < Tolerance);
            var maxYVehicles = vehicles.Where(v => Math.Abs(v.Y - maxY) < Tolerance);

            if (minXVehicles.Count() > 1 || minYVehicles.Count() > 1 || maxXVehicles.Count() > 1 ||
                maxYVehicles.Count() > 1)
                return new Rectangle()
                {
                    Points =
                        new List<Point>()
                        {
                            new Point(minX, minY),
                            new Point(minX, maxY),
                            new Point(maxX, maxY),
                            new Point(maxX, minY)
                        }
                };

            return new Rectangle()
            {
                Points =
                    new List<Point>()
                    {
                        new Point(minXVehicles.Single().X, minXVehicles.Single().Y),
                        new Point(minYVehicles.Single().X, minYVehicles.Single().Y),
                        new Point(maxXVehicles.Single().X, maxXVehicles.Single().Y),
                        new Point(maxYVehicles.Single().X, maxYVehicles.Single().Y),
                    }
            };
        }

        private void SandvichMove(int groupId, IsAStarMoveFinished isAStarMoveFinished, Shift shift, Compress compress)
        {
            var sandvichAction = _sandvichActions[groupId];
            var isCompressed = _isGroupCompressed[groupId];
            var vehicles = GetVehicles(groupId, Ownership.ALLY);
            if (!vehicles.Any()) return;

            if (sandvichAction != SandvichAction.Uncompress && _enemy.NextNuclearStrikeTickIndex > -1 &&
                isCompressed && NeedNuclearUncompress(vehicles))
            {
                Uncompress(groupId);
                return;
            }

            switch (sandvichAction)
            {
                case SandvichAction.AStarMove:
                {
                    var isMoveFinished = isAStarMoveFinished(groupId);
                    if (isMoveFinished)
                    {
                        Scale(vehicles, groupId);
                    }
                    break;
                }

                case SandvichAction.Scaling:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        shift(groupId);
                    }
                    break;

                case SandvichAction.Shifting:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        compress(groupId);
                    }
                    break;

                case SandvichAction.Compressing:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        var centerPoint = GetVehiclesCenter(vehicles);
                        Compress2(centerPoint.X, centerPoint.Y, PrepareCompressinFactor, 100d, groupId); //TODO
                    }
                    break;

                case SandvichAction.Compressing2:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        RotateToEnemy(vehicles, groupId);
                    }
                    break;
                case SandvichAction.Rotating:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        MoveToEnemy(vehicles, groupId);
                    }
                    break;
                case SandvichAction.MovingToEnemy:
                {
                    var centerPoint = GetVehiclesCenter(vehicles);
                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

                    var angle = MathHelper.GetAnlge(
                        new Vector(centerPoint,
                            new Point(centerPoint.X + 100, centerPoint.Y)),
                        new Vector(centerPoint, nearestGroup.Center));

                    var rectangle = GetGroupRectangel(vehicles);

                    if (_me.RemainingNuclearStrikeCooldownTicks == 0 && MakeNuclearStrike(vehicles, groupId))
                    {
                            //���� �������
                    }
                    else if (Math.Abs(rectangle.TurnAngle - angle) > MaxAngle &&
                             _world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        RotateToEnemy(vehicles, groupId);
                    }
                    else if (_world.TickIndex >= _groupEndMovementTime[groupId] &&
                             _currentMoveEnemyPoint[groupId].GetDistance(nearestGroup.Center) > MoveEnemyPointTolerance)
                    {
                        MoveToEnemy(vehicles, groupId);
                    }

                    break;
                }
                case SandvichAction.NuclearStrike:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        MoveToEnemy(vehicles, groupId);
                    }
                    break;
                case SandvichAction.Uncompress:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        Compress2(_enemyNuclearStrikeX, _enemyNuclearStrikeY, NuclearCompressionFactor,
                            _game.TacticalNuclearStrikeDelay, groupId);
                    }
                    break;
            }
        }

        private IList<GroupContainer> GetEnemyVehicleGroups()
        {
            IList<GroupContainer> groupContainers = new List<GroupContainer>();
            var enemyVehicles = GetVehicles(Ownership.ENEMY);

            var counter = 0;
            foreach (var v in enemyVehicles)
            {
                counter++;
                var okGroupContainers = new List<GroupContainer>();
                foreach (var gc in groupContainers)
                {
                    if (gc.Vehicles.Any(gcV => gcV.GetDistanceTo(v) <= GroupMaxRadius))
                    {
                        okGroupContainers.Add(gc);
                    }
                }
                if (okGroupContainers.Count == 0)
                {
                    var gc = new GroupContainer();
                    gc.AddVehicle(v);
                    groupContainers.Add(gc);
                }
                else if (okGroupContainers.Count == 1)
                {
                    okGroupContainers[0].AddVehicle(v);
                }
                else
                {
                    for (var i = 1; i < okGroupContainers.Count; ++i)
                    {
                        okGroupContainers[0].AddVehicle(v);
                        okGroupContainers[0].AddGroupContainer(okGroupContainers[i]);
                        groupContainers.Remove(okGroupContainers[i]);
                    }
                }
            }

            return groupContainers;
        }

        private GroupContainer GetNearestEnemyGroup(IList<GroupContainer> enemyGroups, double centerX, double centerY)
        {
            GroupContainer nearestGroup = null;
            var minDist = double.MaxValue;
            foreach (var eg in enemyGroups)
            {
                var dist = eg.Center.GetDistance(centerX, centerY);
                if (dist < minDist)
                {
                    minDist = dist;
                    nearestGroup = eg;
                }
            }

            return nearestGroup;
        }


        /// <summary>
        ///     ������������� ���������.
        /// </summary>
        /// <param name="world"></param>
        /// <param name="game"></param>
        private void InitializeStrategy(World world, Game game)
        {
            if (_random == null)
            {
                _random = new Random();
                _terrainTypeByCellXY = world.TerrainByCellXY;
                _weatherTypeByCellXY = world.WeatherByCellXY;
                _aStar = new AStar(world);

                //������� ������
                //var vehicleTypes = Enum.GetValues(typeof(VehicleType)).Cast<VehicleType>();
                //foreach (var vehicleType in vehicleTypes)
                //{

                //    _groupIndexes.Add(vehicleType, _groupIndex);
                //    _reversedGroupIndexes.Add(_groupIndex, vehicleType);
                //    _groupIndex++;

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.ClearAndSelect;
                //        move.Right = _world.Width;
                //        move.Bottom = _world.Height;
                //        move.VehicleType = vehicleType;
                //    });

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.Assign;
                //        move.Group = _groupIndexes[vehicleType];
                //    });
                //}
            }
        }

        /// <summary>
        ///     ��������� ��� ������� ������ � ����� ������ ��� ��������� ������� � ���, � ����� ������������� �������� � ������
        ///     ������� � ������� ���������� ��������� � ���������.
        /// </summary>
        /// <param name="me"></param>
        /// <param name="world"></param>
        /// <param name="game"></param>
        /// <param name="move"></param>
        private void InitializeTick(Player me, World world, Game game, Move move)
        {
            _me = me;
            _world = world;
            _game = game;
            _move = move;

            _enemy = world.Players.Single(p => !p.IsMe);

            foreach (var vehicle in world.NewVehicles)
            {
                _vehicleById.Add(vehicle.Id, vehicle);
                _updateTickByVehicleId.Add(vehicle.Id, world.TickIndex);
            }

            foreach (var vehicleUpdate in world.VehicleUpdates)
            {
                var vehicleId = vehicleUpdate.Id;
                if (vehicleUpdate.Durability == 0)
                {
                    _vehicleById.Remove(vehicleId);
                    _updateTickByVehicleId.Remove(vehicleId);
                }
                else
                {
                    _vehicleById[vehicleId] = new Vehicle(_vehicleById[vehicleId], vehicleUpdate);
                    _updateTickByVehicleId[vehicleId] = world.TickIndex;
                }
            }
        }

        /// <summary>
        ///     ������ ���������� �������� �� ������� � ��������� ���.
        /// </summary>
        /// <returns>���������� true, ���� � ������ ���� ���������� �������� ���� ������� � ���������.</returns>
        private bool ExecuteDelayedMove()
        {
            if (!_delayedMoves.Any()) return false;

            var delayedMove = _delayedMoves.Dequeue();
            delayedMove.Invoke(_move);

            return true;
        }
       
        public static bool IsSameTypes(VehicleType vt1, VehicleType vt2)
        {
            if ((vt1 == VehicleType.Fighter || vt1 == VehicleType.Helicopter) && (vt2 == VehicleType.Fighter ||
                                                                                  vt2 == VehicleType.Helicopter))
                return true;
            if (vt1 != VehicleType.Fighter && vt1 != VehicleType.Helicopter && vt2 != VehicleType.Fighter &&
                vt2 != VehicleType.Helicopter)
                return true;
            return false;
        }

       
        private void SetAirGroups()
        {
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Fighter;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Helicopter;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = 2;
            });
        }

        private void SetGroudGroups()
        {
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Arrv;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Ifv;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Tank;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = 1;
            });
        }

        public Point GetVehiclesCenter(IList<Vehicle> vehicles)
        {
            return new Point(vehicles.Select(v => v.X).Average(), vehicles.Select(v => v.Y).Average());
        }

        /// <summary>
        ///     ��������������� �����, ����������� ��� ���������� ���� ������� �������� ������ ��� �������, �����, ��� ������
        ///     �������� ���������� ������ �������.
        /// </summary>
        /// <param name="vehicleType"></param>
        /// <returns></returns>
        private IList<Vehicle> GetPreferredTargetVehilces(VehicleType vehicleType)
        {
            if (!_preferredVehicleTypes.ContainsKey(vehicleType)) return null;
            IList<Vehicle> targetTypeEnemyVehicles = new List<Vehicle>();
            var preferedTypes = _preferredVehicleTypes[vehicleType];
            foreach (var type in preferedTypes)
            {
                targetTypeEnemyVehicles = GetVehicles(Ownership.ENEMY, type);
                if (targetTypeEnemyVehicles.Any()) break;
            }

            return targetTypeEnemyVehicles;
        }

        private enum TornadoAction
        {
            None,
            Rotate,
            MoveCenter,
            RotateToEnemy,
            MoveToEnemy
        }

        private enum SandvichAction
        {
            AStarMove,
            Scaling,
            Shifting,
            Compressing,
            Compressing2,
            Rotating,
            MovingToEnemy,
            NuclearStrike,
            Uncompress
        }

        #region GetVehicles

        private IList<Vehicle> GetGroudVehicles(Ownership ownership = Ownership.ANY)
        {
            var vehicles =
                _vehicleById.Values.Where(
                    v => v.Type == VehicleType.Arrv || v.Type == VehicleType.Ifv || v.Type == VehicleType.Tank).ToList();
            switch (ownership)
            {
                case Ownership.ALLY:
                    vehicles = vehicles.Where(v => v.PlayerId == _me.Id).ToList();
                    break;
                case Ownership.ENEMY:
                    vehicles = vehicles.Where(v => v.PlayerId != _me.Id).ToList();
                    break;
            }
            return vehicles;
        }

        private IList<Vehicle> GetAirVehicles(Ownership ownership = Ownership.ANY)
        {
            var vehicles =
                _vehicleById.Values.Where(
                    v => v.Type == VehicleType.Fighter || v.Type == VehicleType.Helicopter).ToList();
            switch (ownership)
            {
                case Ownership.ALLY:
                    vehicles = vehicles.Where(v => v.PlayerId == _me.Id).ToList();
                    break;
                case Ownership.ENEMY:
                    vehicles = vehicles.Where(v => v.PlayerId != _me.Id).ToList();
                    break;
            }
            return vehicles;
        }

        private IList<Vehicle> GetVehicles(Ownership ownership = Ownership.ANY, VehicleType? vehicleType = null)
        {
            var vehicles = _vehicleById.Values.ToList();
            switch (ownership)
            {
                case Ownership.ALLY:
                    vehicles = vehicles.Where(v => v.PlayerId == _me.Id).ToList();
                    break;
                case Ownership.ENEMY:
                    vehicles = vehicles.Where(v => v.PlayerId != _me.Id).ToList();
                    break;
            }

            if (vehicleType != null)
            {
                vehicles = vehicles.Where(v => v.Type == vehicleType.Value).ToList();
            }
            return vehicles;
        }

        public IList<Vehicle> GetVehicles(int groupIndex, Ownership ownership = Ownership.ANY)
        {
            var vehicles =
                _vehicleById.Values.Where(x => x.PlayerId == _me.Id && x.Groups.Contains(groupIndex)).ToList();

            switch (ownership)
            {
                case Ownership.ALLY:
                    vehicles = vehicles.Where(v => v.PlayerId == _me.Id).ToList();
                    break;
                case Ownership.ENEMY:
                    vehicles = vehicles.Where(v => v.PlayerId != _me.Id).ToList();
                    break;
            }

            return vehicles;
        }

        #endregion
    }
}