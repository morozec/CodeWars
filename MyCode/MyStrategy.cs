using System;
using System.Collections.Generic;
using System.Linq;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk;
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
        private const int MoveToEnemyTicks = 6;

        private const double ShootingDistance = 15d;
        private const double CloseFriendDistance = 100d;

        private const int WeakVehiclesCountAdvantage = 50;
        private const int StrongVehiclesCountAdvantage = 100;
        private const double WeakVehiclesCoeffAdvantage = 1.5;
        private const double StrongVehiclesCoeffAdvantage = 2;

        private const double GroupMaxRadius = 15;

        private const double MaxAngle = Math.PI/180*2;

        private const double NuclearCompressionFactor = 0.1d;

        private const double SquardDelta = 6;
        private const double PrepareCompressinFactor = 0.75;
        private const double MoveEnemyPointTolerance = 10d;

        private const int MinNuclearStrikeCount = 5;

        private const int SmallGroupVehiclesCount = 33;
        private const int ConsiderGroupVehiclesCount = 7;

        private const double MoreSideDist = 15d;

        private const double HpNuclerStrikeCoeff = 0.7;
        private const int GoToEnemyCpCount = 33;
        private const int WeakAviationHelicoptersCount = 50;
        private const double CloseToRectangleBorderDist = 8d;

        private const double CellSize = 32d;

        private bool _isFirstNuclearStrike = true;

        private readonly IList<Point> _airKeyPoints = new List<Point>
        {
            new Point(119, 119),
            new Point(193, 119)
        };

        private readonly Queue<Action<Move>> _delayedMoves = new Queue<Action<Move>>();
        private readonly Queue<Action<Move>> _importantDelayedMoves = new Queue<Action<Move>>();

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

        private bool _isEnemyNuclearStrikeConsidered = false;
        private bool _isMyNuclearStrikeConsidered = false;

        private int _nuclearVehicleGroup = -1;
        private long _nuclearVehicleId = -1;

        //private RotationContainer _rotationContainer;

        private readonly IDictionary<int, bool> _isRotating = new Dictionary<int, bool>()
        {
            {1, false},
            {2, false}
        };

        private Player _me;
        private Move _move;


        private Random _random;

        private readonly IDictionary<int, SandvichAction> _sandvichActions = new Dictionary<int, SandvichAction>()
        {
            {1, SandvichAction.AStarMove },
            {2, SandvichAction.AStarMove },
        };
        //private readonly IDictionary<int, bool> _isGroupCompressed = new Dictionary<int, bool>()
        //{
        //    {1, false },
        //    {2, false },
        //};

        private readonly IDictionary<int, double> _groupEndMovementTime = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

        private readonly IDictionary<int, int> _groupStartUncompressTick = new Dictionary<int, int>()
        {
            {1, -1},
            {2, -1},
        };

        private readonly IDictionary<int, double> _currentGroupAngle = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

        private readonly IDictionary<int, double> _tmpGroupAngle = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

        private readonly IDictionary<int, double> _currentAngularSpeed = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

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


        private double[][] _groundPotentialField;
        

        /// <summary>
        ///     ќсновной метод стратегии, осуществл€ющий управление армией. ¬ызываетс€ каждый тик.
        /// </summary>
        /// <param name="me"></param>
        /// <param name="world"></param>
        /// <param name="game"></param>
        /// <param name="move"></param>
        public void Move(Player me, World world, Game game, Move move)
        {
            //Debug.beginPost();
            //Draw(2);


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

                if (!_isMyNuclearStrikeConsidered && _me.RemainingNuclearStrikeCooldownTicks == 0 && !_importantDelayedMoves.Any() && MakeNuclearStrike())
                {
                    _delayedMoves.Clear();
                    _isMyNuclearStrikeConsidered = true;
                }
                else if (!_isEnemyNuclearStrikeConsidered && _enemy.NextNuclearStrikeTickIndex > -1 &&
                         !_importantDelayedMoves.Any())
                {
                    _delayedMoves.Clear();
                    _isEnemyNuclearStrikeConsidered = true;

                    if (_selectedGroupId == 1 || _selectedGroupId == -1)
                    {
                        var vehicles1 = GetVehicles(1, Ownership.ALLY);
                        if (vehicles1.Any() && NeedNuclearUncompress(vehicles1))
                        {
                            Uncompress(1);
                        }
                        var vehicles2 = GetVehicles(2, Ownership.ALLY);
                        if (vehicles2.Any() && NeedNuclearUncompress(vehicles2))
                        {
                            Uncompress(2);
                        }
                    }
                    else if (_selectedGroupId == 2)
                    {
                        var vehicles2 = GetVehicles(2, Ownership.ALLY);
                        if (vehicles2.Any() && NeedNuclearUncompress(vehicles2))
                        {
                            Uncompress(2);
                        }
                        var vehicles1 = GetVehicles(1, Ownership.ALLY);
                        if (vehicles1.Any() && NeedNuclearUncompress(vehicles1))
                        {
                            Uncompress(1);
                        }
                    }
                }

                if (ExecuteDelayedMove()) return;

                var enemyGroups = GetEnemyVehicleGroups();
                SetFacilitiesProduction(enemyGroups);

                if (ExecuteDelayedMove()) return;

                if (_selectedGroupId == 1 || _selectedGroupId == -1)
                {
                    SandvichMove(1, IsGroundAStarMoveFinished, GroundShift, GroundCompress);
                    if (ExecuteDelayedMove()) return;
                    SandvichMove(2, IsAirAStarMoveFinished, AirShift, AirCompress);
                }
                else if (_selectedGroupId == 2)
                {
                    SandvichMove(2, IsAirAStarMoveFinished, AirShift, AirCompress);
                    if (ExecuteDelayedMove())  return;
                    SandvichMove(1, IsGroundAStarMoveFinished, GroundShift, GroundCompress);
                }
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

            _airPointsVehicleTypes = new Dictionary<int, VehicleType>();

            var fCenter = GetVehiclesCenter(vehicles[VehicleType.Fighter]);
            var hCenter = GetVehiclesCenter(vehicles[VehicleType.Helicopter]);
            if (hCenter.GetDistance(45, 119) < Tolerance && (fCenter.GetDistance(45, 45) < Tolerance || fCenter.GetDistance(45, 193) < Tolerance))
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Helicopter);
                _airPointsVehicleTypes.Add(1, VehicleType.Fighter);
            }
            else if (fCenter.GetDistance(45, 119) < Tolerance &&
                     (hCenter.GetDistance(45, 45) < Tolerance || hCenter.GetDistance(45, 193) < Tolerance))
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Fighter);
                _airPointsVehicleTypes.Add(1, VehicleType.Helicopter);
            }
            else if (hCenter.GetDistance(45, 119) < Tolerance && fCenter.GetDistance(119, 119) < Tolerance)
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Helicopter);
                _airPointsVehicleTypes.Add(1, VehicleType.Fighter);
            }
            else if (fCenter.GetDistance(45, 119) < Tolerance && hCenter.GetDistance(119, 119) < Tolerance)
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Fighter);
                _airPointsVehicleTypes.Add(1, VehicleType.Helicopter);
            }
            else
            {
                var variants = new List<VariantContainer>()
                {
                    new VariantContainer()
                    {
                        PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                        {
                            new PointVehilceTypeContainer(0,
                                VehicleType.Fighter,
                                GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetDistance(_airKeyPoints[0])),
                            new PointVehilceTypeContainer(1,
                                VehicleType.Helicopter,
                                GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetDistance(_airKeyPoints[1])),
                        },
                    },
                    new VariantContainer()
                    {
                        PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                        {
                            new PointVehilceTypeContainer(0,
                                VehicleType.Helicopter,
                                GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetDistance(_airKeyPoints[0])),
                            new PointVehilceTypeContainer(1,
                                VehicleType.Fighter,
                                GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetDistance(_airKeyPoints[1])),
                        },
                    },
                };

                variants.Sort();
                var bestVariant = variants.First();

                _airPointsVehicleTypes.Add(0,
                    bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 0).VehicleType);
                _airPointsVehicleTypes.Add(1,
                    bestVariant.PointVehilceTypeContainers.Single(p => p.PointIndex == 1).VehicleType);
            }

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

            if (!needMove) Scale(GetAirVehicles(Ownership.ALLY), 2);
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

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });
            _selectedGroupId = -1;


            var destX = (_groundAStarPathes[vehicleType][1] as ASquare).CenterX;
            var destY = (_groundAStarPathes[vehicleType][1] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            var endPoint = new Point(destX, destY);
            var speed = GetGroupLineMaxSpeed(vehicles, endPoint);

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;
                _groupEndMovementTime[1] = Math.Max(_groupEndMovementTime[1],
                    _world.TickIndex + dist/ speed);
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
            var path = _aStar.GetPath(startI, startJ, pointIndex + 1, 1);
            _airAStarPathes.Add(vehicleType, _aStar.GetStraightPath(path));
            _airPathIndexes.Add(vehicleType, 1);

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });
            _selectedGroupId = -1;

            var destX = (_airAStarPathes[vehicleType][1] as ASquare).CenterX;
            var destY = (_airAStarPathes[vehicleType][1] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            var endPoint = new Point(destX, destY);
            var speed = GetGroupLineMaxSpeed(vehicles, endPoint);

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;
                _groupEndMovementTime[2] = Math.Max(_groupEndMovementTime[2],
                    _world.TickIndex + dist / speed);
            });
        }

        private void Scale(IList<Vehicle> vehicles, int groupId)
        {
            //костыль, чтобы потом удобно было выдел€ть
            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Group = groupId;
            });
            _selectedGroupId = groupId;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = GetGroupMaxSpeed(groupId);
            });
            

            _sandvichActions[groupId] = SandvichAction.Scaling;

            var minY = vehicles.Min(v => v.Y);
            for (var i = 0; i < 5; ++i)
            {
                var y = minY + GetGroupMaxSpeed(groupId) + SquardDelta*i;

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - Tolerance;
                    move.Bottom = y + Tolerance;
                    move.Left = vehicles.Min(v => v.X);
                    move.Right = vehicles.Max(v => v.X);
                });


                var moveY = SquardDelta*(5 - i);
                if (groupId == 1) moveY *= 2; //дл€ наземной техники

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = -moveY;

                    _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                        _world.TickIndex + moveY/GetGroupMaxSpeed(groupId));
                });
            }

            var maxY = vehicles.Max(v => v.Y);
            for (var i = 0; i < 4; ++i)
            {
                var y = maxY + GetGroupMaxSpeed(groupId) - SquardDelta*i;

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - Tolerance;
                    move.Bottom = y + Tolerance;
                    move.Left = vehicles.Min(v => v.X);
                    move.Right = vehicles.Max(v => v.X);
                });

                var moveY = SquardDelta*(4 - i);
                if (groupId == 1) moveY *= 2;

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = moveY;

                    _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                        _world.TickIndex + moveY/ GetGroupMaxSpeed(groupId));
                });
            }
        }

        private delegate void Shift(int groupId);

        private void GroundShift(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Shifting;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[0];
            });
            _selectedGroupId = -1;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = -SquardDelta;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + SquardDelta/ GetGroupMaxSpeed(groupId));
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[2];
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = SquardDelta;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + SquardDelta/GetGroupMaxSpeed(groupId));
            });
        }

        private void AirShift(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Shifting;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = VehicleType.Fighter;
            });
            _selectedGroupId = -1;

            _importantDelayedMoves.Enqueue(move =>
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

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[0];
            });
            _selectedGroupId = -1;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = AStar.SquareSize;
                move.Y = 0;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + AStar.SquareSize/ GetGroupMaxSpeed(groupId));
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = _groundPointsVehicleTypes[2];
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = -AStar.SquareSize;
                move.Y = 0;

                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                    _world.TickIndex + AStar.SquareSize/ GetGroupMaxSpeed(groupId));
            });
        }

        private void AirCompress(int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Compressing;

            var helicoptersCenter = GetVehiclesCenter(GetVehicles(Ownership.ALLY, VehicleType.Helicopter));
            var figtersCenter = GetVehiclesCenter(GetVehicles(Ownership.ALLY, VehicleType.Fighter));

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Fighter;
            });
            _selectedGroupId = -1;

            var delta = helicoptersCenter.X - figtersCenter.X;

            _importantDelayedMoves.Enqueue(move =>
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

            if (_selectedGroupId != groupId)
            {
                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = compressionCoeff;
                move.X = x;
                move.Y = y;
                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId], _world.TickIndex + time);
            });
            
        }

        private void RotateToEnemy(IList<Vehicle> vehicles, int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Rotating;

            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            //var rectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var newAngle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, nearestGroup.Center));

            var turnAngle = newAngle - _currentGroupAngle[groupId];

            if (turnAngle > Math.PI) turnAngle -= 2*Math.PI;
            else if (turnAngle < -Math.PI) turnAngle += 2*Math.PI;

            if (turnAngle > Math.PI/2) turnAngle -= Math.PI;
            else if (turnAngle < -Math.PI/2) turnAngle += Math.PI;

            var radius = vehicles.Max(v => v.GetDistanceTo(centerPoint.X, centerPoint.Y));
            var speed = GetGroupMaxSpeed(groupId);
            var angularSpeed = speed/radius;

            var turnTime = Math.Abs(turnAngle)/angularSpeed;

            if (_selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            var currentAngle = _currentGroupAngle[groupId];

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Rotate;
                move.X = centerPoint.X;
                move.Y = centerPoint.Y;
                move.Angle = turnAngle;
                _groupEndMovementTime[groupId] = _world.TickIndex + turnTime;
                move.MaxAngularSpeed = angularSpeed;


                _currentAngularSpeed[groupId] = turnAngle > 0 ? angularSpeed : -angularSpeed;
                _isRotating[groupId] = true;
                _tmpGroupAngle[groupId] = currentAngle;
                _currentGroupAngle[groupId] = newAngle;
            });
        }

        private void Draw(int groupId)
        {
            //var vehicles = GetVehicles(groupId);
            //if (!vehicles.Any()) return;
            //var center = GetVehiclesCenter(vehicles);

            //var enemyGroups = GetEnemyVehicleGroups();
            //var nearestGroup = GetNearestEnemyGroup(enemyGroups, center.X, center.Y);

            //var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            //var myrectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));

            //foreach (var p in enemyRectangle.Points)
            //{
            //    Debug.circleFill(p.X, p.Y, 2, 150);
            //}

            //foreach (var p in myrectangle.Points)
            //{
            //    Debug.circleFill(p.X, p.Y, 2, 0x00FFFF);
            //}

            //var centerPoint = GetVehiclesCenter(vehicles);
            //var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));

            //var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            //var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);

            //Debug.circleFill(myCp.X, myCp.Y, 4, 0x00FF00);
            //Debug.circleFill(enemyCp.X, enemyCp.Y, 4, 0xFF0000);

            if (_vehicleById.ContainsKey(_nuclearVehicleId) && _me.NextNuclearStrikeTickIndex > -1)
            {
                var nuclearVehilce = _vehicleById[_nuclearVehicleId];
                Debug.circleFill(nuclearVehilce.X, nuclearVehilce.Y, 2, 0x00000);
            }
            

        }

        private void MoveToFacility(IList<Vehicle> vehicles, int groupId, Facility facility)
        {
            var facilityPoint = new Point(facility.Left + _game.FacilityWidth / 2d,
                facility.Top + _game.FacilityHeight / 2d);

            //Debug.circleFill(facilityPoint.X, facilityPoint.Y, 4, 0x00000);

            var isStatic = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                                _currentMoveEnemyPoint[groupId].GetDistance(facilityPoint.X, facilityPoint.Y) <=
                                MoveEnemyPointTolerance;

            if (isStatic) return;

            _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

            if (_selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            var speed = GetGroupLineMaxSpeed(vehicles, facilityPoint);
            var center = GetVehiclesCenter(vehicles);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = facilityPoint.X - center.X;
                move.Y = facilityPoint.Y - center.Y;
                move.MaxSpeed = speed;
                _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                _currentMoveEnemyPoint[groupId] = new Point(facilityPoint.X, facilityPoint.Y);
            });
        }

        private void SetFacilitiesProduction(IList<GroupContainer> enemyGroups)
        {
            var myFacilities = _world.Facilities.Where(f =>
                f.Type == FacilityType.VehicleFactory && f.OwnerPlayerId == _me.Id && f.ProductionProgress == 0);
            if (!myFacilities.Any()) return;
            var orderedFacilities = myFacilities.OrderByDescending(f =>
            {
                var fcp = GetFacilityCenterPoint(f);
                var ng =  GetNearestEnemyGroup(enemyGroups, fcp.X, fcp.Y);
                return fcp.GetDistance(ng.Center);
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.SetupVehicleProduction;
                move.VehicleType = VehicleType.Helicopter;
                move.FacilityId = orderedFacilities.First().Id;
            });

        }

        private Point GetFacilityCenterPoint(Facility facility)
        {
            return new Point(facility.Left + _game.FacilityWidth / 2d, facility.Top + _game.FacilityHeight / 2d);
        }
        

        private void MoveToEnemy(IList<Vehicle> vehicles, int groupId)
        {
            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);
            
            var hasAdvantege = HasAdvantage(groupId, nearestGroup, enemyGroups, WeakVehiclesCountAdvantage, WeakVehiclesCoeffAdvantage);
            var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);

            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);
            var requiredDistToEnemyCenter = centerPoint.GetDistance(myCp) + nearestGroup.Center.GetDistance(enemyCp) + ShootingDistance;
            var isFarFromEnemy = currentDistanceToEnemyCenter > requiredDistToEnemyCenter;

            Point endPoint;
            double speed;

            if (hasAdvantege)
            {
                var isAdvantageStaticEnemy = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                                    _currentMoveEnemyPoint[groupId]
                                        .GetDistance(nearestGroup.Center.X, nearestGroup.Center.Y) <=
                                    MoveEnemyPointTolerance;
                _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

                if (isAdvantageStaticEnemy) return;

                if (_selectedGroupId != groupId)
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupId;
                    });
                    _selectedGroupId = groupId;
                }

                var x = nearestGroup.Center.X - centerPoint.X;
                var y = nearestGroup.Center.Y - centerPoint.Y;

                endPoint = new Point(nearestGroup.Center.X, nearestGroup.Center.Y);
                speed = GetGroupLineMaxSpeed(vehicles, endPoint);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = x;
                    move.Y = y;
                    move.MaxSpeed = speed;
                    _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                    _currentMoveEnemyPoint[groupId] = new Point(nearestGroup.Center.X, nearestGroup.Center.Y);
                });

                return;
            }




            if (groupId == 2)
            {
                var enemyTargetRadiusPoint = GetEnemyTargetRadiusPoint(vehicles);
                if (enemyTargetRadiusPoint != null)
                {
                    var currVector = new Vector(nearestGroup.Center, enemyCp);
                    var newVector = new Vector(nearestGroup.Center, enemyTargetRadiusPoint);

                    var rotationAngle = MathHelper.GetAnlge(currVector, newVector);
                    Rotate90(vehicles, groupId, rotationAngle, nearestGroup.Center);
                    return;
                }
            }

            double targetX;
            double targetY;

            if (isFarFromEnemy)
            {
                targetX = nearestGroup.Center.X;
                targetY = nearestGroup.Center.Y;
            }
            else
            {
                if (groupId == 2)
                {
                    if (IsBalancedRectange(enemyRectangle, nearestGroup.Center))
                    {
                        var group1Vehicles = GetVehicles(1, Ownership.ALLY);
                        if (group1Vehicles.Any())
                        {
                            var group1Vector = new Vector(new Point(nearestGroup.Center),
                                GetVehiclesCenter(group1Vehicles));
                            var group2Vector = new Vector(new Point(nearestGroup.Center), centerPoint);

                            var rotationAngle = MathHelper.GetAnlge(group2Vector, group1Vector);
                            if (Math.Abs(rotationAngle) > Tolerance)
                            {
                                Rotate90(vehicles, groupId, rotationAngle, nearestGroup.Center);
                                return;
                            }
                        }

                        targetX = nearestGroup.Center.X -
                                  requiredDistToEnemyCenter * Math.Cos(_currentGroupAngle[groupId]);
                        targetY = nearestGroup.Center.Y -
                                  requiredDistToEnemyCenter * Math.Sin(_currentGroupAngle[groupId]);

                    }
                    else
                    {
                        if (vehicles.Count >= GoToEnemyCpCount)
                        {
                            targetX = enemyCp.X;
                            targetY = enemyCp.Y;
                        }
                        else
                        {
                            targetX = nearestGroup.Center.X -
                                      requiredDistToEnemyCenter * Math.Cos(_currentGroupAngle[groupId]);
                            targetY = nearestGroup.Center.Y -
                                      requiredDistToEnemyCenter * Math.Sin(_currentGroupAngle[groupId]);
                        }
                    }

                }
                else
                {
                    targetX = nearestGroup.Center.X - requiredDistToEnemyCenter * Math.Cos(_currentGroupAngle[groupId]);
                    targetY = nearestGroup.Center.Y - requiredDistToEnemyCenter * Math.Sin(_currentGroupAngle[groupId]);
                }
            }

            var isFarFromBorder = IsFarFromBorderPoint(vehicles, new Point(targetX, targetY));
            if (!isFarFromBorder)
            {
                targetX = enemyCp.X;
                targetY = enemyCp.Y;
            }


            var isStaticEnemy = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                                _currentMoveEnemyPoint[groupId].GetDistance(targetX, targetY) <=
                                MoveEnemyPointTolerance;

            _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

            if (isStaticEnemy) return; //TODO: возможно, всегда стоит двигатьс€ вблизи врага

            if (_selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            endPoint = new Point(targetX, targetY);
            speed = GetGroupLineMaxSpeed(vehicles, endPoint);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = targetX - centerPoint.X;
                move.Y = targetY - centerPoint.Y;
                move.MaxSpeed = speed;
                _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                _currentMoveEnemyPoint[groupId] = new Point(targetX, targetY);
            });
            

        }



        private Point GetEnemyTargetRadiusPoint(IList<Vehicle> vehicles)
        {

            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);

            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);

            Point enemyTargetRadiusPoint = null;
            var enemyTargetRadiusPointDistToMe = double.MaxValue;

            var currentEnemyCenterDist = nearestGroup.Center.GetDistance(enemyCp);
            for (var i = 0; i < enemyRectangle.Points.Count; ++i)
            {
                var point = enemyRectangle.Points[i];

                var radius = nearestGroup.Center.GetDistance(point);
                var distToMe = point.GetDistance(centerPoint);

                if (currentDistanceToEnemyCenter - radius - centerPoint.GetDistance(myCp) < ShootingDistance &&
                    radius - currentEnemyCenterDist > MoreSideDist &&
                    distToMe < enemyTargetRadiusPointDistToMe)
                {

                    var turnVector = new Vector(new Point(nearestGroup.Center), new Point(point));
                    turnVector.Mult(centerPoint.GetDistance(nearestGroup.Center) / turnVector.Length);

                    var currVector = new Vector(new Point(nearestGroup.Center), new Point(centerPoint));
                    var angle = MathHelper.GetAnlge(currVector, turnVector);
                    var dAnlge = angle / 10d;

                    var isFarFromBorder = true;
                    for (var j = 0; j < 10; ++j)
                    {
                        currVector.Turn(dAnlge);
                        if (!IsFarFromBorderPoint(vehicles, currVector.P2))
                        {
                            isFarFromBorder = false;
                            break;
                        }
                    }

                    if (!isFarFromBorder) continue;

                    enemyTargetRadiusPoint = point;
                    enemyTargetRadiusPointDistToMe = distToMe;
                }
            }

            return enemyTargetRadiusPoint;
        }

        //private void PrepareToRotate90(IList<Vehicle> vehicles, int groupId)
        //{
        //    _sandvichActions[groupId] = SandvichAction.PrepareToRotate90;

        //    var centerPoint = GetVehiclesCenter(vehicles);

        //    if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
        //    {
        //        _delayedMoves.Enqueue(move =>
        //        {
        //            move.Action = ActionType.ClearAndSelect;
        //            move.Group = groupId;
        //        });
        //        _selectedGroupId = groupId;
        //    }

        //    var time = centerPoint.GetDistance(_rotationContainer.PrepareRotationPoint) / GetMaxSpeed(groupId);

        //    _delayedMoves.Enqueue(move =>
        //    {
        //        move.Action = ActionType.Move;
        //        move.X = _rotationContainer.PrepareRotationPoint.X - centerPoint.X;
        //        move.Y = _rotationContainer.PrepareRotationPoint.Y - centerPoint.Y;
        //        move.MaxSpeed = GetMaxSpeed(groupId);
        //        _groupEndMovementTime[groupId] = _world.TickIndex + time;
        //    });
            
        //}

        private void Rotate90(IList<Vehicle> vehicles, int groupId, double rotationAngle, Point rotationCenter)
        {
            _sandvichActions[groupId] = SandvichAction.Rotate90;
            var newAngle = _currentGroupAngle[groupId] + rotationAngle;

            if (newAngle > Math.PI) newAngle -= 2 * Math.PI;
            else if (newAngle < -Math.PI) newAngle += 2 * Math.PI;

            var radius =
                vehicles.Max(
                    v =>
                        v.GetDistanceTo(rotationCenter.X,
                            rotationCenter.Y));
            var speed = GetGroupMaxSpeed(groupId);
            var angularSpeed = speed / radius;

            var turnTime = Math.Abs(rotationAngle) / angularSpeed;

            if (_selectedGroupId != groupId)
            {
                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }
            var currentAngle = _currentGroupAngle[groupId];
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Rotate;
                move.X = rotationCenter.X;
                move.Y = rotationCenter.Y;
                move.Angle = rotationAngle;
                _groupEndMovementTime[groupId] = _world.TickIndex + turnTime;
                move.MaxAngularSpeed = angularSpeed;

                _currentAngularSpeed[groupId] = rotationAngle > 0 ? angularSpeed : -angularSpeed;
                _isRotating[groupId] = true;

                _tmpGroupAngle[groupId] = currentAngle;
                _currentGroupAngle[groupId] = newAngle;
                
            });
        }


        private RotationContainer GetRotationContainer(IList<Vehicle> vehicles, double angle)
        {
            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));

            var vector = new Vector(new Point(nearestGroup.Center), new Point(centerPoint));
            var radius = vector.Length;
            vector.Turn(angle);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(vector.P2, enemyRectangle, vector.P1);
            
            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var myDist = myCp.GetDistance(centerPoint) + ShootingDistance;

            var OCp = new Vector(vector.P1, enemyCp); 
            var coeff1 = (OCp.Length + myDist)/ OCp.Length;
            OCp.Mult(coeff1);
            var afterRotationPoint = new Point(OCp.P2);

            var coeff2 = (OCp.Length - radius)/OCp.Length;
            OCp.Mult(coeff2);
            var rotationCenterPoint = new Point(OCp.P2);

            var vTmp = new Vector(new Point(rotationCenterPoint), new Point(afterRotationPoint));
            vTmp.Turn(-angle);
            var prepareRotationPoint = new Point(vTmp.P2);

            return new RotationContainer()
            {
                AfterRotationPoint = afterRotationPoint,
                PrepareRotationPoint = prepareRotationPoint,
                RotationCenterPoint = rotationCenterPoint,
                RotationAngle =  angle
            };
        }


        private bool HasAdvantage(int groupId, GroupContainer enemyGroup, IList<GroupContainer> enemyGroups,
            double vehiclesCountAdvantage, double vehiclesCoeffAdvantage)
        {
            var vehicles = GetVehicles(groupId, Ownership.ALLY);
            var enemyWeight = enemyGroup.Vehicles.Aggregate(0d, (current, v) => current + v.Durability);
            switch (groupId)
            {
                case 1:
                    var myWeight = vehicles.Aggregate(0d, (current, v) => current + v.Durability);
                    var g2 = GetVehicles(2, Ownership.ALLY);
                    if (g2.Any())
                    {
                        var centerPoint = GetVehiclesCenter(g2);
                        var g2NearestEnemy = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

                        if (centerPoint.GetDistance(g2NearestEnemy.Center) < CloseFriendDistance &&
                            g2NearestEnemy.Center.GetDistance(enemyGroup.Center) < Tolerance)
                        {
                            myWeight = g2.Aggregate(myWeight, (current, v) => current + v.Durability);
                        }
                    }

                    return myWeight - enemyWeight >= _game.TankDurability * vehiclesCountAdvantage ||
                           myWeight * 1d / enemyWeight >= vehiclesCoeffAdvantage;
                case 2:

                    myWeight = vehicles.Aggregate(0d, (current, v) => current + v.Durability);

                    var g1 = GetVehicles(1, Ownership.ALLY);
                    var hasCloseG1 = false;
                    if (g1.Any())
                    {
                        var centerPoint = GetVehiclesCenter(g1);
                        var g1NearestEnemy = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

                        if (centerPoint.GetDistance(g1NearestEnemy.Center) < CloseFriendDistance &&
                            g1NearestEnemy.Center.GetDistance(enemyGroup.Center) < Tolerance)
                        {
                            hasCloseG1 = true;
                        }
                    }

                    if (hasCloseG1)
                    {
                        myWeight = g1.Aggregate(myWeight, (current, v) => current + v.Durability);
                        return myWeight - enemyWeight >= _game.TankDurability * vehiclesCountAdvantage ||
                               myWeight * 1d / enemyWeight >= vehiclesCoeffAdvantage;
                    }

                    var otherEnemyWeight = 0d;
                    foreach (var v in enemyGroup.Vehicles)
                    {
                        switch (v.Type)
                        {
                            case VehicleType.Fighter:
                                otherEnemyWeight += 1 * v.Durability;
                                break;
                            case VehicleType.Helicopter:
                                otherEnemyWeight += 1 * v.Durability;
                                break;
                            case VehicleType.Tank:
                                otherEnemyWeight += 1 * v.Durability;
                                break;
                            case VehicleType.Ifv:
                                otherEnemyWeight += 3 * v.Durability;
                                break;
                        }
                    }

                    return myWeight >= otherEnemyWeight;

                default:
                    throw new Exception("Unknown group id");
            }
        }


        private bool MakeNuclearStrike()
        {
            var enemyGroups = GetEnemyVehicleGroups();
            var targetPoint = GetNuclearStrikeEnemyPoint(enemyGroups);
            var myVehicles = GetVehicles(Ownership.ALLY);

            if (targetPoint == null) return false;

            var strikingVehicle = GetNuclearStrikeVehicle(targetPoint, myVehicles);
            _nuclearVehicleId = strikingVehicle.Id;

            var isUncompressing = false;
            var groupId = strikingVehicle.Groups.SingleOrDefault();
            if (groupId != 0)
            {
                if (_sandvichActions[groupId] == SandvichAction.Rotating ||
                    _sandvichActions[groupId] == SandvichAction.Rotate90)
                {
                    _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
                }

                isUncompressing = _sandvichActions[groupId] == SandvichAction.Uncompress;
                var isCompressing = _sandvichActions[groupId] == SandvichAction.Compressing2;

                if (!isUncompressing && !isCompressing)
                {
                    if (_selectedGroupId != groupId)
                    {
                        _importantDelayedMoves.Enqueue(move =>
                        {
                            move.Action = ActionType.ClearAndSelect;
                            move.Group = groupId;
                        });
                        _selectedGroupId = groupId;
                    }

                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = 0;
                        move.Y = 0;
                    });

                    _sandvichActions[groupId] = SandvichAction.NuclearStrike;
                }
            }

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.TacticalNuclearStrike;
                move.X = targetPoint.X;
                move.Y = targetPoint.Y;
                move.VehicleId = strikingVehicle.Id;
                _isMyNuclearStrikeConsidered = false;
                _isFirstNuclearStrike = false;
                if (!isUncompressing)
                    _groupEndMovementTime[groupId] = _world.TickIndex + _game.TacticalNuclearStrikeDelay;
            });
           

            return true;
        }

        private void Uncompress(int groupId)
        {
            if (_sandvichActions[groupId] == SandvichAction.Rotating || _sandvichActions[groupId] == SandvichAction.Rotate90)
            {
                _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
            }

            _sandvichActions[groupId] = SandvichAction.Uncompress;

            if (_selectedGroupId != groupId)
            {
                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            _enemyNuclearStrikeX = _enemy.NextNuclearStrikeX;
            _enemyNuclearStrikeY = _enemy.NextNuclearStrikeY;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = 1/NuclearCompressionFactor;
                move.X = _enemy.NextNuclearStrikeX;
                move.Y = _enemy.NextNuclearStrikeY;
                _groupEndMovementTime[groupId] = _enemy.NextNuclearStrikeTickIndex;
                _groupStartUncompressTick[groupId] = _world.TickIndex;
            });
        }

        private delegate bool IsAStarMoveFinished(int groupId);

        private bool IsGroundAStarMoveFinished(int groupId)
        {
            //TODO: данный тип уже может отсутствовать

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

                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Bottom = _world.Height;
                        move.Right = _world.Width;
                        move.VehicleType = vehicleType;
                    });
                    _selectedGroupId = -1;

                    var destX = (_groundAStarPathes[vehicleType][_groundPathIndexes[vehicleType]] as ASquare).CenterX;
                    var destY = (_groundAStarPathes[vehicleType][_groundPathIndexes[vehicleType]] as ASquare).CenterY;
                    var dist = GetVehiclesCenter(currVehicles).GetDistance(destX, destY);

                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = destX - GetVehiclesCenter(currVehicles).X;
                        move.Y = destY - GetVehiclesCenter(currVehicles).Y;

                        _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                            _world.TickIndex + dist / GetGroupMaxSpeed(groupId));
                    });
                }
            }

            return isFinished;
        }

        private bool IsAirAStarMoveFinished(int groupId)
        {
            //TODO: данный тип уже может отсутствовать

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

                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Bottom = _world.Height;
                        move.Right = _world.Width;
                        move.VehicleType = vehicleType;
                    });
                    _selectedGroupId = -1;

                    var destX = (_airAStarPathes[vehicleType][_airPathIndexes[vehicleType]] as ASquare).CenterX;
                    var destY = (_airAStarPathes[vehicleType][_airPathIndexes[vehicleType]] as ASquare).CenterY;
                    var dist = GetVehiclesCenter(currVehicles).GetDistance(destX, destY);

                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = destX - GetVehiclesCenter(currVehicles).X;
                        move.Y = destY - GetVehiclesCenter(currVehicles).Y;

                        _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId],
                            _world.TickIndex + dist / GetGroupMaxSpeed(groupId));
                    });
                }
            }

            return isFinished;
        }

        private double GetGroupMaxSpeed(int groupId)
        {
            //var vehicles = GetVehicles(groupId, Ownership.ALLY);
            //return vehicles.Select(v => GetActualMaxSpeed(v)).Min();

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

       
        private void SandvichMove(int groupId, IsAStarMoveFinished isAStarMoveFinished, Shift shift, Compress compress)
        {
            var sandvichAction = _sandvichActions[groupId];
            var vehicles = GetVehicles(groupId, Ownership.ALLY);
            if (!vehicles.Any()) return;

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
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        shift(groupId);
                    }
                    break;

                case SandvichAction.Shifting:
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        compress(groupId);
                    }
                    break;

                case SandvichAction.Compressing:
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        var centerPoint = GetVehiclesCenter(vehicles);
                        Compress2(centerPoint.X, centerPoint.Y, PrepareCompressinFactor, 100d, groupId); //TODO
                    }
                    break;

                case SandvichAction.Compressing2:
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }
                    break;
                case SandvichAction.Rotating:
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        _isRotating[groupId] = false;

                        DoMilitaryAction(vehicles, groupId);
                    }
                    break;
                case SandvichAction.MovingToEnemy:
                {
                    if (_world.TickIndex > _groupEndMovementTime[groupId])
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }

                    break;
                }
                case SandvichAction.Uncompress:
                    if (_world.TickIndex > _groupEndMovementTime[groupId]) //TODO: все сто€т?
                    {
                        Compress2(_enemyNuclearStrikeX,
                            _enemyNuclearStrikeY,
                            NuclearCompressionFactor,
                            _world.TickIndex - _groupStartUncompressTick[groupId],
                            groupId);
                    }
                    break;
                case SandvichAction.Rotate90:
                    var enemyTargetRadiusPoint = GetEnemyTargetRadiusPoint(vehicles);
                    if (enemyTargetRadiusPoint == null) //ƒостигли точки, с которой можно переть на врага
                    {
                        _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
                        _isRotating[groupId] = false;
                        MoveToEnemy(vehicles, groupId);
                    }

                    else if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        _isRotating[groupId] = false;
                        MoveToEnemy(vehicles, groupId);
                    }
                    break;

                case SandvichAction.NuclearStrike:
                    if (_world.TickIndex > _groupEndMovementTime[groupId])
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }
                    break;

            }
        }

        private void DoMilitaryAction(IList<Vehicle> vehicles, int groupId)
        {
            var centerPoint = GetVehiclesCenter(vehicles);
            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);
            var angle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, nearestGroup.Center));

            var isSmallEnemyGroup = HasAdvantage(groupId,
                                        nearestGroup,
                                        enemyGroups,
                                        StrongVehiclesCountAdvantage,
                                        StrongVehiclesCoeffAdvantage) && nearestGroup.Vehicles.Count <
                                    SmallGroupVehiclesCount;
            var isFarFromBorder = IsFarFromBorderPoint(vehicles, GetVehiclesCenter(vehicles));

            var nearestFacility = GetNearestFacility(centerPoint);

            var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);
            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);
            var requiredDistToEnemyCenter = centerPoint.GetDistance(myCp) + nearestGroup.Center.GetDistance(enemyCp) + ShootingDistance;
            var isFarFromEnemy = currentDistanceToEnemyCenter > requiredDistToEnemyCenter;

            var isFacilityCloser = nearestFacility != null &&
                centerPoint.GetDistance(nearestFacility.Left + _game.FacilityWidth / 2d,
                    nearestFacility.Top + _game.FacilityHeight / 2d) < centerPoint.GetDistance(nearestGroup.Center);

            var hasGroupVehicle = vehicles.Any(v => v.Type != VehicleType.Helicopter && v.Type != VehicleType.Fighter);

            if (hasGroupVehicle && isFacilityCloser && isFarFromEnemy)
            {
                MoveToFacility(vehicles, groupId, nearestFacility);
            }
            else if (!isSmallEnemyGroup && Math.Abs(_currentGroupAngle[groupId] - angle) > MaxAngle && isFarFromBorder)
            {
                RotateToEnemy(vehicles, groupId);
            }
            else
            {
                MoveToEnemy(vehicles, groupId);
            }
        }

        private bool IsWeakAviationGroup(IList<Vehicle> vehicles)
        {
            return vehicles.Count(v => v.Type == VehicleType.Helicopter) < WeakAviationHelicoptersCount;
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

        private GroupContainer GetWeakestEnemyGroup(IList<GroupContainer> enemyGroups, IList<Vehicle> myVehicles)
        {
            var dA = myVehicles.Count(v => v.Type == VehicleType.Fighter) * 2 +
                     myVehicles.Count(v => v.Type == VehicleType.Helicopter);
            var dT = myVehicles.Count(v => v.Type == VehicleType.Helicopter);

            var maxValue = -double.MaxValue;
            GroupContainer maxValueGc = null;
            foreach (var group in enemyGroups)
            {
                var nA = group.Vehicles.Count(v => v.Type == VehicleType.Fighter || v.Type == VehicleType.Helicopter);
                var nT = group.Vehicles.Count(v =>
                    v.Type == VehicleType.Tank || v.Type == VehicleType.Ifv || v.Type == VehicleType.Arrv);
                var mA = Math.Min(dA, nA);
                var mT = Math.Min(dT, nT);

                var dE = group.Vehicles.Count(v => v.Type == VehicleType.Fighter) * 2 +
                         group.Vehicles.Count(v => v.Type == VehicleType.Helicopter) +
                         group.Vehicles.Count(v => v.Type == VehicleType.Tank) +
                         group.Vehicles.Count(v => v.Type == VehicleType.Ifv) * 3;


                var delta = mA + mT - dE;
                if (delta < maxValue)
                {
                    maxValue = delta;
                    maxValueGc = group;
                }
            }
            maxValueGc.DeltaDamage = maxValue;
            return maxValueGc;
        }

        private GroupContainer GetNearestEnemyGroup(IList<GroupContainer> enemyGroups, double centerX, double centerY)
        {
            var hasBigGroups = enemyGroups.Any(g => g.Vehicles.Count >= ConsiderGroupVehiclesCount);

            GroupContainer nearestGroup = null;
            var minDist = double.MaxValue;
            foreach (var eg in enemyGroups)
            {
                if (hasBigGroups && eg.Vehicles.Count < ConsiderGroupVehiclesCount) continue;

                var dist = eg.Center.GetDistance(centerX, centerY);
                if (dist < minDist)
                {
                    minDist = dist;
                    nearestGroup = eg;
                }
            }

            return nearestGroup;
        }

        private Facility GetNearestFacility(Point groupCenter)
        {
            var minDist = double.MaxValue;
            Facility targetFacility = null;
            foreach (var facility in _world.Facilities.Where(f => f.CapturePoints < _game.MaxFacilityCapturePoints))
            {
                var dist = groupCenter.GetDistance(facility.Left + _game.FacilityWidth / 2d,
                    facility.Top + _game.FacilityHeight / 2d);
                if (dist < minDist)
                {
                    minDist = dist;
                    targetFacility = facility;
                }
            }
            return targetFacility;
        }

        private double GetGroupNuclearDamage(IList<Vehicle> vehicles, double nuclearX, double nuclearY)
        {
            var sumDamage = 0d;

            foreach (var v in vehicles)
            {
                var dist = v.GetDistanceTo(nuclearX, nuclearY);
                if (dist < _game.TacticalNuclearStrikeRadius)
                {
                    var closeCoeff = (_game.TacticalNuclearStrikeRadius - dist) / _game.TacticalNuclearStrikeRadius;
                    var damage = _game.MaxTacticalNuclearStrikeDamage * closeCoeff;

                    if (damage > v.Durability)
                    {
                        damage = v.Durability;
                    }
                    sumDamage += damage;
                }
            }
            return sumDamage;
        }

        private Point GetNuclearStrikeEnemyPoint(IList<GroupContainer> enemyGroups)
        {
            Point targetPoint = null;
            var maxDiffDamage = 0d;
            var myVehicles = GetVehicles(Ownership.ALLY);

            if (enemyGroups.All(g =>
                myVehicles.All(mv => mv.GetDistanceTo(g.Center.X, g.Center.Y) > GetActualVisualRange(mv))))
                return null;

            var enemyVehicles = GetVehicles(Ownership.ENEMY);

            if (_isFirstNuclearStrike)
            {
                foreach (var group in enemyGroups)
                {
                    if (group.Vehicles.Count < MinNuclearStrikeCount) continue;

                    var strikingVehicle = GetNuclearStrikeVehicle(group.Center, myVehicles);

                    if (strikingVehicle == null) continue;

                    var damage = GetGroupNuclearDamage(enemyVehicles, group.Center.X, group.Center.Y);
                    var myVehiclesDamage = GetGroupNuclearDamage(myVehicles, group.Center.X, group.Center.Y);
                    var diffDamage = damage - myVehiclesDamage;

                    if (diffDamage > maxDiffDamage)
                    {
                        maxDiffDamage = diffDamage;
                        targetPoint = group.Center;
                    }
                }
                return targetPoint;
            }
           
           

            var isOkToStrikeCenterGroups = new Dictionary<GroupContainer, bool>();
            foreach (var v in enemyVehicles)
            {
                var group = enemyGroups.Single(g => g.Vehicles.Any(gv => gv.Id == v.Id));
                
                if (group.Vehicles.Count < MinNuclearStrikeCount) continue;
                //≈сли все мои далеко от центра группы, стрел€ть еще рано
                if (!isOkToStrikeCenterGroups.ContainsKey(group))
                {
                    isOkToStrikeCenterGroups.Add(group, GetNuclearStrikeVehicle(group.Center, myVehicles) != null);
                }
                if (!isOkToStrikeCenterGroups[group])
                    continue;

                var strikingVehicle = GetNuclearStrikeVehicle(new Point(v.X, v.Y), myVehicles);

                if (strikingVehicle == null) continue;

                var damage = GetGroupNuclearDamage(enemyVehicles, v.X, v.Y);
                var myVehiclesDamage = GetGroupNuclearDamage(myVehicles, v.X, v.Y);
                var diffDamage = damage - myVehiclesDamage;

                if (diffDamage > maxDiffDamage)
                {
                    maxDiffDamage = diffDamage;
                    targetPoint = new Point(v.X, v.Y);
                }
            }

            return targetPoint;
        }
       

        private Vehicle GetNuclearStrikeVehicle(Point nuclearStrikePoint, IList<Vehicle> myVehicles)
        {

            int runAwayTime;
            if (_enemy.NextNuclearStrikeTickIndex > -1)
            {
                runAwayTime = _enemy.NextNuclearStrikeTickIndex - _world.TickIndex;
            }
            else
            {
                runAwayTime = _enemy.RemainingNuclearStrikeCooldownTicks >= _game.TacticalNuclearStrikeDelay
                    ? 0
                    : _game.TacticalNuclearStrikeDelay - _enemy.RemainingNuclearStrikeCooldownTicks;
            }

           

            var group1 = GetVehicles(1, Ownership.ALLY);
            Rectangle group1Rectangle = null;
            if (group1.Any())
            {
                group1Rectangle = MathHelper.GetJarvisRectangle(group1.Select(v => new Point(v.X, v.Y)).ToList());
            }

            var group2 = GetVehicles(2, Ownership.ALLY);
            Rectangle group2Rectangle = null;
            if (group2.Any())
            {
                group2Rectangle = MathHelper.GetJarvisRectangle(group2.Select(v => new Point(v.X, v.Y)).ToList());
            }


            var orderedVehicles = myVehicles
                .Where(mv =>
                {
                    if (!mv.Groups.Any()) return true;
                    var group = mv.Groups.Contains(1) ? group1 : group2;
                    if (group.Count < SmallGroupVehiclesCount) return true;

                    var groupRectangle = mv.Groups.Contains(1) ? group1Rectangle : group2Rectangle;
                    return !IsCloseToGroupBorder(new Point(mv.X, mv.Y), groupRectangle, nuclearStrikePoint);
                })
                .OrderByDescending(v => v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y))
                .ToList();

            var vehicle = orderedVehicles.FirstOrDefault(v => 
                //!IsCloseToGroupBorder(new Point(v.X, v.Y), v.Groups.Contains(1) ? group1Rectangle : group2Rectangle, nuclearStrikePoint) &&
                v.Durability >= v.MaxDurability * HpNuclerStrikeCoeff && GetActualVisualRange(v) >=
                v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                GetActualMaxSpeed(v, (int) Math.Truncate(v.X / 32d), (int) Math.Truncate(v.Y / 32d)) * runAwayTime &&
                (runAwayTime == 0 || !HasWorseNearSquares(v)));

            if (vehicle != null) return vehicle;

            vehicle = orderedVehicles.FirstOrDefault(v =>
                //!IsCloseToGroupBorder(new Point(v.X, v.Y), v.Groups.Contains(1) ? group1Rectangle : group2Rectangle, nuclearStrikePoint) &&
                GetActualVisualRange(v) >= v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                GetActualMaxSpeed(v, (int) Math.Truncate(v.X / 32d), (int) Math.Truncate(v.Y / 32d)) * runAwayTime &&
                (runAwayTime == 0 || !HasWorseNearSquares(v)));

            if (vehicle != null) return vehicle;

            vehicle = orderedVehicles.FirstOrDefault(v =>
                //!IsCloseToGroupBorder(new Point(v.X, v.Y), v.Groups.Contains(1) ? group1Rectangle : group2Rectangle, nuclearStrikePoint) &&
                GetActualVisualRange(v) >= v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                GetActualMaxSpeed(v, (int)Math.Truncate(v.X / 32d), (int)Math.Truncate(v.Y / 32d)) * runAwayTime);

            return vehicle;
        }

        private bool IsCloseToGroupBorder(Point vehiclePoint, Rectangle vehicleRectangle, Point sourcePoint)
        {
            var cp = MathHelper.GetNearestRectangleCrossPoint(sourcePoint,
                vehicleRectangle,
                vehiclePoint);
            return cp == null || vehiclePoint.GetDistance(cp) <= CloseToRectangleBorderDist;
        }

        private bool HasWorseNearSquares(Vehicle vehicle)
        {
            var x = MathHelper.GetSquareIndex(vehicle.X);
            var y = MathHelper.GetSquareIndex(vehicle.Y);

            switch (vehicle.Type)
            {
                case VehicleType.Helicopter:
                case VehicleType.Fighter:
                    var visionFactor = GetVisionFactor(_weatherTypeByCellXY[x][y]);
                    if (x > 0 && GetVisionFactor(_weatherTypeByCellXY[x - 1][y]) < visionFactor) return true;
                    if (y > 0 && GetVisionFactor(_weatherTypeByCellXY[x][y - 1]) < visionFactor) return true;
                    if (x > 0 && y > 0 && GetVisionFactor(_weatherTypeByCellXY[x - 1][y - 1]) < visionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && GetVisionFactor(_weatherTypeByCellXY[x + 1][y]) < visionFactor) return true;
                    if (y < _game.TerrainWeatherMapRowCount - 1 && GetVisionFactor(_weatherTypeByCellXY[x][y + 1]) < visionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && y < _game.TerrainWeatherMapRowCount - 1 &&
                        GetVisionFactor(_weatherTypeByCellXY[x + 1][y + 1]) < visionFactor) return true;

                    if (x > 0 && y < _game.TerrainWeatherMapRowCount - 1 && GetVisionFactor(_weatherTypeByCellXY[x - 1][y + 1]) < visionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && y > 0 && GetVisionFactor(_weatherTypeByCellXY[x + 1][y - 1]) < visionFactor) return true;
                    break;

                case VehicleType.Arrv:
                case VehicleType.Ifv:
                case VehicleType.Tank:
                    var terrVisionFactor = GetVisionFactor(_terrainTypeByCellXY[x][y]);
                    if (x > 0 && GetVisionFactor(_terrainTypeByCellXY[x - 1][y]) < terrVisionFactor) return true;
                    if (y > 0 && GetVisionFactor(_terrainTypeByCellXY[x][y - 1]) < terrVisionFactor) return true;
                    if (x > 0 && y > 0 && GetVisionFactor(_terrainTypeByCellXY[x - 1][y - 1]) < terrVisionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && GetVisionFactor(_terrainTypeByCellXY[x + 1][y]) < terrVisionFactor) return true;
                    if (y < _game.TerrainWeatherMapRowCount - 1 && GetVisionFactor(_terrainTypeByCellXY[x][y + 1]) < terrVisionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && y < _game.TerrainWeatherMapRowCount - 1 &&
                        GetVisionFactor(_terrainTypeByCellXY[x + 1][y + 1]) < terrVisionFactor) return true;

                    if (x > 0 && y < _game.TerrainWeatherMapRowCount - 1 && GetVisionFactor(_terrainTypeByCellXY[x - 1][y + 1]) < terrVisionFactor) return true;
                    if (x < _game.TerrainWeatherMapColumnCount - 1 && y > 0 && GetVisionFactor(_terrainTypeByCellXY[x + 1][y - 1]) < terrVisionFactor) return true;

                    break;
            }
            return false;
        }

        private double GetVisionFactor(TerrainType terrainType)
        {
            switch (terrainType)
            {
                case TerrainType.Forest:
                    return _game.ForestTerrainVisionFactor;
                case TerrainType.Plain:
                    return _game.PlainTerrainVisionFactor;
                case TerrainType.Swamp:
                    return _game.SwampTerrainVisionFactor;
                default:
                    throw new Exception("unknown type");
            }
        }

        private double GetVisionFactor(WeatherType weatherType)
        {
            switch (weatherType)
            {
                case WeatherType.Clear:
                    return _game.ClearWeatherVisionFactor;
                case WeatherType.Cloud:
                    return _game.CloudWeatherVisionFactor;
                case WeatherType.Rain:
                    return _game.RainWeatherVisionFactor;
                default:
                    throw new Exception("unknown type");
            }
        }

        private bool IsFarFromBorderPoint(IList<Vehicle> vehicles, Point point)
        {
            var radius = GetSandvichRadius(vehicles);
            var isFarFromBorder = point.X >= radius && point.Y >= radius &&
                                  point.X <= _world.Width - radius &&
                                  point.Y <= _world.Height - radius;
            return isFarFromBorder;
        }

        private double GetSandvichRadius(IList<Vehicle> vehicles)
        {
            var rect = MathHelper.GetJarvisRectangle(vehicles.Select(v => new Point(v.X, v.Y)).ToList());
            return rect.Points.Max(p => p.GetDistance(GetVehiclesCenter(vehicles)));
        }


        private double GetActualVisualRange(Vehicle vehicle)
        {
            var visualRange = vehicle.VisionRange;
            var x = (int) Math.Truncate(vehicle.X / 32d);
            var y = (int) Math.Truncate(vehicle.Y / 32d);

            switch (vehicle.Type)
            {
                case VehicleType.Fighter:
                case VehicleType.Helicopter:
                    var weatherType = _weatherTypeByCellXY[x][y];
                    switch (weatherType)
                    {
                        case WeatherType.Clear:
                            visualRange *= _game.ClearWeatherVisionFactor;
                            break;
                        case WeatherType.Cloud:
                            visualRange *= _game.CloudWeatherVisionFactor;
                            break;
                        case WeatherType.Rain:
                            visualRange *= _game.RainWeatherVisionFactor;
                            break;
                    }
                    break;

                case VehicleType.Arrv:
                case VehicleType.Ifv:
                case VehicleType.Tank:
                    var terrainType = _terrainTypeByCellXY[x][y];
                    switch (terrainType)
                    {
                        case TerrainType.Plain:
                            visualRange *= _game.PlainTerrainVisionFactor;
                            break;
                        case TerrainType.Forest:
                            visualRange *= _game.ForestTerrainVisionFactor;
                            break;
                        case TerrainType.Swamp:
                            visualRange *= _game.SwampTerrainVisionFactor;
                            break;
                    }
                    break;
            }
            return visualRange;
        }

        private double GetGroupLineMaxSpeed(IList<Vehicle> vehicles, Point endPoint)
        {
            var startPoint = GetVehiclesCenter(vehicles);

            var dx = endPoint.X - startPoint.X;
            var dy = endPoint.Y - startPoint.Y;

            var minSpeed = double.MaxValue;
            foreach (var v in vehicles)
            {
                var currStartPoint = new Point(v.X, v.Y);
                var currEndPoint = new Point(v.X + dx, v.Y + dy);
                var line = MathHelper.GetLineSquares(currStartPoint, currEndPoint);

                foreach (var sqaure in line.Where(sq => sq.Item1 >= 0 && sq.Item2 >= 0 &&
                    sq.Item1 < _game.TerrainWeatherMapColumnCount && sq.Item2 < _game.TerrainWeatherMapRowCount))
                {
                    var speed = GetActualMaxSpeed(v, sqaure.Item1, sqaure.Item2);
                    if (speed < minSpeed) minSpeed = speed;
                }
            }
            return minSpeed;
        }

        private double GetActualMaxSpeed(Vehicle vehicle, int x, int y)
        {
            var maxSpeed = vehicle.MaxSpeed;

            switch (vehicle.Type)
            {
                case VehicleType.Fighter:
                case VehicleType.Helicopter:
                    var weatherType = _weatherTypeByCellXY[x][y];
                    switch (weatherType)
                    {
                        case WeatherType.Clear:
                            maxSpeed *= _game.ClearWeatherSpeedFactor;
                            break;
                        case WeatherType.Cloud:
                            maxSpeed *= _game.CloudWeatherSpeedFactor;
                            break;
                        case WeatherType.Rain:
                            maxSpeed *= _game.RainWeatherSpeedFactor;
                            break;
                    }
                    break;

                case VehicleType.Arrv:
                case VehicleType.Ifv:
                case VehicleType.Tank:
                    var terrainType = _terrainTypeByCellXY[x][y];
                    switch (terrainType)
                    {
                        case TerrainType.Plain:
                            maxSpeed *= _game.PlainTerrainSpeedFactor;
                            break;
                        case TerrainType.Forest:
                            maxSpeed *= _game.ForestTerrainSpeedFactor;
                            break;
                        case TerrainType.Swamp:
                            maxSpeed *= _game.SwampTerrainSpeedFactor;
                            break;
                    }
                    break;
            }
            return maxSpeed;
        }



        /// <summary>
        ///     »нциализируем стратегию.
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
                //_groundPotentialField = new double[game.TerrainWeatherMapColumnCount][];
                //for (var i = 0; i < game.TerrainWeatherMapColumnCount; ++i)
                //{
                //    _groundPotentialField[i] = new double[game.TerrainWeatherMapColumnCount];
                //}
            }
        }

        private void InitFicilitiesField()
        {
            var notMyFacilities = _world.Facilities.Where(f => f.CapturePoints < _game.FacilityCaptureScore);
            foreach (var f in notMyFacilities)
            {
                var fPoint = GetFacilityCenterPoint(f);
                for (var i = 0; i < _game.TerrainWeatherMapColumnCount; ++i)
                {
                    for (var j = 0; j < _game.TerrainWeatherMapColumnCount; ++j)
                    {
                        var mapPoint = new Point(CellSize * i + CellSize / 2, CellSize * j + CellSize/2);
                        var dist = mapPoint.GetDistance(fPoint);

                    }
                }
            }
        }

        /// <summary>
        ///     —охран€ем все входные данные в пол€х класса дл€ упрощени€ доступа к ним, а также актуализируем сведени€ о каждой
        ///     технике и времени последнего изменени€ еЄ состо€ни€.
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

            if (_enemy.NextNuclearStrikeTickIndex == -1) _isEnemyNuclearStrikeConsidered = false;
            for (var i = 1; i <= 2; ++i)
            {
                if (_isRotating[i] && _world.TickIndex < _groupEndMovementTime[i])
                {
                    _tmpGroupAngle[i] += _currentAngularSpeed[i];
                }
            }

        }

        /// <summary>
        ///     ƒостаЄм отложенное действие из очереди и выполн€ем его.
        /// </summary>
        /// <returns>¬озвращает true, если и только если отложенное действие было найдено и выполнено.</returns>
        private bool ExecuteDelayedMove()
        {
            if (!_importantDelayedMoves.Any())
            {
                if (!_delayedMoves.Any()) return false;

                var delayedMove = _delayedMoves.Dequeue();
                delayedMove.Invoke(_move);

                return true;
            }
            else
            {
                var delayedMove = _importantDelayedMoves.Dequeue();
                delayedMove.Invoke(_move);

                return true;
            }
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
            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Fighter;
            });
            _selectedGroupId = 2;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Helicopter;
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = 2;
            });
        }

        private void SetGroudGroups()
        {
            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Arrv;
            });
            _selectedGroupId = 1;

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Ifv;
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.AddToSelection;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Tank;
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = 1;
            });
        }

        public Point GetVehiclesCenter(IList<Vehicle> vehicles)
        {
            return new Point(vehicles.Select(v => v.X).Average(), vehicles.Select(v => v.Y).Average());
        }


        private bool IsBalancedRectange(Rectangle rectangle, Point rectCenter)
        {
            var rectPointDistances = rectangle.Points.Select(p => p.GetDistance(rectCenter));
            var minRectPointDistance = rectPointDistances.Min();
            var maxRectPointDistance = rectPointDistances.Max();

            if (maxRectPointDistance - minRectPointDistance > MoreSideDist) return false;

            var midPoints = new List<Point>();
            for (var i = 0; i < rectangle.Points.Count; ++i)
            {
                var p0 = rectangle.Points[i];
                var p1 = rectangle.Points[i < rectangle.Points.Count - 1 ? i + 1 : 0];
                var midPoint = new Point((p0.X + p1.X)/2d, (p0.Y + p1.Y) / 2d);
                midPoints.Add(midPoint);
            }

            var midRectPointDistances = midPoints.Select(p => p.GetDistance(rectCenter));
            var minMidRectPointDistance = midRectPointDistances.Min();
            var maxMidRectPointDistance = midRectPointDistances.Max();

            if (maxMidRectPointDistance - minMidRectPointDistance > MoreSideDist) return false;
            return true;
        }

        /// <summary>
        ///     ¬спомогательный метод, позвол€ющий дл€ указанного типа техники получить другой тип техники, такой, что первый
        ///     наиболее эффективен против второго.
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

        private IList<Point> GetVehicleGroupPoints(IList<Vehicle> vehicles)
        {
            return vehicles.Select(v => new Point(v.X, v.Y)).ToList();
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
            Uncompress,
            Rotate90
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