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

        static MyStrategy()
        {
            Debug.connect("localhost", 13579);
        }

        private const double Tolerance = 1E-3;
        private const int MoveToEnemyTicks = 6;

        private const double FarBorderDistance = 100;
        private const double ShootingDistance = 15d;
        private const double CloseFriendDistance = 100d;

        private const int VehiclesCountAdvantage = 50;
        private const double VehiclesCoeffAdvantage = 1.5;

        private const double NuclearStrikeDistDelta = 20d;

        private const double GroupMaxRadius = 15;

        private const double MaxAngle = Math.PI/180*2;

        private const double NuclearCompressionFactor = 0.1d;

        private const double SquardDelta = 6;
        private const double PrepareCompressinFactor = 0.75;
        private const double MoveEnemyPointTolerance = 10d;

        private const int MinNuclearStrikeCount = 5;

        private const int SmallGroupVehiclesCount = 15;

        private const double MoreSideDist = 20d;

        private const double HpNuclerStrikeCoeff = 0.7;


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

        private RotationContainer _rotationContainer;

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

                var isCompressed = _isGroupCompressed.Values.All(x => x);

                if (!_isMyNuclearStrikeConsidered && isCompressed && _me.RemainingNuclearStrikeCooldownTicks == 0 && MakeNuclearStrike())
                {
                    _delayedMoves.Clear();
                    _isMyNuclearStrikeConsidered = true;
                }
                else if (!_isEnemyNuclearStrikeConsidered && isCompressed && _enemy.NextNuclearStrikeTickIndex > -1)
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

                if (_selectedGroupId == 1 || _selectedGroupId == -1)
                {
                    SandvichMove(1, IsGroundAStarMoveFinished, GroundShift, GroundCompress);
                    if (ExecuteDelayedMove()) return;
                    SandvichMove(2, IsAirAStarMoveFinished, AirShift, AirCompress);
                }
                else if (_selectedGroupId == 2)
                {
                    SandvichMove(2, IsAirAStarMoveFinished, AirShift, AirCompress);
                    if (ExecuteDelayedMove()) return;
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
                    _world.TickIndex + dist/ GetGroupMaxSpeed(1));
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
                    _world.TickIndex + dist / GetGroupMaxSpeed(2));
            });
        }

        private void Scale(IList<Vehicle> vehicles, int groupId)
        {
            //костыль, чтобы потом удобно было выдел€ть
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Group = groupId;
            });
            _delayedMoves.Enqueue(move =>
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

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - Tolerance;
                    move.Bottom = y + Tolerance;
                    move.Left = vehicles.Min(v => v.X);
                    move.Right = vehicles.Max(v => v.X);
                });


                var moveY = SquardDelta*(5 - i);
                if (groupId == 1) moveY *= 2; //дл€ наземной техники

                _delayedMoves.Enqueue(move =>
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
                        _world.TickIndex + moveY/ GetGroupMaxSpeed(groupId));
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
                    _world.TickIndex + SquardDelta/ GetGroupMaxSpeed(groupId));
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
                    _world.TickIndex + SquardDelta/GetGroupMaxSpeed(groupId));
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
                    _world.TickIndex + AStar.SquareSize/ GetGroupMaxSpeed(groupId));
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
                    _world.TickIndex + AStar.SquareSize/ GetGroupMaxSpeed(groupId));
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

        private void Compress2(double x, double y, double compressionCoeff, double time, int groupId, Queue<Action<Move>> actions )
        {
            _sandvichActions[groupId] = SandvichAction.Compressing2;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            {
                actions.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = groupId;
                });
                _selectedGroupId = groupId;
            }

            actions.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = compressionCoeff;
                move.X = x;
                move.Y = y;
                _groupEndMovementTime[groupId] = Math.Max(_groupEndMovementTime[groupId], _world.TickIndex + time);
                _isGroupCompressed[groupId] = true;
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

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
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
            var vehicles = GetVehicles(groupId);
            if (!vehicles.Any()) return;
            var center = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, center.X, center.Y);

            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            var myrectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));

            foreach (var p in enemyRectangle.Points)
            {
                Debug.circleFill(p.X, p.Y, 2, 150);
            }

            foreach (var p in myrectangle.Points)
            {
                Debug.circleFill(p.X, p.Y, 2, 0x00FFFF);
            }

            var centerPoint = GetVehiclesCenter(vehicles);
            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));

            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);

            Debug.circleFill(myCp.X, myCp.Y, 4, 0x00FF00);
            Debug.circleFill(enemyCp.X, enemyCp.Y, 4, 0xFF0000);

            if (_vehicleById.ContainsKey(_nuclearVehicleId) && _me.NextNuclearStrikeTickIndex > -1)
            {
                var nuclearVehilce = _vehicleById[_nuclearVehicleId];
                Debug.circleFill(nuclearVehilce.X, nuclearVehilce.Y, 2, 0x00000);
            }


            if (_sandvichActions[2] == SandvichAction.Rotate90)
            {
                //Debug.line(center.X, center.Y, _rotationContainer.PrepareRotationPoint.X, _rotationContainer.PrepareRotationPoint.Y, 0x0000FF);

                //var radius =
                //    _rotationContainer.RotationCenterPoint.GetDistance(_rotationContainer.PrepareRotationPoint);
                //Debug.circle(_rotationContainer.RotationCenterPoint.X, _rotationContainer.RotationCenterPoint.Y, radius, 0x0000FF);

                Debug.circle(_rotationContainer.AfterRotationPoint.X, _rotationContainer.AfterRotationPoint.Y, 3d, 0x000505);
            }

        }

        private void MoveToEnemy(IList<Vehicle> vehicles, int groupId)
        {

            

            var centerPoint = GetVehiclesCenter(vehicles);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var isFarFromBorder = centerPoint.X >= FarBorderDistance && centerPoint.Y >= FarBorderDistance &&
                                  centerPoint.X <= _world.Width - FarBorderDistance &&
                                  centerPoint.Y <= _world.Height - FarBorderDistance;
            var hasAdvantege = HasAdvantage(groupId, nearestGroup, enemyGroups);
            var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);

            var myRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
            var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
            var myCp = MathHelper.GetNearestRectangleCrossPoint(nearestGroup.Center, myRectangle, centerPoint);
            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, nearestGroup.Center);
            var requiredDistToEnemyCenter = centerPoint.GetDistance(myCp) + nearestGroup.Center.GetDistance(enemyCp) + ShootingDistance;
            

            if (hasAdvantege)
            {
                var isStaticEnemy = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                                    _currentMoveEnemyPoint[groupId]
                                        .GetDistance(nearestGroup.Center.X, nearestGroup.Center.Y) <=
                                    MoveEnemyPointTolerance;
                _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

                if (isStaticEnemy) return;

                if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
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

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = x;
                    move.Y = y;
                    move.MaxSpeed = GetGroupMaxSpeed(groupId);
                    _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                    _currentMoveEnemyPoint[groupId] = new Point(nearestGroup.Center.X, nearestGroup.Center.Y);
                });

                return;
            }




            //if (isFarFromEnemy || hasAdvantege)
            //{
            //    var x = nearestGroup.Center.X - centerPoint.X;
            //    var y = nearestGroup.Center.Y - centerPoint.Y;

            //    if (!hasAdvantege && isFarFromBorder)
            //    {
            //        x -= requiredDistToEnemyCenter * Math.Cos(_currentGroupAngle[groupId]);
            //        y -= requiredDistToEnemyCenter * Math.Sin(_currentGroupAngle[groupId]);
            //    }

                

            //    if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
            //    {
            //        _delayedMoves.Enqueue(move =>
            //        {
            //            move.Action = ActionType.ClearAndSelect;
            //            move.Group = groupId;
            //        });
            //        _selectedGroupId = groupId;
            //    }

            //    _delayedMoves.Enqueue(move =>
            //    {
            //        move.Action = ActionType.Move;
            //        move.X = x;
            //        move.Y = y;
            //        move.MaxSpeed = GetMaxSpeed(groupId);
            //        _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
            //        _currentMoveEnemyPoint[groupId] = new Point(x, y);
            //    });

            //    return;
                
            //}
           

            var needRotate90 = false;
            if (groupId == 2)
            {

                //var maxRadiusPoint = GetMaxRadiusEnemyPoint(centerPoint, enemyRectangle, nearestGroup.Center);

                Point enemyTargetRadiusPoint = null;
                var enemyTargetRadiusPointDistToMe = double.MaxValue;

                var currentEnemyCenterDist = nearestGroup.Center.GetDistance(enemyCp);
                for (var i = 0; i < enemyRectangle.Points.Count; ++i)
                {
                    var point = enemyRectangle.Points[i];

                    var isEnemyFarFromBorder = point.X >= FarBorderDistance && point.Y >= FarBorderDistance &&
                                          point.X <= _world.Width - FarBorderDistance &&
                                          point.Y <= _world.Height - FarBorderDistance;

                    if (!isEnemyFarFromBorder) continue;

                    var radius = nearestGroup.Center.GetDistance(point);
                    var distToMe = point.GetDistance(centerPoint);

                    if (currentDistanceToEnemyCenter - radius - centerPoint.GetDistance(myCp) < ShootingDistance &&
                        radius - currentEnemyCenterDist > MoreSideDist &&
                        distToMe < enemyTargetRadiusPointDistToMe)
                    {
                        enemyTargetRadiusPoint = point;
                        enemyTargetRadiusPointDistToMe = distToMe;
                    }
                }

                if (enemyTargetRadiusPoint != null)
                {
                    var currVector = new Vector(nearestGroup.Center, enemyCp);
                    var newVector = new Vector(nearestGroup.Center, enemyTargetRadiusPoint);
                    
                    var angle = MathHelper.GetAnlge(currVector, newVector);
                    needRotate90 = true;
                    _rotationContainer = GetRotationContainer(vehicles, angle);
                    
                }


                //var rotationContainerPlus = GetRotationContainer(vehicles, Math.PI/2);
                //var distancePlus = nearestGroup.Center.GetDistance(rotationContainerPlus.AfterRotationPoint);
                //var rotationContainerMinus = GetRotationContainer(vehicles, -Math.PI/2);
                //var distanceMinus = nearestGroup.Center.GetDistance(rotationContainerMinus.AfterRotationPoint);


                //var isOkPlus = rotationContainerPlus.IsFarFromBroder(_world.Width, _world.Height, FarBorderDistance) &&
                //               distancePlus - requiredDistToEnemyCenter > MoreSideDist;
                //var isOkMinus = rotationContainerMinus.IsFarFromBroder(_world.Width, _world.Height, FarBorderDistance) &&
                //                distanceMinus - requiredDistToEnemyCenter > MoreSideDist;

                //if (isOkPlus && isOkMinus)
                //{
                //    needRotate90 = true;
                //    _rotationContainer = distancePlus > distanceMinus ? rotationContainerPlus : rotationContainerMinus;
                //}
                //else if (isOkPlus)
                //{
                //    needRotate90 = true;
                //    _rotationContainer = rotationContainerPlus;
                //}
                //else if (isOkMinus)
                //{
                //    needRotate90 = true;
                //    _rotationContainer = rotationContainerMinus;
                //}
            }

            if (needRotate90)
            {
                Rotate90(vehicles, groupId);
            }
            else
            {
                var targetX = nearestGroup.Center.X;
                var targetY = nearestGroup.Center.Y;

                if (isFarFromBorder)
                {
                    targetX -= requiredDistToEnemyCenter * Math.Cos(_currentGroupAngle[groupId]);
                    targetY -= requiredDistToEnemyCenter * Math.Sin(_currentGroupAngle[groupId]);
                }

                var isFarFromEnemy = currentDistanceToEnemyCenter > requiredDistToEnemyCenter;
                var isStaticEnemy = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                                    _currentMoveEnemyPoint[groupId].GetDistance(targetX, targetY) <=
                                    MoveEnemyPointTolerance;

                _sandvichActions[groupId] = SandvichAction.MovingToEnemy;

                if (isFarFromEnemy && isStaticEnemy) return;


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
                    move.Action = ActionType.Move;
                    move.X = targetX - centerPoint.X;
                    move.Y = targetY - centerPoint.Y;
                    move.MaxSpeed = GetGroupMaxSpeed(groupId);
                    _groupEndMovementTime[groupId] = _world.TickIndex + MoveToEnemyTicks;
                    _currentMoveEnemyPoint[groupId] = new Point(targetX, targetY);
                });
            }
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

        private void Rotate90(IList<Vehicle> vehicles, int groupId)
        {
            _sandvichActions[groupId] = SandvichAction.Rotate90;

            var centerPoint = GetVehiclesCenter(vehicles);
            
            var turnAngle = _rotationContainer.RotationAngle;
            var newAngle = _currentGroupAngle[groupId] + turnAngle;

            if (newAngle > Math.PI) newAngle -= 2 * Math.PI;
            else if (newAngle < -Math.PI) newAngle += 2 * Math.PI;

            var radius = vehicles.Max(v => v.GetDistanceTo(_rotationContainer.RotationCenterPoint.X, _rotationContainer.RotationCenterPoint.Y));
            var speed = GetGroupMaxSpeed(groupId);
            var angularSpeed = speed / radius;

            var turnTime = Math.Abs(turnAngle) / angularSpeed;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
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
                move.X = _rotationContainer.RotationCenterPoint.X;
                move.Y = _rotationContainer.RotationCenterPoint.Y;
                move.Angle = turnAngle;
                _groupEndMovementTime[groupId] = _world.TickIndex + turnTime;
                move.MaxAngularSpeed = angularSpeed;

                _currentAngularSpeed[groupId] = turnAngle > 0 ? angularSpeed : -angularSpeed;
                _isRotating[groupId] = true;

                _tmpGroupAngle[groupId] = currentAngle;
                _currentGroupAngle[groupId] = newAngle;
                
            });
        }

        private Point GetMaxRadiusEnemyPoint(Point myVehiclesCenter, Rectangle rect, Point rectangleCenter)
        {
            var maxRadius = 0d;
            Point maxRadiusPoint = null;
            for (var i = 0; i < rect.Points.Count; ++i)
            {
                var point = rect.Points[i];

                var isFarFromBorder = point.X >= FarBorderDistance && point.Y >= FarBorderDistance &&
                                      point.X <= _world.Width - FarBorderDistance &&
                                      point.Y <= _world.Height - FarBorderDistance;

                if (!isFarFromBorder) continue;

                var radius = rectangleCenter.GetDistance(point);
               
                if (radius > maxRadius)
                {
                    maxRadius = radius;
                    maxRadiusPoint = point;
                }
            }

            return maxRadiusPoint;

            //var myVector = new Vector(rectangleCenter, myVehiclesCenter); 
            //var newVector = new Vector(rectangleCenter, maxRadiusPoint);
            //return MathHelper.GetAnlge(myVector, newVector);
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

      
        private bool HasAdvantage(int groupId, GroupContainer enemyGroup, IList<GroupContainer> enemyGroups)
        {
            var vehicles = GetVehicles(groupId, Ownership.ALLY);
            switch (groupId)
            {
                case 1:
                    var count = vehicles.Count;
                    var g2 = GetVehicles(2, Ownership.ALLY);
                    if (g2.Any())
                    {
                        var centerPoint = GetVehiclesCenter(g2);
                        var g2NearestEnemy = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

                        if (centerPoint.GetDistance(g2NearestEnemy.Center) < CloseFriendDistance &&
                            g2NearestEnemy.Center.GetDistance(enemyGroup.Center) < Tolerance)
                        {
                            count += g2.Count;
                        }
                    }

                    return count - enemyGroup.Vehicles.Count >= VehiclesCountAdvantage ||
                           count * 1d / enemyGroup.Vehicles.Count >= VehiclesCoeffAdvantage;
                case 2:

                    var myWeight = vehicles.Count;

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
                        myWeight += g1.Count;
                        return myWeight - enemyGroup.Vehicles.Count >= VehiclesCountAdvantage ||
                               myWeight * 1d / enemyGroup.Vehicles.Count >= VehiclesCoeffAdvantage;
                    }

                    
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
       

        private bool MakeNuclearStrike()
        {
            var enemyGroups = GetEnemyVehicleGroups();
            var targetGroup = GetNuclearStrikeEnemyGroup(enemyGroups);

            if (targetGroup == null) return false;
           
            _nuclearVehicleId = targetGroup.NuclearStrikeVehicle.Id;

            var groupId = targetGroup.NuclearStrikeVehicle.Groups.Single();

            if (_sandvichActions[groupId] == SandvichAction.Rotating || _sandvichActions[groupId] == SandvichAction.Rotate90)
            {
                _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
            }

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
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

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.TacticalNuclearStrike;
                move.X = targetGroup.Center.X;
                move.Y = targetGroup.Center.Y;
                move.VehicleId = targetGroup.NuclearStrikeVehicle.Id;
                _isMyNuclearStrikeConsidered = false;
            });

            _sandvichActions[groupId] = SandvichAction.NuclearStrike;
            _groupEndMovementTime[groupId] = _world.TickIndex + _game.TacticalNuclearStrikeDelay;

            return true;
        }

        private void Uncompress(int groupId)
        {
            if (_sandvichActions[groupId] == SandvichAction.Rotating || _sandvichActions[groupId] == SandvichAction.Rotate90)
            {
                _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
            }

            _sandvichActions[groupId] = SandvichAction.Uncompress;

            if (_isGroupCompressed.Keys.Any(key => !_isGroupCompressed[key]) || _selectedGroupId != groupId)
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

        //private Rectangle GetGroupRectangel(IList<Vehicle> vehicles)
        //{
        //    var minX = vehicles.Min(v => v.X);
        //    var minY = vehicles.Min(v => v.Y);

        //    var maxX = vehicles.Max(v => v.X);
        //    var maxY = vehicles.Max(v => v.Y);

        //    var minXVehicles = vehicles.Where(v => Math.Abs(v.X - minX) < Tolerance);
        //    var minYVehicles = vehicles.Where(v => Math.Abs(v.Y - minY) < Tolerance);
        //    var maxXVehicles = vehicles.Where(v => Math.Abs(v.X - maxX) < Tolerance);
        //    var maxYVehicles = vehicles.Where(v => Math.Abs(v.Y - maxY) < Tolerance);

        //    if (minXVehicles.Count() > 1 || minYVehicles.Count() > 1 || maxXVehicles.Count() > 1 ||
        //        maxYVehicles.Count() > 1)
        //        return new Rectangle()
        //        {
        //            Points =
        //                new List<Point>()
        //                {
        //                    new Point(minX, minY),
        //                    new Point(minX, maxY),
        //                    new Point(maxX, maxY),
        //                    new Point(maxX, minY)
        //                }
        //        };

        //    return new Rectangle()
        //    {
        //        Points =
        //            new List<Point>()
        //            {
        //                new Point(minXVehicles.Single().X, minXVehicles.Single().Y),
        //                new Point(minYVehicles.Single().X, minYVehicles.Single().Y),
        //                new Point(maxXVehicles.Single().X, maxXVehicles.Single().Y),
        //                new Point(maxYVehicles.Single().X, maxYVehicles.Single().Y),
        //            }
        //    };
        //}

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
                        Compress2(centerPoint.X, centerPoint.Y, PrepareCompressinFactor, 100d, groupId, _delayedMoves); //TODO
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
                        _isRotating[groupId] = false;

                        var centerPoint = GetVehiclesCenter(vehicles);
                        var enemyGroups = GetEnemyVehicleGroups();
                        var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);
                        var angle = MathHelper.GetAnlge(
                            new Vector(centerPoint,
                                new Point(centerPoint.X + 100, centerPoint.Y)),
                            new Vector(centerPoint, nearestGroup.Center));

                        var isSmallEnemyGroup = IsSmallEnemyGroup(vehicles, nearestGroup.Vehicles);
                        if (!isSmallEnemyGroup && Math.Abs(_currentGroupAngle[groupId] - angle) > MaxAngle)
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            MoveToEnemy(vehicles, groupId);
                        }
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

                    var isSmallEnemyGroup = IsSmallEnemyGroup(vehicles, nearestGroup.Vehicles);

                    //var isGroup1Close = false;
                    //if (groupId == 2 && !_isGroup2Rotated)
                    //{
                    //    var g1Vehilces = GetVehicles(1, Ownership.ALLY);
                    //    if (g1Vehilces.Any() && GetVehiclesCenter(g1Vehilces).GetDistance(nearestGroup.Center) < 700) //TODO: кол-во больше некой константы
                    //    {
                    //        isGroup1Close = true;
                    //    }
                    //}
                    //if (isGroup1Close)
                    //{
                    //    PrepareToRotate90(vehicles, groupId);
                    //}
                    if (!isSmallEnemyGroup && Math.Abs(_currentGroupAngle[groupId] - angle) > MaxAngle &&
                             _world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        RotateToEnemy(vehicles, groupId);
                    }
                    else if (_world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        MoveToEnemy(vehicles, groupId);
                    }

                    break;
                }
                //case SandvichAction.NuclearStrike:
                //    if (_world.TickIndex >= _groupEndMovementTime[groupId]) //TODO: все сто€т?
                //    {
                //        MoveToEnemy(vehicles, groupId);
                //    }
                //    break;
                case SandvichAction.Uncompress:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId]) //TODO: все сто€т?
                    {
                        Compress2(_enemyNuclearStrikeX, _enemyNuclearStrikeY, NuclearCompressionFactor,
                            _game.TacticalNuclearStrikeDelay, groupId, _importantDelayedMoves);
                    }
                    break;
                case SandvichAction.Rotate90:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        _isRotating[groupId] = false;
                        MoveToEnemy(vehicles, groupId);
                    }
                    break;

                case SandvichAction.NuclearStrike:
                    if (_world.TickIndex >= _groupEndMovementTime[groupId])
                    {
                        var centerPoint = GetVehiclesCenter(vehicles);
                        var enemyGroups = GetEnemyVehicleGroups();
                        var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);
                        var angle = MathHelper.GetAnlge(
                            new Vector(centerPoint,
                                new Point(centerPoint.X + 100, centerPoint.Y)),
                            new Vector(centerPoint, nearestGroup.Center));

                        var isSmallEnemyGroup = IsSmallEnemyGroup(vehicles, nearestGroup.Vehicles);
                        if (!isSmallEnemyGroup && Math.Abs(_currentGroupAngle[groupId] - angle) > MaxAngle)
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            MoveToEnemy(vehicles, groupId);
                        }
                    }
                    break;

            }
        }

        private bool IsSmallEnemyGroup(IList<Vehicle> myVehicles, IList<Vehicle> enemyVehicles)
        {
            return enemyVehicles.Count <= SmallGroupVehiclesCount &&
                   (myVehicles.Count - enemyVehicles.Count >= VehiclesCountAdvantage ||
                    myVehicles.Count*1d/enemyVehicles.Count >= VehiclesCoeffAdvantage);
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

        private GroupContainer GetNuclearStrikeEnemyGroup(IList<GroupContainer> enemyGroups)
        {
            var myVehicles = GetVehicles(Ownership.ALLY);

            GroupContainer targetGroup = null;

            var maxDiffDamage = 0d;
            foreach (var eg in enemyGroups.Where(x => x.Vehicles.Count >= MinNuclearStrikeCount))
            {
                var strikingVehicle = GetNuclearStrikeVehcile(eg, myVehicles);

                if (strikingVehicle == null) continue;

                var damage = GetGroupNuclearDamage(eg.Vehicles, eg.Center.X, eg.Center.Y);
                var myVehiclesDamage = GetGroupNuclearDamage(myVehicles, eg.Center.X, eg.Center.Y);
                var diffDamage = damage - myVehiclesDamage;

                if (diffDamage > maxDiffDamage)
                {
                    maxDiffDamage = diffDamage;
                    targetGroup = eg;
                    targetGroup.NuclearStrikeVehicle = strikingVehicle;
                }
            }

            return targetGroup;
        }

        private Vehicle GetNuclearStrikeVehcile(GroupContainer enemyGroup, IList<Vehicle> myVehicles)
        {
            var orderedVehicles = myVehicles.OrderByDescending(v => v.GetDistanceTo(enemyGroup.Center.X, enemyGroup.Center.Y))
                .ToList();

            var vehicle = orderedVehicles.FirstOrDefault(v =>
                v.Durability >= v.MaxDurability * HpNuclerStrikeCoeff && GetActualVisualRange(v) >=
                v.GetDistanceTo(enemyGroup.Center.X, enemyGroup.Center.Y) +
                v.MaxSpeed * _game.TacticalNuclearStrikeDelay);

            if (vehicle != null) return vehicle;

            vehicle = orderedVehicles.FirstOrDefault(v =>
                GetActualVisualRange(v) >= v.GetDistanceTo(enemyGroup.Center.X, enemyGroup.Center.Y) +
                v.MaxSpeed * _game.TacticalNuclearStrikeDelay);

            if (vehicle != null) return vehicle;

            vehicle = orderedVehicles.FirstOrDefault(v =>
                v.Durability >= v.MaxDurability * HpNuclerStrikeCoeff &&
                GetActualVisualRange(v) >= v.GetDistanceTo(enemyGroup.Center.X, enemyGroup.Center.Y) + NuclearStrikeDistDelta);

            if (vehicle != null) return vehicle;

            vehicle = orderedVehicles.FirstOrDefault(v =>
                GetActualVisualRange(v) >= v.GetDistanceTo(enemyGroup.Center.X, enemyGroup.Center.Y) + NuclearStrikeDistDelta);
         
            return vehicle;
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

        private double GetActualMaxSpeed(Vehicle vehicle)
        {
            var maxSpeed = vehicle.MaxSpeed;
            var x = (int)Math.Truncate(vehicle.X / 32d);
            var y = (int)Math.Truncate(vehicle.Y / 32d);

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

                //создаем группы
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