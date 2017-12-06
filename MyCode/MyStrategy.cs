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
        private const double EnemyVehicleDeltaShootingDist = 100d;
        private const double CloseBorderDist = 0d;
        private const double OrbitalWidth = 25d;

        private const int SquardCount = 33;
        private const double NeedCompressionRadius = 7d;
        private const double NeedRotationAdvantage = 0.15;
        private const double MakeNuclearStrikePart = 0.1;
        private const double MakeNuclearStrikeCount = 10;
       
        private IDictionary<int, IList<Vehicle>> _groups = new Dictionary<int, IList<Vehicle>>();
        private int _lastGroupIndex = -1;

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

        private readonly IDictionary<int, double> _currentMovingAngle = new Dictionary<int, double>()
        {
            {1, 0d},
            {2, 0d},
        };

        private readonly IDictionary<int, bool> _isGroupCompressed = new Dictionary<int, bool>()
        {
            {0, false },
            {1, false },
        };

        private int _selectedGroupId = -1;

        private TerrainType[][] _terrainTypeByCellXY;
        private double _tornadoRadius = 100;

        private int _tornadoVehiclesCount = 500;
        private WeatherType[][] _weatherTypeByCellXY;
        private World _world;

        private IList<Vehicle> _enemyVehicles;
        private IList<GroupContainer> _enemyVehiclesGroups;
        private double[][] _groundPotentialField;

        private int _moveEnemyTicks = -1;
        private IDictionary<int, int> _apolloSoyuzIndexes = new Dictionary<int, int>();
        private int _movingNuclearGroupId = -1;

        private MyStrategyNoBuildings _myStrategyNoBuildings;

        /// <summary>
        ///     ќсновной метод стратегии, осуществл€ющий управление армией. ¬ызываетс€ каждый тик.
        /// </summary>
        /// <param name="me"></param>
        /// <param name="world"></param>
        /// <param name="game"></param>
        /// <param name="move"></param>
        public void Move(Player me, World world, Game game, Move move)
        {
            if (game.IsFogOfWarEnabled) return;
            

            if (!world.Facilities.Any())
            {
                if (_myStrategyNoBuildings == null)
                    _myStrategyNoBuildings = new MyStrategyNoBuildings();
                _myStrategyNoBuildings.Move(me, world, game, move);
                return;
            }

            //Debug.beginPost();
            //Draw(2);


            InitializeStrategy(world, game);
            InitializeTick(me, world, game, move);

            if (_nuclearVehicleId != -1 && !_vehicleById.ContainsKey(_nuclearVehicleId))
            {

            }

            if (_world.TickIndex == 0)
            {
                AirVehiclesInit();
                GroundVehiclesInit();
            }
            else
            {
                if (me.RemainingActionCooldownTicks > 0) return;

                _enemyVehicles = GetVehicles(Ownership.ENEMY);
                _enemyVehiclesGroups = GetVehicleGroups(_enemyVehicles);

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

                    if (_groups.ContainsKey(_selectedGroupId) && _isGroupCompressed[_selectedGroupId] &&
                        NeedNuclearUncompress(_groups[_selectedGroupId]))
                    {
                        Uncompress(_selectedGroupId);
                    }
                    foreach (var key in _groups.Keys)
                    {
                        if (key == _selectedGroupId) continue;
                        if (_isGroupCompressed[key] && NeedNuclearUncompress(_groups[key]))
                        {
                            Uncompress(key);
                        }
                    }
                }

                if (ExecuteDelayedMove()) return;

                CreateNewGroups();

                if (ExecuteDelayedMove()) return;

                SetFacilitiesProduction(_enemyVehiclesGroups);

                if (ExecuteDelayedMove()) return;

                if (_groups.ContainsKey(_selectedGroupId))
                {
                    IsAStarMoveFinished isAStarMoveFinished = null;
                    Shift shift = null;
                    Compress compress = null;
                    if (_selectedGroupId == 1)
                    {
                        isAStarMoveFinished = IsGroundAStarMoveFinished;
                        shift = GroundShift;
                        compress = GroundCompress;
                    }
                    else if (_selectedGroupId == 2)
                    {
                        isAStarMoveFinished = IsAirAStarMoveFinished;
                        shift = AirShift;
                        compress = AirCompress;
                    }
                    SandvichMove(_selectedGroupId, isAStarMoveFinished, shift, compress);
                    if (ExecuteDelayedMove()) return;
                }


                foreach (var key in _groups.Keys)
                {
                    if (key == _selectedGroupId) continue;
                    IsAStarMoveFinished isAStarMoveFinished = null;
                    Shift shift = null;
                    Compress compress = null;
                    if (key == 1)
                    {
                        isAStarMoveFinished = IsGroundAStarMoveFinished;
                        shift = GroundShift;
                        compress = GroundCompress;
                    }
                    else if (key == 2)
                    {
                        isAStarMoveFinished = IsAirAStarMoveFinished;
                        shift = AirShift;
                        compress = AirCompress;
                    }

                    SandvichMove(key, isAStarMoveFinished, shift, compress);
                    if (ExecuteDelayedMove()) return;
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
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[2])),
                    },
                },
                new VariantContainer()
                {
                    PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                    {
                        new PointVehilceTypeContainer(0, VehicleType.Tank,
                            GetVehiclesCenter(vehicles[VehicleType.Tank]).GetSquareDistance(_groundKeyPoints[0])),
                        new PointVehilceTypeContainer(1, VehicleType.Arrv,
                            GetVehiclesCenter(vehicles[VehicleType.Arrv]).GetSquareDistance(_groundKeyPoints[1])),
                        new PointVehilceTypeContainer(2, VehicleType.Ifv,
                            GetVehiclesCenter(vehicles[VehicleType.Ifv]).GetSquareDistance(_groundKeyPoints[2])),
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
                if (Math.Abs(GetVehiclesCenter(currentVehicles).GetSquareDistance(_groundKeyPoints[pointIndex])) > Tolerance)
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
            if (hCenter.GetSquareDistance(45, 119) < Tolerance && (fCenter.GetSquareDistance(45, 45) < Tolerance || fCenter.GetSquareDistance(45, 193) < Tolerance))
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Helicopter);
                _airPointsVehicleTypes.Add(1, VehicleType.Fighter);
            }
            else if (fCenter.GetSquareDistance(45, 119) < Tolerance &&
                     (hCenter.GetSquareDistance(45, 45) < Tolerance || hCenter.GetSquareDistance(45, 193) < Tolerance))
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Fighter);
                _airPointsVehicleTypes.Add(1, VehicleType.Helicopter);
            }
            else if (hCenter.GetSquareDistance(45, 119) < Tolerance && fCenter.GetSquareDistance(119, 119) < Tolerance)
            {
                _airPointsVehicleTypes.Add(0, VehicleType.Helicopter);
                _airPointsVehicleTypes.Add(1, VehicleType.Fighter);
            }
            else if (fCenter.GetSquareDistance(45, 119) < Tolerance && hCenter.GetSquareDistance(119, 119) < Tolerance)
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
                                GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetSquareDistance(_airKeyPoints[0])),
                            new PointVehilceTypeContainer(1,
                                VehicleType.Helicopter,
                                GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetSquareDistance(_airKeyPoints[1])),
                        },
                    },
                    new VariantContainer()
                    {
                        PointVehilceTypeContainers = new List<PointVehilceTypeContainer>()
                        {
                            new PointVehilceTypeContainer(0,
                                VehicleType.Helicopter,
                                GetVehiclesCenter(vehicles[VehicleType.Helicopter]).GetSquareDistance(_airKeyPoints[0])),
                            new PointVehilceTypeContainer(1,
                                VehicleType.Fighter,
                                GetVehiclesCenter(vehicles[VehicleType.Fighter]).GetSquareDistance(_airKeyPoints[1])),
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
                if (Math.Abs(GetVehiclesCenter(currentVehicles).GetSquareDistance(_airKeyPoints[pointIndex])) > Tolerance)
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
            var dist = Math.Sqrt(GetVehiclesCenter(vehicles).GetSquareDistance(destX, destY));

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
            var dist = Math.Sqrt(GetVehiclesCenter(vehicles).GetSquareDistance(destX, destY));

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

            var nearestGroup = GetNearestEnemyGroup(_enemyVehiclesGroups, centerPoint.X, centerPoint.Y);

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
            var speed = GetGroupRotationMaxSpeed(vehicles);
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

            //if (_vehicleById.ContainsKey(_nuclearVehicleId) && _me.NextNuclearStrikeTickIndex > -1)
            //{
            //    var nuclearVehilce = _vehicleById[_nuclearVehicleId];
            //    Debug.circleFill(nuclearVehilce.X, nuclearVehilce.Y, 2, 0x00000);
            //}
            

        }

        private void MoveToFacility(IList<Vehicle> vehicles, int groupId, Facility facility)
        {
            var facilityPoint = GetFacilityCenterPoint(facility);
            var centerPoint = GetVehiclesCenter(vehicles);

            var resFunction = new Point(0d, 0d);
            var facilityFunction = GetAttractiveFunction(facilityPoint, 1d, centerPoint.X, centerPoint.Y);
            resFunction = new Point(resFunction.X + facilityFunction.X, resFunction.Y + facilityFunction.Y);
            //var enemyVehicles = GetVehicles(Ownership.ENEMY);
            //foreach (var enemy in enemyVehicles)
            //{
            //    var enemyFunction = GetEnemyVehicleRepulsiveFunction(enemy, 0.5d, vehicles, groupId == 1);
            //    resFunction = new Point(resFunction.X + enemyFunction.X, resFunction.Y + enemyFunction.Y);
                
            //}

            if (Math.Abs(resFunction.X) < Tolerance && Math.Abs(resFunction.Y) < Tolerance) return; //уже в точке
            var angle = MathHelper.GetVectorAngle(resFunction);

            var isStatic = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                           Math.Abs(angle - _currentMovingAngle[groupId]) < Tolerance; //угол тот же
           
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

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = resFunction.X;
                move.Y = resFunction.Y;
                move.MaxSpeed = speed;
                _groupEndMovementTime[groupId] = _world.TickIndex + _moveEnemyTicks;
                _currentMoveEnemyPoint[groupId] = new Point(facilityPoint.X, facilityPoint.Y);
                _currentMovingAngle[groupId] = angle;
            });
        }

        private void MoveToSomewhere(IList<Vehicle> vehicles, int groupId, Point destPoint, Point attractiveFunction,
            IList<GroupContainer> enemyGroups, GroupContainer targetGroup = null, Circle circle = null)
        {
            var centerPoint = GetVehiclesCenter(vehicles);

            var resFunction = new Point(0d, 0d);
            resFunction = new Point(resFunction.X + attractiveFunction.X, resFunction.Y + attractiveFunction.Y);
            
            foreach (var group in enemyGroups)
            {
                if (targetGroup != null && Equals(group, targetGroup)) continue;
                var advantage = GetAdvantage(vehicles, group);
                if (advantage > 0 && !double.IsNaN(advantage)) continue;
                var enemyFunction = GetEnemyGroupRepulsiveFunction(group, 1d, vehicles);
                resFunction = new Point(resFunction.X + enemyFunction.X, resFunction.Y + enemyFunction.Y);
            }
            var borderFunction = GetBorderRepulsiveFunction(vehicles);
            resFunction = new Point(resFunction.X + borderFunction.X, resFunction.Y + borderFunction.Y);

            var vehicles0 =
                _vehicleById.Values.Where(x => x.PlayerId == _me.Id && !x.Groups.Any()).ToList();
            var allyNoGroupsfunction = GetAllyNoGroupRepulsiveFunction(vehicles, vehicles0, 1d);
            resFunction = new Point(resFunction.X + allyNoGroupsfunction.X, resFunction.Y + allyNoGroupsfunction.Y);

            //var isNoForce = Math.Abs(resFunction.X) < Tolerance && Math.Abs(resFunction.Y) < Tolerance;
            //if (!isNoForce)
            //{
                foreach (var key in _groups.Keys)
                {
                    if (key == groupId) continue;
                    var allyFunction = GetAllyGroupRepulsiveFunction(vehicles, _groups[key], 1d);
                    resFunction = new Point(resFunction.X + allyFunction.X, resFunction.Y + allyFunction.Y);
                }
            //}
          

            if (Math.Abs(resFunction.X) < Tolerance && Math.Abs(resFunction.Y) < Tolerance) return; //уже в точке

            var hasNegativeCharge = Math.Abs(resFunction.X - attractiveFunction.X) > Tolerance || Math.Abs(resFunction.Y - attractiveFunction.Y) > Tolerance;

            var resVector = new Vector(new Point(centerPoint.X, centerPoint.Y),
                new Point(centerPoint.X + resFunction.X, centerPoint.Y + resFunction.Y));

            Point targetPoint = null;
            if (circle != null)
            {
                if (Math.Abs(resVector.V.X) < Tolerance)
                {
                    var sqrt = circle.R * circle.R - Math.Pow(resVector.P1.X - circle.X, 2);
                    if (sqrt > 0)
                    {
                        var y1 = circle.Y + sqrt;
                        var y2 = circle.Y - sqrt;
                        var p1 = new Point(resVector.P1.X, y1);
                        var p2 = new Point(resVector.P1.X, y2);
                        if (!circle.IsInside(resVector.P2.X, resVector.P2.Y))
                        {
                            targetPoint = p1.GetSquareDistance(resVector.P2) < p2.GetSquareDistance(resVector.P2) ? p1 : p2;
                        }
                        else
                        {
                            targetPoint = p1.GetSquareDistance(resVector.P2) < p1.GetSquareDistance(resVector.P1) ? p1 : p2;
                        }
                    }
                }
                else
                {
                    targetPoint = MathHelper.GetVectorCircleCrossPoint(resVector, circle);
                }
            }
            if (targetPoint == null)
            {
                var destPointDist = Math.Sqrt(centerPoint.GetSquareDistance(destPoint));
                var coeff = destPointDist / resVector.Length;
                resVector.Mult(coeff);
                targetPoint = new Point(centerPoint.X + resVector.V.X, centerPoint.Y + resVector.V.Y);
            }

            //Debug.line(centerPoint.X,
            //    centerPoint.Y,
            //    targetPoint.X,
            //    targetPoint.Y,
            //    0x000000);

            var angle = MathHelper.GetVectorAngle(resFunction);

            //TODO: кривой критерий в isStatic
            var isStatic = _sandvichActions[groupId] == SandvichAction.MovingToEnemy &&
                           (centerPoint.GetSquareDistance(_currentMoveEnemyPoint[groupId]) > Tolerance //еще не долетели до точки
                           && !hasNegativeCharge && Math.Abs(angle - _currentMovingAngle[groupId]) < MaxAngle/2); //угол тот же

            _groupEndMovementTime[groupId] = _world.TickIndex + _moveEnemyTicks;
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

            var speed = GetGroupLineMaxSpeed(vehicles, targetPoint);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = targetPoint.X - centerPoint.X;
                move.Y = targetPoint.Y - centerPoint.Y;
                move.MaxSpeed = speed;
                
                _currentMoveEnemyPoint[groupId] = new Point(targetPoint.X, targetPoint.Y);
                
                _currentMovingAngle[groupId] = angle;
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
                return fcp.GetSquareDistance(ng.Center);
            });

            var airCount = _enemyVehicles.Count(v => v.IsAerial);
            var groundCount = _enemyVehicles.Count - airCount;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.SetupVehicleProduction;
                move.VehicleType = airCount > groundCount ? VehicleType.Fighter : VehicleType.Helicopter;
                move.FacilityId = orderedFacilities.First().Id;
            });

        }

        private Point GetFacilityCenterPoint(Facility facility)
        {
            return new Point(facility.Left + _game.FacilityWidth / 2d, facility.Top + _game.FacilityHeight / 2d);
        }
       
        private double GetAdvantage(IList<Vehicle> myVehicles, GroupContainer enemyGroup)
        {
            var mySumDamage = 0d;
            var mySumDurability = 0d;
            foreach (var myV in myVehicles)
            {
                var damage = enemyGroup.Vehicles.Sum(enemyV => GetDamage(myV.Type, enemyV.Type));
                if (Math.Abs(damage) > Tolerance) mySumDurability += myV.Durability;
                mySumDamage += damage / enemyGroup.Vehicles.Count;
            }

            var enemySumDamage = 0d;
            var enemySumDurability = 0d;
            foreach (var enemyV in enemyGroup.Vehicles)
            {
                var damage = myVehicles.Sum(myV => GetDamage(enemyV.Type, myV.Type));
                if (Math.Abs(damage) > Tolerance) enemySumDurability += enemyV.Durability;
                enemySumDamage += damage / myVehicles.Count;
            }

            var myPower = mySumDamage / enemySumDurability;
            var enemyPower = enemySumDamage / mySumDurability;
            return myPower - enemyPower;
        }


        private double GetDamage(VehicleType attacker, VehicleType defender)
        {
           
            switch (attacker)
            {
                case VehicleType.Fighter:
                    switch (defender)
                    {
                        case VehicleType.Fighter:
                            return _game.FighterAerialDamage - _game.FighterAerialDefence;
                        case VehicleType.Helicopter:
                            return _game.FighterAerialDamage - _game.HelicopterAerialDefence;
                        case VehicleType.Arrv:
                        case VehicleType.Ifv:
                        case VehicleType.Tank:
                            return 0d;
                    }
                    break;
                case VehicleType.Helicopter:
                    switch (defender)
                    {
                        case VehicleType.Fighter:
                            return _game.HelicopterAerialDamage - _game.FighterAerialDefence;
                        case VehicleType.Helicopter:
                            return _game.HelicopterAerialDamage - _game.HelicopterAerialDefence;
                        case VehicleType.Arrv:
                            return _game.HelicopterGroundDamage - _game.ArrvAerialDefence;
                        case VehicleType.Ifv:
                            return _game.HelicopterGroundDamage - _game.IfvAerialDefence;
                        case VehicleType.Tank:
                            return _game.HelicopterGroundDamage - _game.TankAerialDefence;
                    }
                    break;
                case VehicleType.Arrv:
                    return 0d;
               
                case VehicleType.Ifv:
                    switch (defender)
                    {
                        case VehicleType.Fighter:
                            return _game.IfvAerialDamage - _game.FighterGroundDefence;
                        case VehicleType.Helicopter:
                            return _game.IfvAerialDamage - _game.HelicopterGroundDefence;
                        case VehicleType.Arrv:
                            return _game.IfvGroundDamage - _game.ArrvGroundDefence;
                        case VehicleType.Ifv:
                            return _game.IfvGroundDamage - _game.IfvGroundDefence;
                        case VehicleType.Tank:
                            return _game.IfvGroundDamage - _game.TankGroundDefence;
                        
                    }
                    break;
                case VehicleType.Tank:
                    switch (defender)
                    {
                        case VehicleType.Fighter:
                            return 0d;
                        case VehicleType.Helicopter:
                            return _game.TankAerialDamage - _game.HelicopterGroundDefence;
                        case VehicleType.Arrv:
                            return _game.TankGroundDamage - _game.ArrvGroundDefence;
                        case VehicleType.Ifv:
                            return _game.TankGroundDamage - _game.IfvGroundDefence;
                        case VehicleType.Tank:
                            return _game.TankGroundDamage - _game.TankGroundDefence;
                    }
                    break;
            }
            throw new Exception("Unknown vehicle type");
        }


        private int GetNuclearStrikeMoveToEnemyGroupId(IList<GroupContainer> enemyGroups)
        {

            var maxDamage = 0d;
            var maxDamageDistance = double.MaxValue;
            var maxDamageKey = -1;
            foreach (var key in _groups.Keys)
            {
                var center = GetVehiclesCenter(_groups[key]);
                var nearestGroup = GetNearestEnemyGroup(enemyGroups, center.X, center.Y);
                var dist = Math.Sqrt(center.GetSquareDistance(nearestGroup.Center));
                if (dist - GetSandvichRadius(_groups[key]) -
                    GetSandvichRadius(nearestGroup.Vehicles) > EnemyVehicleDeltaShootingDist) continue;

                var enemyCenter = GetVehiclesCenter(nearestGroup.Vehicles);
                var damage = GetGroupNuclearDamage(nearestGroup.Vehicles, enemyCenter.X, enemyCenter.Y);

                if (Math.Abs(damage - maxDamage) < Tolerance)
                {
                    
                    if (dist < maxDamageDistance)
                    {
                        maxDamage = damage;
                        maxDamageDistance = dist;
                        maxDamageKey = key;
                    }
                }

               else if (damage > maxDamage)
                {
                    maxDamage = damage;
                    maxDamageDistance = dist;
                    maxDamageKey = key;
                }

            }
            return maxDamageKey;
        }

       
        private bool MakeNuclearStrike()
        {
            var myVehicles = GetVehicles(Ownership.ALLY);
            var targetPoint = GetNuclearStrikeEnemyPoint(_enemyVehiclesGroups, myVehicles);
            

            if (targetPoint == null)
            {
                if (_movingNuclearGroupId != -1) return false;

                var movingGroupId = GetNuclearStrikeMoveToEnemyGroupId(_enemyVehiclesGroups);
                if (movingGroupId == -1) return false;

                var isUncompressing = _sandvichActions[movingGroupId] == SandvichAction.Uncompress;
                var isCompressing = _sandvichActions[movingGroupId] == SandvichAction.Compressing2;

                if (isCompressing || isUncompressing) return false;

                if (_sandvichActions[movingGroupId] == SandvichAction.Rotating)
                {
                    _currentGroupAngle[movingGroupId] = _tmpGroupAngle[movingGroupId];
                }

                if (_selectedGroupId != movingGroupId)
                {
                    _importantDelayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = movingGroupId;
                    });
                    _selectedGroupId = movingGroupId;
                }

                var center = GetVehiclesCenter(_groups[movingGroupId]);
                var nearestGroup = GetNearestEnemyGroup(_enemyVehiclesGroups, center.X, center.Y);
                var speed = GetGroupLineMaxSpeed(_groups[movingGroupId], nearestGroup.Center);

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = nearestGroup.Center.X - center.X;
                    move.Y = nearestGroup.Center.Y - center.Y;
                    move.MaxSpeed = speed;
                });

                _sandvichActions[movingGroupId] = SandvichAction.NuclearStrikeMove;
                _movingNuclearGroupId = movingGroupId;
                return false;
            }
            else
            {
                var strikingVehicle = GetNuclearStrikeVehicle(targetPoint, myVehicles);
                _nuclearVehicleId = strikingVehicle.Id;

                var isUncompressing = false;
                var groupId = strikingVehicle.Groups.SingleOrDefault();
                if (groupId != 0)
                {
                    if (_sandvichActions[groupId] == SandvichAction.Rotating)
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
                    _movingNuclearGroupId = -1;
                    if (!isUncompressing)
                        _groupEndMovementTime[groupId] = _world.TickIndex + _game.TacticalNuclearStrikeDelay;

                    _nuclearVehicleId = -1;
                });


                return true;
            }
        }
     

        private void Uncompress(int groupId)
        {
            if (_sandvichActions[groupId] == SandvichAction.Rotating)
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
                    var dist = Math.Sqrt(GetVehiclesCenter(currVehicles).GetSquareDistance(destX, destY));

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
                    var dist = Math.Sqrt(GetVehiclesCenter(currVehicles).GetSquareDistance(destX, destY));

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
            if (groupId % 2 == 1)
            {
                return _game.TankSpeed * _game.SwampTerrainSpeedFactor;
            }
            return _game.HelicopterSpeed * _game.RainWeatherSpeedFactor;
        }

        private bool NeedNuclearUncompress(IList<Vehicle> vehicles)
        {
            return
                vehicles.Any(
                    v =>
                        v.GetSquaredDistanceTo(_enemy.NextNuclearStrikeX, _enemy.NextNuclearStrikeY) <=
                        _game.TacticalNuclearStrikeRadius * _game.TacticalNuclearStrikeRadius);
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
                        _isGroupCompressed[groupId] = true;
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
                    else 
                    {
                        var center = GetVehiclesCenter(vehicles);
                        var nearestGroup = GetNearestEnemyGroup(_enemyVehiclesGroups, center.X, center.Y);

                        //var rectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(vehicles));
                        var newAngle = MathHelper.GetAnlge(
                            new Vector(center,
                                new Point(center.X + 100, center.Y)),
                            new Vector(center, nearestGroup.Center));

                        var turnAngle = newAngle - _tmpGroupAngle[groupId];

                        if (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
                        else if (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

                        if (turnAngle > Math.PI / 2) turnAngle -= Math.PI;
                        else if (turnAngle < -Math.PI / 2) turnAngle += Math.PI;

                        var needStop = turnAngle * _currentAngularSpeed[groupId] < 0;
                        var isSmallAngle = Math.Abs(turnAngle) < MaxAngle / 2;
                        if (needStop || isSmallAngle)
                        {
                            _isRotating[groupId] = false;
                            _currentGroupAngle[groupId] = _tmpGroupAngle[groupId];
                            DoMilitaryAction(vehicles, groupId);
                        }
                    }

                    break;
                case SandvichAction.ApolloSoyuzRotate:
                    if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                        vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex))
                    {
                        _isRotating[groupId] = false;
                        if (_groups.ContainsKey(groupId) && _groups.ContainsKey(_apolloSoyuzIndexes[groupId]))
                        {
                            if (_world.TickIndex > _groupEndMovementTime[_apolloSoyuzIndexes[groupId]])
                            {
                                ApolloSoyuzMove(groupId, _apolloSoyuzIndexes[groupId]);
                            }
                        }
                        else
                        {
                            DoMilitaryAction(vehicles, groupId);
                        }
                    }
                    break;
                case SandvichAction.ApolloSoyuzMove:
                    if (!_groups.ContainsKey(_apolloSoyuzIndexes[groupId]))
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }
                    else
                    {
                        var group1 = _groups[groupId];
                        var rect1 = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(group1));
                        var center1 = GetVehiclesCenter(group1);

                        var group2 = _groups[_apolloSoyuzIndexes[groupId]];
                        var rect2 = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(group2));
                        var center2 = GetVehiclesCenter(group2);

                        var cp2 = MathHelper.GetNearestRectangleCrossPoint(center1, rect2, center2);
                        var cp1 = MathHelper.GetNearestRectangleCrossPoint(center2, rect1, center1);

                        var isJointFinished =
                            center1.GetDistance(center2) - (center1.GetDistance(cp1) + center2.GetDistance(cp2)) <
                            2 * Tolerance;
                        var endMovementTime = Math.Max(_groupEndMovementTime[groupId],
                            _groupEndMovementTime[_apolloSoyuzIndexes[groupId]]);
                        var allVehiclesAreStatic = vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex) &&
                                                   _groups[_apolloSoyuzIndexes[groupId]].All(v =>
                                                       _updateTickByVehicleId[v.Id] < _world.TickIndex);

                        if (isJointFinished || _world.TickIndex > endMovementTime || allVehiclesAreStatic)
                        {
                            ApolloSoyuzJoin(groupId, _apolloSoyuzIndexes[groupId]);
                        }
                    }

                    //if (_world.TickIndex > _groupEndMovementTime[groupId] ||
                    //    vehicles.All(v => _updateTickByVehicleId[v.Id] < _world.TickIndex) ||
                    //    !_groups.ContainsKey(_apolloSoyuzIndexes[groupId]))
                    //{


                    //    if (_groups.ContainsKey(groupId) && _groups.ContainsKey(_apolloSoyuzIndexes[groupId]))
                    //    {
                            
                    //    }
                    //    else
                    //    {
                    //        DoMilitaryAction(vehicles, groupId);
                    //    }
                    //}
                    break;
                case SandvichAction.ApolloSoyuzJoin:
                    if (_apolloSoyuzIndexes.ContainsKey(groupId))
                    {
                        if (_apolloSoyuzIndexes.ContainsKey(_apolloSoyuzIndexes[groupId]))
                            _apolloSoyuzIndexes.Remove(_apolloSoyuzIndexes[groupId]);
                        _apolloSoyuzIndexes.Remove(groupId);
                    }
                    DoMilitaryAction(vehicles, groupId);
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

                case SandvichAction.NuclearStrike:
                    if (_world.TickIndex > _groupEndMovementTime[groupId])
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }
                    break;
                case SandvichAction.NuclearStrikeMove:
                    if (_movingNuclearGroupId != groupId) //удар нанесла друга€ группа
                    {
                        DoMilitaryAction(vehicles, groupId);
                    }
                    
                    break;
            }
        }

        private void DoMilitaryAction(IList<Vehicle> vehicles, int groupId)
        {
            var centerPoint = GetVehiclesCenter(vehicles);
            var nearestGroup = GetNearestEnemyGroup(_enemyVehiclesGroups, centerPoint.X, centerPoint.Y, vehicles);
            var nearestGroupAngle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, nearestGroup.Center));
           
            if (groupId % 2 == 0)
            {
                int nearestFriendKey = -1;
                var minFriendDist = double.MaxValue;
                foreach (var key in _groups.Keys.Where(k => k != groupId))
                {
                    if (_sandvichActions[key] != SandvichAction.MovingToEnemy && _sandvichActions[key] != SandvichAction.Rotating) continue;
                    
                    var isThisGround = IsGroundGroup(vehicles);
                    var isThisAir = IsAirGroup(vehicles);
                    var isOtherGround = IsGroundGroup(_groups[key]);
                    var isOtherAir = IsAirGroup(_groups[key]);

                    if (isThisGround && !isOtherGround || isThisAir && !isOtherAir || isOtherGround && !isThisGround ||
                        isOtherAir && !isThisAir) continue;

                    var dist = centerPoint.GetDistance(GetVehiclesCenter(_groups[key]));
                    if (dist < minFriendDist)
                    {
                        minFriendDist = dist;
                        nearestFriendKey = key;
                    }
                }

                var nearestGroupDist = centerPoint.GetDistance(nearestGroup.Center);
                var needConnect = minFriendDist < nearestGroupDist && minFriendDist < GetSandvichRadius(vehicles) +
                                  GetSandvichRadius(_groups[nearestFriendKey]) + EnemyVehicleDeltaShootingDist;

                var needCompress =
                    vehicles.All(v => v.GetSquaredDistanceTo(centerPoint.X, centerPoint.Y) > NeedCompressionRadius * NeedCompressionRadius);

                if (needCompress)
                {
                    Compress2(centerPoint.X, centerPoint.Y, NuclearCompressionFactor, 100d, groupId);
                }
                else if (!_apolloSoyuzIndexes.ContainsKey(groupId) &&
                         !_apolloSoyuzIndexes.ContainsKey(nearestFriendKey) && needConnect)
                {
                    ApolloSoyuzRotate(groupId, nearestFriendKey);
                }
                else if (_world.TickIndex > _groupEndMovementTime[groupId])
                {
                    var targetGroup = GetNearestAdvantageEnemyGroup(_enemyVehiclesGroups, groupId);

                    if (targetGroup != null)
                    {
                        var currentDistanceToEnemyCenter = centerPoint.GetDistance(targetGroup.Center);
                        var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(targetGroup.Vehicles));
                        var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, targetGroup.Center);
                        var radius = targetGroup.Center.GetDistance(enemyCp) + EnemyVehicleDeltaShootingDist;
                        var advantage = GetAdvantage(vehicles, nearestGroup);
                        if (currentDistanceToEnemyCenter <= radius && (advantage < NeedRotationAdvantage || double.IsNaN(advantage)) &&
                            Math.Abs(_currentGroupAngle[groupId] - nearestGroupAngle) > MaxAngle
                        ) //TODO: вращаемс€ к ближайшему???
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            var attractiveFunction =
                                GetAttractiveFunction(targetGroup.Center, 1d, centerPoint.X, centerPoint.Y);
                            MoveToSomewhere(vehicles,
                                groupId,
                                targetGroup.Center,
                                attractiveFunction,
                                _enemyVehiclesGroups,
                                targetGroup);
                        }
                    }
                    else
                    {
                        var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);
                        var enemyRectangle =
                            MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
                        var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint,
                                enemyRectangle,
                                nearestGroup.Center);
                        var radius = nearestGroup.Center.GetDistance(enemyCp) + EnemyVehicleDeltaShootingDist;

                        if (currentDistanceToEnemyCenter <= radius &&
                            Math.Abs(_currentGroupAngle[groupId] - nearestGroupAngle) > MaxAngle)
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            var attractiveFunction = GetAttractiveRadiusFunction(nearestGroup.Center,
                                1d,
                                radius,
                                centerPoint.X,
                                centerPoint.Y);
                            var circle = new Circle(nearestGroup.Center.X, nearestGroup.Center.Y, radius);
                            MoveToSomewhere(vehicles,
                                groupId,
                                nearestGroup.Center,
                                attractiveFunction,
                                _enemyVehiclesGroups,
                                nearestGroup,
                                circle);
                        }
                    }
                }
            }
            else
            {
                int nearestFriendKey = -1;
                var minFriendDist = double.MaxValue;
                foreach (var key in _groups.Keys.Where(k => k != groupId))
                {

                    var isThisGround = IsGroundGroup(vehicles);
                    var isThisAir = IsAirGroup(vehicles);
                    var isOtherGround = IsGroundGroup(_groups[key]);
                    var isOtherAir = IsAirGroup(_groups[key]);

                    if (isThisGround && !isOtherGround || isThisAir && !isOtherAir || isOtherGround && !isThisGround ||
                        isOtherAir && !isThisAir) continue;

                    var dist = centerPoint.GetDistance(GetVehiclesCenter(_groups[key]));
                    if (dist < minFriendDist)
                    {
                        minFriendDist = dist;
                        nearestFriendKey = key;
                    }
                }

                var nearestGroupDist = centerPoint.GetDistance(nearestGroup.Center);
                var needConnect = minFriendDist < nearestGroupDist && minFriendDist < GetSandvichRadius(vehicles) +
                                  GetSandvichRadius(_groups[nearestFriendKey]) + EnemyVehicleDeltaShootingDist;

                var needCompress =
                    vehicles.All(v => v.GetSquaredDistanceTo(centerPoint.X, centerPoint.Y) > NeedCompressionRadius * NeedCompressionRadius);

                var nearestFacility = GetNearestFacility(centerPoint);

                var isFacilityCloser = nearestFacility != null &&
                                       centerPoint.GetDistance(nearestFacility.Left + _game.FacilityWidth / 2d,
                                           nearestFacility.Top + _game.FacilityHeight / 2d) < centerPoint.GetDistance(nearestGroup.Center);

                var hasGroundVehicle = vehicles.Any(v => v.Type != VehicleType.Helicopter && v.Type != VehicleType.Fighter);
                
                if (needCompress)
                {
                    Compress2(centerPoint.X, centerPoint.Y, NuclearCompressionFactor, 100d, groupId);
                }
                else if (needConnect)
                {
                    ApolloSoyuzRotate(groupId, nearestFriendKey);
                }
                else if (hasGroundVehicle && isFacilityCloser)
                {
                    var facilityCenter = GetFacilityCenterPoint(nearestFacility);
                    var attractiveFunction = GetAttractiveFunction(facilityCenter,
                        1d,
                        centerPoint.X,
                        centerPoint.Y);
                    MoveToSomewhere(vehicles, groupId, facilityCenter, attractiveFunction, _enemyVehiclesGroups);
                }
                else if (_world.TickIndex > _groupEndMovementTime[groupId])
                {
                    var targetGroup = GetNearestAdvantageEnemyGroup(_enemyVehiclesGroups, groupId);
                    if (targetGroup != null)
                    {
                        var currentDistanceToEnemyCenter = centerPoint.GetDistance(targetGroup.Center);
                        var enemyRectangle = MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(targetGroup.Vehicles));
                        var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint, enemyRectangle, targetGroup.Center);
                        var radius = targetGroup.Center.GetDistance(enemyCp) + EnemyVehicleDeltaShootingDist;
                        var advantage = GetAdvantage(vehicles, nearestGroup);
                        if (currentDistanceToEnemyCenter <= radius && (advantage < NeedRotationAdvantage || double.IsNaN(advantage)) &&
                            Math.Abs(_currentGroupAngle[groupId] - nearestGroupAngle) > MaxAngle
                        ) //TODO: вращаемс€ к ближайшему???
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            var attractiveFunction =
                                GetAttractiveFunction(targetGroup.Center, 1d, centerPoint.X, centerPoint.Y);
                            MoveToSomewhere(vehicles,
                                groupId,
                                targetGroup.Center,
                                attractiveFunction,
                                _enemyVehiclesGroups,
                                targetGroup);
                        }
                    }
                    else
                    {
                        var currentDistanceToEnemyCenter = centerPoint.GetDistance(nearestGroup.Center);
                        var enemyRectangle =
                            MathHelper.GetJarvisRectangle(GetVehicleGroupPoints(nearestGroup.Vehicles));
                        var enemyCp = MathHelper.GetNearestRectangleCrossPoint(centerPoint,
                                enemyRectangle,
                                nearestGroup.Center);
                        var radius = nearestGroup.Center.GetDistance(enemyCp) + EnemyVehicleDeltaShootingDist;

                        if (currentDistanceToEnemyCenter <= radius &&
                            Math.Abs(_currentGroupAngle[groupId] - nearestGroupAngle) > MaxAngle)
                        {
                            RotateToEnemy(vehicles, groupId);
                        }
                        else
                        {
                            var attractiveFunction = GetAttractiveRadiusFunction(nearestGroup.Center,
                                1d,
                                radius,
                                centerPoint.X,
                                centerPoint.Y);
                            var circle = new Circle(nearestGroup.Center.X, nearestGroup.Center.Y, radius);
                            MoveToSomewhere(vehicles,
                                groupId,
                                nearestGroup.Center,
                                attractiveFunction,
                                _enemyVehiclesGroups,
                                nearestGroup,
                                circle);
                        }
                    }

                }

            }

            
        }


        private void ApolloSoyuzRotate(int groupId1, int groupId2)
        {
            var vehicles1 = _groups[groupId1];
            var vehicles2 = _groups[groupId2];

            _sandvichActions[groupId1] = SandvichAction.ApolloSoyuzRotate;
            _sandvichActions[groupId2] = SandvichAction.ApolloSoyuzRotate;

            _apolloSoyuzIndexes.Add(groupId1, groupId2);
            _apolloSoyuzIndexes.Add(groupId2, groupId1);

            if (_selectedGroupId == groupId2)
            {
                RotateToPoint(groupId2, GetVehiclesCenter(vehicles1));
                RotateToPoint(groupId1, GetVehiclesCenter(vehicles2));
            }
            else
            {
                RotateToPoint(groupId1, GetVehiclesCenter(vehicles2));
                RotateToPoint(groupId2, GetVehiclesCenter(vehicles1));
            }
        }

        private void ApolloSoyuzMove(int groupId1, int groupId2)
        {
            var vehicles1 = _groups[groupId1];
            var vehicles2 = _groups[groupId2];
            var vehicles1Center = GetVehiclesCenter(vehicles1);
            var vehicles2Center = GetVehiclesCenter(vehicles2);
            var center = new Point((vehicles1Center.X + vehicles2Center.X) / 2,
                (vehicles1Center.Y + vehicles2Center.Y) / 2);

            _sandvichActions[groupId1] = SandvichAction.ApolloSoyuzMove;
            _sandvichActions[groupId2] = SandvichAction.ApolloSoyuzMove;

            if (_selectedGroupId == groupId2)
            {
                if (_selectedGroupId != groupId2)
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupId2;
                    });
                    _selectedGroupId = groupId2;
                }

                var speed2 = GetGroupLineMaxSpeed(vehicles2, center);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = center.X - vehicles2Center.X;
                    move.Y = center.Y - vehicles2Center.Y;
                    move.MaxSpeed = speed2;
                    _groupEndMovementTime[groupId2] = _world.TickIndex + vehicles2Center.GetDistance(center) / speed2;
                    _currentMoveEnemyPoint[groupId2] = new Point(center.X, center.Y);
                });

                if (_selectedGroupId != groupId1)
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupId1;
                    });
                    _selectedGroupId = groupId1;
                }

                var speed1 = GetGroupLineMaxSpeed(vehicles1, center);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = center.X - vehicles1Center.X;
                    move.Y = center.Y - vehicles1Center.Y;
                    move.MaxSpeed = speed1;
                    _groupEndMovementTime[groupId1] = _world.TickIndex + vehicles1Center.GetDistance(center) / speed1;
                    _currentMoveEnemyPoint[groupId1] = new Point(center.X, center.Y);
                });

            }
            else
            {
                if (_selectedGroupId != groupId1)
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupId1;
                    });
                    _selectedGroupId = groupId1;
                }

                var speed1 = GetGroupLineMaxSpeed(vehicles1, center);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = center.X - vehicles1Center.X;
                    move.Y = center.Y - vehicles1Center.Y;
                    move.MaxSpeed = speed1;
                    _groupEndMovementTime[groupId1] = _world.TickIndex + vehicles1Center.GetDistance(center) / speed1;
                    _currentMoveEnemyPoint[groupId1] = new Point(center.X, center.Y);
                });


                if (_selectedGroupId != groupId2)
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupId2;
                    });
                    _selectedGroupId = groupId2;
                }

                var speed2 = GetGroupLineMaxSpeed(vehicles2, center);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = center.X - vehicles2Center.X;
                    move.Y = center.Y - vehicles2Center.Y;
                    move.MaxSpeed = speed2;
                    _groupEndMovementTime[groupId2] = _world.TickIndex + vehicles2Center.GetDistance(center) / speed2;
                    _currentMoveEnemyPoint[groupId2] = new Point(center.X, center.Y);
                });
            }

        }

        private void ApolloSoyuzJoin(int groupId1, int groupId2)
        {
            _sandvichActions[groupId1] = SandvichAction.ApolloSoyuzJoin;
            _sandvichActions[groupId2] = SandvichAction.ApolloSoyuzJoin;

            var maxIndex = Math.Max(groupId1, groupId2);
            var minIndex = Math.Min(groupId1, groupId2);

            _sandvichActions.Remove(maxIndex);
            _groupEndMovementTime.Remove(maxIndex);
            _groupStartUncompressTick.Remove(maxIndex);
            _currentGroupAngle.Remove(maxIndex);
            _tmpGroupAngle.Remove(maxIndex);
            _currentAngularSpeed.Remove(maxIndex);
            _currentMoveEnemyPoint.Remove(maxIndex);
            _currentMovingAngle.Remove(maxIndex); 
            _isGroupCompressed.Remove(maxIndex); 
            

            if (_selectedGroupId != maxIndex)
            {
                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = maxIndex;
                });
                _selectedGroupId = maxIndex;
            }

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Dismiss;
                move.Group = maxIndex;
            });

            _importantDelayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = minIndex;
            });
            _selectedGroupId = -1;
        }


        private void RotateToPoint(int groupId, Point targetPoint)
        {
            var vehicles = _groups[groupId];
            var centerPoint = GetVehiclesCenter(vehicles);

            var newAngle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, targetPoint));

            var turnAngle = newAngle - _currentGroupAngle[groupId];

            if (turnAngle > Math.PI) turnAngle -= 2 * Math.PI;
            else if (turnAngle < -Math.PI) turnAngle += 2 * Math.PI;

            if (turnAngle > Math.PI / 2) turnAngle -= Math.PI;
            else if (turnAngle < -Math.PI / 2) turnAngle += Math.PI;

            var radius = vehicles.Max(v => v.GetDistanceTo(centerPoint.X, centerPoint.Y));
            var speed = GetGroupRotationMaxSpeed(vehicles);
            var angularSpeed = speed / radius;

            var turnTime = Math.Abs(turnAngle) / angularSpeed;

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
        

        private IList<GroupContainer> GetVehicleGroups(IList<Vehicle> vehicles)
        {
            IList<GroupContainer> groupContainers = new List<GroupContainer>();

            foreach (var v in vehicles)
            {
                var okGroupContainers = new List<GroupContainer>();
                foreach (var gc in groupContainers)
                {
                    if (gc.Vehicles.Any(gcV => gcV.GetSquaredDistanceTo(v) <= GroupMaxRadius*GroupMaxRadius))
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

       

        private GroupContainer GetNearestAdvantageEnemyGroup(IList<GroupContainer> enemyGroups, int groupId)
        {
            var hasBigGroups = enemyGroups.Any(g => g.Vehicles.Count >= ConsiderGroupVehiclesCount);
            var myVehicles = _groups[groupId];
            var center = GetVehiclesCenter(myVehicles);

            GroupContainer nearestGroup = null;
            var minDist = double.MaxValue;
            foreach (var eg in enemyGroups)
            {
                var hasMyNearVehicles = myVehicles.Any(myV =>
                                            eg.Vehicles.Any(enV =>
                                                GetActualShootingDistance(myV, !enV.IsAerial) >=
                                                myV.GetDistanceTo(enV)));

                if (hasBigGroups && eg.Vehicles.Count < ConsiderGroupVehiclesCount && !hasMyNearVehicles) continue;
                var egRadius = GetSandvichRadius(eg.Vehicles);

                var allVehicles = new List<Vehicle>(myVehicles);
                foreach (var key in _groups.Keys.Where(k => k != groupId))
                {
                    var currCenter = GetVehiclesCenter(_groups[key]);
                    var currNearestGroup = GetNearestEnemyGroup(enemyGroups, center.X, center.Y);
                    var currRadius = egRadius + EnemyVehicleDeltaShootingDist;
                    if (Equals(currNearestGroup, eg) && currCenter.GetDistance(eg.Center) < currRadius)
                    {
                        allVehicles.AddRange(_groups[key]);
                    }
                }

                var advantage = GetAdvantage(allVehicles, eg);
                if (advantage < 0 || double.IsNaN(advantage)) continue;

                var dist = eg.Center.GetSquareDistance(center.X, center.Y);
                if (dist < minDist)
                {
                    minDist = dist;
                    nearestGroup = eg;
                }
            }

            return nearestGroup;
        }

        private GroupContainer GetMostAdvantageEnemyGroup(IList<GroupContainer> enemyGroups, IList<Vehicle> myVehicles)
        {
            var hasBigGroups = enemyGroups.Any(g => g.Vehicles.Count >= ConsiderGroupVehiclesCount);

            GroupContainer nearestGroup = null;
            var maxAdvantage = 0d;
            foreach (var eg in enemyGroups)
            {
                if (hasBigGroups && eg.Vehicles.Count < ConsiderGroupVehiclesCount) continue;
                var advantage = GetAdvantage(myVehicles, eg);

                if (advantage > maxAdvantage && !double.IsNaN(advantage))
                {
                    maxAdvantage = advantage;
                    nearestGroup = eg;
                }
            }

            return nearestGroup;
        }


        private GroupContainer GetNearestEnemyGroup(IList<GroupContainer> enemyGroups, double centerX, double centerY, IList<Vehicle> myVehicles = null)
        {
            var hasBigGroups = enemyGroups.Any(g => g.Vehicles.Count >= ConsiderGroupVehiclesCount);

            GroupContainer nearestGroup = null;
            var minDist = double.MaxValue;
            foreach (var eg in enemyGroups)
            {
                var hasMyNearVehicles = myVehicles != null && myVehicles.Any(myV =>
                                            eg.Vehicles.Any(enV =>
                                                GetActualShootingDistance(myV, !enV.IsAerial) >=
                                                myV.GetDistanceTo(enV)));

                if (hasBigGroups && eg.Vehicles.Count < ConsiderGroupVehiclesCount && !hasMyNearVehicles) continue;
                
                var dist = eg.Center.GetSquareDistance(centerX, centerY);
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
                var dist = groupCenter.GetSquareDistance(facility.Left + _game.FacilityWidth / 2d,
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

        private Point GetNuclearStrikeEnemyPoint(IList<GroupContainer> enemyGroups, IList<Vehicle> myVehicles)
        {
            Point targetPoint = null;
            var maxDiffDamage = 0d;

            if (enemyGroups.All(g =>
                myVehicles.All(mv => mv.GetSquaredDistanceTo(g.Center.X, g.Center.Y) > GetActualVisualRange(mv) * GetActualVisualRange(mv))))
                return null;

            if (_isFirstNuclearStrike)
            {
                foreach (var group in enemyGroups)
                {
                    if (group.Vehicles.Count < MinNuclearStrikeCount) continue;

                    var strikingVehicle = GetNuclearStrikeVehicle(group.Center, myVehicles);

                    if (strikingVehicle == null) continue;

                    var damage = GetGroupNuclearDamage(_enemyVehicles, group.Center.X, group.Center.Y);
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
            foreach (var v in _enemyVehicles)
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

                var damage = GetGroupNuclearDamage(_enemyVehicles, v.X, v.Y);
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
                runAwayTime = _game.TacticalNuclearStrikeDelay;
            }
            else
            {
                runAwayTime = _enemy.RemainingNuclearStrikeCooldownTicks >= _game.TacticalNuclearStrikeDelay
                    ? 0
                    : _game.TacticalNuclearStrikeDelay - _enemy.RemainingNuclearStrikeCooldownTicks;
            }

            var orderedVehicles = myVehicles 
                .OrderByDescending(v => v.GetSquaredDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y))
                .ToList();

            var vehicles = orderedVehicles.Where(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                var runAwayTimeCurr = runAwayTime;
                if (groupIndex > 0 && _sandvichActions[groupIndex] == SandvichAction.Compressing2)
                {
                    runAwayTimeCurr = Math.Max(runAwayTime, (int)_groupEndMovementTime[groupIndex] - _world.TickIndex);
                }
                
                return
                    v.Durability >= v.MaxDurability * HpNuclerStrikeCoeff && GetActualVisualRange(v) >=
                    v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                    GetActualMaxSpeed(v, (int) Math.Truncate(v.X / 32d), (int) Math.Truncate(v.Y / 32d)) *
                    runAwayTimeCurr &&
                    (runAwayTimeCurr == 0 || !HasWorseNearSquares(v));
            }).ToList();

            var vehicle = vehicles.FirstOrDefault(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                if (groupIndex == 0) return true;
                var okVehiclesCount = vehicles.Count(vv => vv.Groups.Contains(groupIndex));
                var part = okVehiclesCount / _groups[groupIndex].Count;
                return _groups[groupIndex].Count < MakeNuclearStrikeCount ||
                       okVehiclesCount >= MakeNuclearStrikeCount || part > MakeNuclearStrikePart;
            });

            if (vehicle != null) return vehicle;

            vehicles = orderedVehicles.Where(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                var runAwayTimeCurr = runAwayTime;
                if (groupIndex > 0 && _sandvichActions[groupIndex] == SandvichAction.Compressing2)
                {
                    runAwayTimeCurr = Math.Max(runAwayTime, (int)_groupEndMovementTime[groupIndex] - _world.TickIndex);
                }

                return
                    GetActualVisualRange(v) >= v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                    GetActualMaxSpeed(v, (int) Math.Truncate(v.X / 32d), (int) Math.Truncate(v.Y / 32d)) *
                    runAwayTimeCurr &&
                    (runAwayTimeCurr == 0 || !HasWorseNearSquares(v));
            }).ToList();

            vehicle = vehicles.FirstOrDefault(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                if (groupIndex == 0) return true;
                var okVehiclesCount = vehicles.Count(vv => vv.Groups.Contains(groupIndex));
                var part = okVehiclesCount / _groups[groupIndex].Count;
                return _groups[groupIndex].Count < MakeNuclearStrikeCount ||
                       okVehiclesCount >= MakeNuclearStrikeCount || part > MakeNuclearStrikePart;
            });

            if (vehicle != null) return vehicle;

            vehicles = orderedVehicles.Where(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                var runAwayTimeCurr = runAwayTime;
                if (groupIndex > 0 && _sandvichActions[groupIndex] == SandvichAction.Compressing2)
                {
                    runAwayTimeCurr = Math.Max(runAwayTime, (int)_groupEndMovementTime[groupIndex] - _world.TickIndex);
                }

                return
                    GetActualVisualRange(v) >= v.GetDistanceTo(nuclearStrikePoint.X, nuclearStrikePoint.Y) +
                    GetActualMaxSpeed(v, (int) Math.Truncate(v.X / 32d), (int) Math.Truncate(v.Y / 32d)) * runAwayTimeCurr;
            }).ToList();

            vehicle = vehicles.FirstOrDefault(v =>
            {
                var groupIndex = v.Groups.FirstOrDefault();
                if (groupIndex == 0) return true;
                var okVehiclesCount = vehicles.Count(vv => vv.Groups.Contains(groupIndex));
                var part = okVehiclesCount / _groups[groupIndex].Count;
                return _groups[groupIndex].Count < MakeNuclearStrikeCount ||
                       okVehiclesCount >= MakeNuclearStrikeCount || part > MakeNuclearStrikePart;
            });

            return vehicle;
        }

        private bool IsCloseToGroupBorder(Point vehiclePoint, Rectangle vehicleRectangle, Point sourcePoint)
        {
            var cp = MathHelper.GetNearestRectangleCrossPoint(sourcePoint,
                vehicleRectangle,
                vehiclePoint);
            return cp == null || vehiclePoint.GetSquareDistance(cp) <= CloseToRectangleBorderDist * CloseToRectangleBorderDist;
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
            return GetCellModificationValue(visualRange, x, y, vehicle.Type);
        }

        private double GetCellModificationValue(double value, int x, int y, VehicleType vehicleType)
        {
            var newValue = 0d;
            switch (vehicleType)
            {
                case VehicleType.Fighter:
                case VehicleType.Helicopter:
                    var weatherType = _weatherTypeByCellXY[x][y];
                    switch (weatherType)
                    {
                        case WeatherType.Clear:
                            newValue = value * _game.ClearWeatherVisionFactor;
                            break;
                        case WeatherType.Cloud:
                            newValue = value * _game.CloudWeatherVisionFactor;
                            break;
                        case WeatherType.Rain:
                            newValue = value * _game.RainWeatherVisionFactor;
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
                            newValue = value * _game.PlainTerrainVisionFactor;
                            break;
                        case TerrainType.Forest:
                            newValue = value * _game.ForestTerrainVisionFactor;
                            break;
                        case TerrainType.Swamp:
                            newValue = value * _game.SwampTerrainVisionFactor;
                            break;
                    }
                    break;
            }
            return newValue;
        }

        private double GetActualShootingDistance(Vehicle vehicle, bool isGroundAttack)
        {
            var shootingRange = isGroundAttack ? vehicle.GroundAttackRange : vehicle.AerialAttackRange;
            var x = (int)Math.Truncate(vehicle.X / 32d);
            var y = (int)Math.Truncate(vehicle.Y / 32d);
            return GetCellModificationValue(shootingRange, x, y, vehicle.Type);
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

        private double GetGroupRotationMaxSpeed(IList<Vehicle> vehicles)
        {
            var center = GetVehiclesCenter(vehicles);
            var radius = vehicles.Max(v => v.GetDistanceTo(center.X, center.Y));

            var startX = Math.Max(MathHelper.GetSquareIndex(center.X - radius), 0);
            var endX = Math.Min( MathHelper.GetSquareIndex(center.X + radius), _game.TerrainWeatherMapColumnCount - 1);
            var startY = Math.Max(MathHelper.GetSquareIndex(center.Y - radius), 0);
            var endY = Math.Min(MathHelper.GetSquareIndex(center.Y + radius), _game.TerrainWeatherMapRowCount - 1);

            var minSpeed = double.MaxValue;
            foreach (var v in vehicles)
            {
                for (var i = startX; i <= endX; ++i)
                {
                    for (var j = startY; j <= endY; ++j)
                    {
                        var speed = GetActualMaxSpeed(v, i, j);
                        if (speed < minSpeed) minSpeed = speed;
                    }
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

        private Point GetAttractiveRadiusFunction(Point destPoint, double coeff, double radius, double x, double y)
        {
            var dist = destPoint.GetDistance(x, y);
            if (Math.Abs(dist - radius) < OrbitalWidth) return new Point(0d, 0d);
            if (dist > radius) return new Point(-coeff * (x - destPoint.X) / dist, -coeff * (y - destPoint.Y) / dist);

            double resX, resY;
            if (dist < radius / 2)
            {
                //TODO: деление на 0, если находимс€ в цетре круга
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

        private Point GetAttractiveFunction(Point destPoint, double coeff, double x, double y)
        {
            var dist = destPoint.GetDistance(x, y);
            if (Math.Abs(dist) < Tolerance) return new Point(0d, 0d);
            return new Point(- coeff * (x - destPoint.X)/dist, - coeff * (y - destPoint.Y)/dist);
        }

        private Point GetEnemyVehicleRepulsiveFunction(Vehicle enemyVehicle, double coeff, IList<Vehicle> vehicles, bool isGroundAttak)
        {
            var radius = GetActualShootingDistance(enemyVehicle, isGroundAttak) + EnemyVehicleDeltaShootingDist;
            var centerPoint = GetVehiclesCenter(vehicles);
            var dist = centerPoint.GetDistance(enemyVehicle.X, enemyVehicle.Y);
            if (dist > radius) return new Point(0d, 0d);
            var x = coeff * (centerPoint.X - enemyVehicle.X) * (1 / dist - 1 / radius) / Math.Pow(dist, 3d);
            var y = coeff * (centerPoint.Y - enemyVehicle.Y) * (1 / dist - 1 / radius) / Math.Pow(dist, 3d);
            return new Point(x, y);
        }

        private Point GetEnemyGroupRepulsiveFunction(GroupContainer groupContainer, double coeff,
            IList<Vehicle> vehicles)
        {
            var enemyRectangle =
                MathHelper.GetJarvisRectangle(groupContainer.Vehicles.Select(v => new Point(v.X, v.Y)).ToList());
            //var myRectangle =
            //    MathHelper.GetJarvisRectangle(vehicles.Select(v => new Point(v.X, v.Y)).ToList());
            var myCenter = GetVehiclesCenter(vehicles);

            var enemyCp = MathHelper.GetNearestRectangleCrossPoint(myCenter, enemyRectangle, groupContainer.Center);
            //var myCp = MathHelper.GetNearestRectangleCrossPoint(groupContainer.Center, myRectangle, myCenter);

            //var crossPointsDist = enemyCp.GetDistance(myCp);
            var myCenterDist = myCenter.GetDistance(groupContainer.Center);
            var radius = groupContainer.Center.GetDistance(enemyCp) + EnemyVehicleDeltaShootingDist;
            //Debug.circle(groupContainer.Center.X, groupContainer.Center.Y, radius, 0xFF0000);

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

        private bool IsGroundGroup(IList<Vehicle> vehicles)
        {
            return vehicles.Any(v =>
                v.Type == VehicleType.Arrv || v.Type == VehicleType.Ifv || v.Type == VehicleType.Tank);
        }

        private bool IsAirGroup(IList<Vehicle> vehicles)
        {
            return vehicles.Any(v =>
                v.Type == VehicleType.Helicopter || v.Type == VehicleType.Fighter);
        }

        private Point GetAllyGroupRepulsiveFunction(IList<Vehicle> thisVehicles, IList<Vehicle> otherVehicles, double coeff)
        {
            var isThisGround = IsGroundGroup(thisVehicles);
            var isThisAir = IsAirGroup(thisVehicles);
            var isOtherGround = IsGroundGroup(otherVehicles);
            var isOtherAir = IsAirGroup(otherVehicles);

            if (isThisGround && !isOtherGround || isThisAir && !isOtherAir || isOtherGround && !isThisGround ||
                isOtherAir && !isThisAir) return new Point(0d, 0d);

            var myCenter = GetVehiclesCenter(thisVehicles);
            var myRadius = GetSandvichRadius(thisVehicles);
            var otherCenter = GetVehiclesCenter(otherVehicles);
            var otherRadius = GetSandvichRadius(otherVehicles);

            var centersDist = myCenter.GetDistance(otherCenter);
            if (centersDist > myRadius + otherRadius + EnemyVehicleDeltaShootingDist) return new Point(0d, 0d);

            //Debug.circle(myCenter.X, myCenter.Y, myRadius + otherRadius + EnemyVehicleDeltaShootingDist, 0x00FF00);

            double x, y;
            if (centersDist < myRadius + otherRadius)
            {
                x = coeff * (myCenter.X - otherCenter.X) / centersDist;
                y = coeff * (myCenter.Y - otherCenter.Y) / centersDist;
            }
            else
            {
                x = 2 * coeff * (myCenter.X - otherCenter.X) * (1 / centersDist - 1 / (myRadius + otherRadius + EnemyVehicleDeltaShootingDist));
                y = 2 * coeff * (myCenter.Y - otherCenter.Y) * (1 / centersDist - 1 / (myRadius + otherRadius + EnemyVehicleDeltaShootingDist));
            }

            return new Point(x, y);
        }


        private Point GetAllyNoGroupRepulsiveFunction(IList<Vehicle> thisVehicles, IList<Vehicle> otherVehicles, double coeff)
        {
            var isThisGround = IsGroundGroup(thisVehicles);
            var isThisAir = IsAirGroup(thisVehicles);
            var myCenter = GetVehiclesCenter(thisVehicles);
            var myRadius = GetSandvichRadius(thisVehicles);

            var resPoint = new Point(0d, 0d);

            foreach (var v in otherVehicles)
            {
                var isGroundVehicle = v.Type == VehicleType.Arrv || v.Type == VehicleType.Ifv ||
                                      v.Type == VehicleType.Tank;
                if (isThisGround && !isGroundVehicle) continue; 
                if (isThisAir && isGroundVehicle) continue; 

                var centersDist = myCenter.GetDistance(v.X, v.Y);
                if (centersDist > myRadius + EnemyVehicleDeltaShootingDist) continue;
                
                double x, y;
                if (centersDist < myRadius)
                {
                    x = coeff * (myCenter.X - v.X) / centersDist;
                    y = coeff * (myCenter.Y - v.Y) / centersDist;
                }
                else
                {
                    x = 2 * coeff * (myCenter.X - v.X) * (1 / centersDist - 1 / (myRadius + EnemyVehicleDeltaShootingDist));
                    y = 2 * coeff * (myCenter.Y - v.Y) * (1 / centersDist - 1 / (myRadius + EnemyVehicleDeltaShootingDist));
                }

                resPoint = new Point(resPoint.X + x, resPoint.Y + y);
            }

            return resPoint;
        }


        private Point GetBorderRepulsiveFunction(IList<Vehicle> vehicles)
        {
            var resPoint = new Point(0d, 0d);

            var center = GetVehiclesCenter(vehicles);
            var radius = GetSandvichRadius(vehicles);

            if (center.X - radius < CloseBorderDist) resPoint = new Point(resPoint.X + 1d, resPoint.Y);
            if (center.Y - radius < CloseBorderDist) resPoint = new Point(resPoint.X, resPoint.Y + 1d);
            if (center.X + radius > _world.Width - CloseBorderDist) resPoint = new Point(resPoint.X - 1d, resPoint.Y);
            if (center.Y + radius > _world.Height - CloseBorderDist) resPoint = new Point(resPoint.X, resPoint.Y - 1d);

            return resPoint;
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

            _groups = new Dictionary<int, IList<Vehicle>>();
            for (var i = 1; i <= _lastGroupIndex; ++i)
            {
                var vehicles = GetVehicles(i, Ownership.ALLY);
                if (vehicles.Any())
                    _groups.Add(i, vehicles);
            }
            if (_movingNuclearGroupId != -1 && !_groups.ContainsKey(_movingNuclearGroupId)) _movingNuclearGroupId = -1;

            _moveEnemyTicks = GetMoveEnemyTicks();

        }

        private int GetMoveEnemyTicks()
        {
            var myControlCenterCount =
                _world.Facilities.Count(f => f.Type == FacilityType.ControlCenter && f.OwnerPlayerId == _me.Id);
            var actionsCount = _game.BaseActionCount +
                               myControlCenterCount * _game.AdditionalActionCountPerControlCenter;

            var myGroupsCount = _groups.Count;
            if (myGroupsCount == 1) return 5;

            var ticksPer60 = 60 / actionsCount;
            var tickPer60For2Actions = ticksPer60 * 2;
            var tickForAllGroups = tickPer60For2Actions * myGroupsCount;
            return tickForAllGroups + myGroupsCount;

        }

        private VehicleType GetGroupType(IList<Vehicle> vehicles)
        {
            var dict = new Dictionary<VehicleType, int>()
            {
                {VehicleType.Arrv, 0},
                {VehicleType.Ifv, 0},
                {VehicleType.Tank, 0},
                {VehicleType.Fighter, 0},
                {VehicleType.Helicopter, 0},
            };
            foreach (var v in vehicles)
            {
                dict[v.Type] += 1;
            }

            var resType = VehicleType.Arrv;
            foreach (var key in dict.Keys)
            {
                if (dict[key] > dict[resType])
                {
                    resType = key;
                }
            }
            return resType;
        }

        private void CreateNewGroups()
        {
            if (_lastGroupIndex < 2) return;

            var vehicles0 =
                _vehicleById.Values.Where(x => x.PlayerId == _me.Id && !x.Groups.Any()).ToList();

            var groupContainers = GetVehicleGroups(vehicles0);
            
            foreach (var gc in groupContainers)
            {
                if (gc.Vehicles.Count < SquardCount) continue;
                var groundCount = gc.Vehicles.Count(v =>
                    v.Type == VehicleType.Arrv || v.Type == VehicleType.Ifv || v.Type == VehicleType.Tank);
                var airCount =
                    gc.Vehicles.Count(v => v.Type == VehicleType.Helicopter || v.Type == VehicleType.Fighter);
                var isGround = groundCount > airCount;

                int index;
                if (isGround)
                {
                    var groundKeys = _groups.Keys.Where(k => k % 2 == 1).ToList();
                    index = groundKeys.Any() ? groundKeys.Max() + 2 : 1;
                }
                else
                {
                    var airKeys = _groups.Keys.Where(k => k % 2 == 0).ToList();
                    index = airKeys.Any() ? airKeys.Max() + 2 : 2;
                }



                _importantDelayedMoves.Enqueue(move =>
                {
                    var left = gc.Vehicles.Select(v => v.X).Min();
                    var right = gc.Vehicles.Select(v => v.X).Max();
                    var top = gc.Vehicles.Select(v => v.Y).Min();
                    var bottom = gc.Vehicles.Select(v => v.Y).Max();

                    move.Action = ActionType.ClearAndSelect;
                    move.Left = left - Tolerance;
                    move.Right = right + Tolerance;
                    move.Top = top - Tolerance;
                    move.Bottom = bottom + Tolerance;
                    move.VehicleType = GetGroupType(gc.Vehicles);
                });
                _selectedGroupId = index;

                _importantDelayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Assign;
                    move.Group = index;
                    _lastGroupIndex = Math.Max(_lastGroupIndex, index);

                    _sandvichActions[index] = SandvichAction.MovingToEnemy;
                    _groupEndMovementTime[index] = 0d;
                    _groupStartUncompressTick[index] = -1;
                    _currentGroupAngle[index] = Math.PI / 2d;
                    _tmpGroupAngle[index] = 0d;
                    _currentAngularSpeed[index] = 0d;
                    _currentMoveEnemyPoint[index] = new Point(0d, 0d);
                    _currentMovingAngle[index] = 0d;
                    _isGroupCompressed[index] = true;
                });

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
                _lastGroupIndex = 2;
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
                _lastGroupIndex = Math.Max(_lastGroupIndex, 1);
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
            NuclearStrikeMove,
            Uncompress,
            ApolloSoyuzRotate,
            ApolloSoyuzMove,
            ApolloSoyuzJoin,
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