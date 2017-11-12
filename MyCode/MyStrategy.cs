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

        private const double SquareSize = 45;
        private const double BigWeight = 999999;

        private const int RotationTime = 50;
        private const int MoveCenterTime = 100;
        private const double OkTornadoRadius = 55;
        private const double DistEps = 5;

        private const double GroupMaxRadius = 50;

        private const double MaxAngle = Math.PI/180*2;
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

        private TornadoAction _currentTornadoAction = TornadoAction.MoveCenter;
        private readonly Queue<Action<Move>> _delayedMoves = new Queue<Action<Move>>();

        //private Point _enemyPoint;
        private Game _game;

        private int _groupIndex = 1;
        private bool _isAirGroupsSet;
        private bool _isGroudGroupsSet;
        private bool _isVerticalCenterMove = true;
        private int _m;

        private Player _me;
        private Move _move;
        private int _n;

        private Random _random;
        private IList<ASquare> _squares;
        private ASquare _startSquare;

        private double _startX;
        private double _startY;
        private ASquare[,] _table;
        private TerrainType[][] _terrainTypeByCellXY;

        private int _startTornadoActionTick;
        private readonly IDictionary<long, int> _updateTickByVehicleId = new Dictionary<long, int>();

        private readonly IDictionary<long, Vehicle> _vehicleById = new Dictionary<long, Vehicle>();
        private WeatherType[][] _weatherTypeByCellXY;
        private World _world;

        /// <summary>
        ///     Основной метод стратегии, осуществляющий управление армией. Вызывается каждый тик.
        /// </summary>
        /// <param name="me"></param>
        /// <param name="world"></param>
        /// <param name="game"></param>
        /// <param name="move"></param>
        public void Move(Player me, World world, Game game, Move move)
        {
            InitializeStrategy(world, game);
            InitializeTick(me, world, game, move);


            if (me.RemainingActionCooldownTicks > 0) return;

            if (ExecuteDelayedMove()) return;

            var torandoAction = GetTornadoAction();
            TornadoMove(torandoAction);

            ExecuteDelayedMove();
        }

        private TornadoAction GetTornadoAction()
        {
            var vehicles = GetVehicles(Ownership.ALLY);
            var centerX = vehicles.Select(v => v.X).Average();
            var centerY = vehicles.Select(v => v.Y).Average();
            var radius = GetTornadoRadius();

            switch (_currentTornadoAction)
            {
                case TornadoAction.Rotate:
                    if (_world.TickIndex - _startTornadoActionTick == RotationTime || vehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] >= 1))
                    {
                        _startTornadoActionTick = _world.TickIndex;
                        _currentTornadoAction = TornadoAction.MoveCenter;
                        return TornadoAction.MoveCenter;
                    }
                    break;
                case TornadoAction.MoveCenter:
                    if (_world.TickIndex == 0)
                    {
                        return TornadoAction.MoveCenter;
                    }
                    if (_world.TickIndex - _startTornadoActionTick == MoveCenterTime || vehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] >= 1))
                    {
                        
                        _isVerticalCenterMove = !_isVerticalCenterMove;

                        //var notMovingVehiclesCount = vehicles.Count(v => _updateTickByVehicleId[v.Id] <= _startTornadoActionTick);
                        _startTornadoActionTick = _world.TickIndex;

                        if (radius <= OkTornadoRadius)
                        {
                            if (centerX < OkTornadoRadius || centerY < OkTornadoRadius)
                            {
                                _currentTornadoAction = TornadoAction.MoveToEnemy;
                                return TornadoAction.MoveToEnemy;
                            }
                            else
                            {
                                _currentTornadoAction = TornadoAction.RotateToEnemy;
                                return TornadoAction.RotateToEnemy;
                            }
                        }
                        _currentTornadoAction = TornadoAction.Rotate;
                        return TornadoAction.Rotate;
                    }
                    break;
                case TornadoAction.RotateToEnemy:
                {

                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);

                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var arrvsСenterX = arrvs.Select(v => v.X).Average();
                    var arrvsСenterY = arrvs.Select(v => v.Y).Average();
                    var arrvsPoint = new Point(arrvsСenterX, arrvsСenterY);
                    var groundVehicles = GetGroudVehicles(Ownership.ALLY);
                    var groundVehiclesX = groundVehicles.Select(v => v.X).Average();
                    var groundVehiclesY = groundVehicles.Select(v => v.Y).Average();

                    var groundRotateToEnemyPoint = GetLineCircleBehindCrossPoint(new Point(groundVehiclesX, groundVehiclesY),
                    nearestGroup.Center,
                    arrvsPoint);

                  

                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var helicoptersСenterX = helicopters.Select(v => v.X).Average();
                    var helicoptersСenterY = helicopters.Select(v => v.Y).Average();
                    var helicoptersPoint = new Point(helicoptersСenterX, helicoptersСenterY);
                    var airVehicles = GetAirVehicles(Ownership.ALLY);
                    var airVehiclesX = airVehicles.Select(v => v.X).Average();
                    var airVehiclesY = airVehicles.Select(v => v.Y).Average();

                    var airRotateToEnemyPoint = GetLineCircleBehindCrossPoint(new Point(airVehiclesX, airVehiclesY),
                    nearestGroup.Center,
                    helicoptersPoint);
                  

                    if (helicoptersPoint.GetDistance(airRotateToEnemyPoint) <= DistEps && 
                            arrvsPoint.GetDistance(groundRotateToEnemyPoint) <= DistEps)
                    {
                            //if (vehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] >= 6))
                            //{
                            _startTornadoActionTick = _world.TickIndex;
                            _delayedMoves.Enqueue(move =>
                                {
                                    move.Action = ActionType.ClearAndSelect;
                                    move.Right = _world.Width;
                                    move.Bottom = _world.Height;
                                });
                                _currentTornadoAction = TornadoAction.MoveToEnemy;
                                return TornadoAction.MoveToEnemy;
                            //}

                        //if (_tornadoActionTime == 6)
                        //{
                        //        _tornadoActionTime = 0;
                        //    }
                       
                    }

                    else if (_world.TickIndex - _startTornadoActionTick >= 6)
                    {
                            _startTornadoActionTick = _world.TickIndex;
                            return TornadoAction.RotateToEnemy;
                    }

                    break;
                }
                case TornadoAction.MoveToEnemy:
                {
                    if (radius > OkTornadoRadius)
                    {
                            _startTornadoActionTick = _world.TickIndex;
                            _currentTornadoAction = TornadoAction.MoveCenter;
                        return TornadoAction.MoveCenter;
                    }

                        //если угол на группу больше допустимого, надо вращаться
                        //var enemyGroups = GetEnemyVehicleGroups();
                        //var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);
                        //var angle = GetAnlge(new Vector(new Point(centerX, centerY), nearestGroup.Center),
                        //    new Vector(new Point(centerX, centerY), _enemyPoint));
                        //if (Math.Abs(angle) > MaxAngle)
                        //{
                        //    _tornadoActionTime = 0;
                        //    _currentTornadoAction = TornadoAction.RotateToEnemy;
                        //    return TornadoAction.RotateToEnemy;
                        //}

                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);

                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var arrvsСenterX = arrvs.Select(v => v.X).Average();
                    var arrvsСenterY = arrvs.Select(v => v.Y).Average();
                        var arrvsPoint = new Point(arrvsСenterX, arrvsСenterY);
                    var groundVehicles = GetGroudVehicles(Ownership.ALLY);
                    var groundVehiclesX = groundVehicles.Select(v => v.X).Average();
                    var groundVehiclesY = groundVehicles.Select(v => v.Y).Average();

                    var groundRotateToEnemyPoint =
                        GetLineCircleBehindCrossPoint(new Point(groundVehiclesX, groundVehiclesY),
                            nearestGroup.Center,
                            arrvsPoint);


                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var helicoptersСenterX = helicopters.Select(v => v.X).Average();
                    var helicoptersСenterY = helicopters.Select(v => v.Y).Average();
                    var helicoptersPoint = new Point(helicoptersСenterX, helicoptersСenterY);
                    var airVehicles = GetAirVehicles(Ownership.ALLY);
                    var airVehiclesX = airVehicles.Select(v => v.X).Average();
                    var airVehiclesY = airVehicles.Select(v => v.Y).Average();

                        var airRotateToEnemyPoint = GetLineCircleBehindCrossPoint(new Point(airVehiclesX, airVehiclesY),
                    nearestGroup.Center,
                    helicoptersPoint);

              
                    var isFarFromBorders = centerX >= OkTornadoRadius && centerY >= OkTornadoRadius;

                    if (isFarFromBorders && 
                            (arrvsPoint.GetDistance(groundRotateToEnemyPoint) > DistEps || helicoptersPoint.GetDistance(airRotateToEnemyPoint) > DistEps))
                    {
                            _startTornadoActionTick = _world.TickIndex;
                            _currentTornadoAction = TornadoAction.RotateToEnemy;
                        return TornadoAction.RotateToEnemy;
                    }


                    if (_world.TickIndex - _startTornadoActionTick >= 6)
                    {
                            _startTornadoActionTick = _world.TickIndex;
                            return TornadoAction.MoveToEnemy;
                    }
                    break;
                }
            }

            return TornadoAction.None;
        }

        private double GetTornadoRadius()
        {
            var vehicles = GetVehicles(Ownership.ALLY);
            var centerX = vehicles.Select(v => v.X).Average();
            var centerY = vehicles.Select(v => v.Y).Average();
            var radius = vehicles.Max(v => v.GetDistanceTo(centerX, centerY));
            return radius;
        }

        /// <summary>
        ///     Находит дальнюю точку пересчечения окружности и прямой, проходящей через центр окружности
        /// </summary>
        /// <param name="a">центр окружности</param>
        /// <param name="b">вторая точка прямой</param>
        /// <param name="c">точка на краю окружности</param>
        /// <returns></returns>
        private Point GetLineCircleBehindCrossPoint(Point a, Point b, Point c)
        {
            var radius = a.GetDistance(c);
            var x1 = a.X + radius/Math.Sqrt(1 + Math.Pow((b.Y - a.Y)/(b.X - a.X), 2));
            var x2 = a.X - radius/Math.Sqrt(1 + Math.Pow((b.Y - a.Y)/(b.X - a.X), 2));

            var y1 = (x1 - a.X)*(b.Y - a.Y)/(b.X - a.X) + a.Y;
            var y2 = (x2 - a.X)*(b.Y - a.Y)/(b.X - a.X) + a.Y;

            var p1 = new Point(x1, y1);
            var p2 = new Point(x2, y2);

            return b.GetDistance(p1) > b.GetDistance(p2) ? p1 : p2;
        }

        private IList<GroupContainer> GetEnemyVehicleGroups()
        {
            IList<GroupContainer> groupContainers = new List<GroupContainer>();
            var enemyVehicles = GetVehicles(Ownership.ENEMY);
            foreach (var v in enemyVehicles)
            {
                var okGroupContainers = new List<GroupContainer>();
                foreach (var gc in groupContainers)
                {
                    if (v.GetDistanceTo(gc.Center.X, gc.Center.Y) <= GroupMaxRadius)
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
        ///     Инциализируем стратегию.
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
                MakeDinamycAStar(world);

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
        ///     Сохраняем все входные данные в полях класса для упрощения доступа к ним, а также актуализируем сведения о каждой
        ///     технике и времени последнего изменения её состояния.
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
        ///     Достаём отложенное действие из очереди и выполняем его.
        /// </summary>
        /// <returns>Возвращает true, если и только если отложенное действие было найдено и выполнено.</returns>
        private bool ExecuteDelayedMove()
        {
            if (!_delayedMoves.Any()) return false;

            var delayedMove = _delayedMoves.Dequeue();
            delayedMove.Invoke(_move);

            return true;
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
                move.Action = ActionType.Assign;
                move.Group = 2;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
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
                move.Action = ActionType.Assign;
                move.Group = 1;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Right = _world.Width;
                move.Bottom = _world.Height;
                move.VehicleType = VehicleType.Ifv;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Assign;
                move.Group = 1;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
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


        private void TornadoMove(TornadoAction tornadoAction)
        {
            var myVehicles = GetVehicles(Ownership.ALLY);
            var centerX = myVehicles.Select(v => v.X).Average();
            var centerY = myVehicles.Select(v => v.Y).Average();

            switch (tornadoAction)
            {
                case TornadoAction.MoveCenter:


                    var vehicles1 =
                        myVehicles.Where(v => _isVerticalCenterMove ? v.Y <= centerY : v.X <= centerX).ToList();
                    var centerX1 = vehicles1.Select(v => v.X).Average();
                    var centerY1 = vehicles1.Select(v => v.Y).Average();

                    var vehicles2 =
                        myVehicles.Where(v => _isVerticalCenterMove ? v.Y > centerY : v.X > centerX).ToList();
                    var centerX2 = vehicles2.Select(v => v.X).Average();
                    var centerY2 = vehicles2.Select(v => v.Y).Average();

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Bottom = _isVerticalCenterMove ? centerY : _world.Height;
                        move.Right = _isVerticalCenterMove ? _world.Width : centerX;
                    });

                    //_delayedMoves.Enqueue(move =>
                    //{
                    //    move.Action = ActionType.Assign;
                    //    move.Group = 1;
                    //});

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = centerX - centerX1;
                        move.Y = centerY - centerY1;
                    });

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Top = _isVerticalCenterMove ? centerY + 1 : 0;
                        move.Right = _world.Width;
                        move.Left = _isVerticalCenterMove ? 0 : centerX + 1;
                        move.Bottom = _world.Height;
                    });

                    //_delayedMoves.Enqueue(move =>
                    //{
                    //    move.Action = ActionType.Assign;
                    //    move.Group = 2;
                    //});

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = centerX - centerX2;
                        move.Y = centerY - centerY2;
                    });

                    if (!_isGroudGroupsSet)
                    {
                        SetGroudGroups();
                        _isGroudGroupsSet = true;
                    }
                    else if (!_isAirGroupsSet)
                    {
                        SetAirGroups();
                        _isAirGroupsSet = true;
                    }


                    break;

                case TornadoAction.Rotate:

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Right = _world.Width;
                        move.Bottom = _world.Height;
                    });

                    // ... и поворачиваем её на случайный угол.

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Rotate;
                        move.X = centerX;
                        move.Y = centerY;
                        move.Angle = _random.NextDouble() > 0.5 ? Math.PI : -Math.PI;
                    });
                    break;


                //case TornadoAction.Rotate:
                case TornadoAction.RotateToEnemy:
                {
                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);
                    //_enemyPoint = nearestGroup.Center;

                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var arrvsСenterX = arrvs.Select(v => v.X).Average();
                    var arrvsСenterY = arrvs.Select(v => v.Y).Average();
                    var groundVehicles = GetGroudVehicles(Ownership.ALLY);
                    var groundVehiclesX = groundVehicles.Select(v => v.X).Average();
                    var groundVehiclesY = groundVehicles.Select(v => v.Y).Average();

                    var groundRotateToEnemyPoint = GetLineCircleBehindCrossPoint(new Point(groundVehiclesX, groundVehiclesY),
                        nearestGroup.Center,
                        new Point(arrvsСenterX, arrvsСenterY));


                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var helicoptersСenterX = helicopters.Select(v => v.X).Average();
                    var helicoptersСenterY = helicopters.Select(v => v.Y).Average();
                    var airVehicles = GetAirVehicles(Ownership.ALLY);
                    var airVehiclesX = airVehicles.Select(v => v.X).Average();
                    var airVehiclesY = airVehicles.Select(v => v.Y).Average();

                        var airRotateToEnemyPoint = GetLineCircleBehindCrossPoint(new Point(airVehiclesX, airVehiclesY),
                        nearestGroup.Center,
                        new Point(helicoptersСenterX, helicoptersСenterY));

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = 1;
                    });

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Rotate;
                        move.X = groundVehiclesX;
                        move.Y = groundVehiclesY;
                        move.Angle =
                            GetAnlge(new Vector(new Point(groundVehiclesX, groundVehiclesY), new Point(arrvsСenterX, arrvsСenterY)),
                                new Vector(new Point(groundVehiclesX, groundVehiclesY), groundRotateToEnemyPoint));
                        move.MaxAngularSpeed = 0.005;
                    });

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = 2;
                    });

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Rotate;
                        move.X = airVehiclesX;
                        move.Y = airVehiclesY;
                        move.Angle =
                            GetAnlge(
                                new Vector(new Point(airVehiclesX, airVehiclesY),
                                    new Point(helicoptersСenterX, helicoptersСenterY)),
                                new Vector(new Point(airVehiclesX, airVehiclesY), airRotateToEnemyPoint));
                        move.MaxAngularSpeed = 0.025;
                    });

                    break;
                }

                case TornadoAction.MoveToEnemy:
                {
                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);
                    //_enemyPoint = nearestGroup.Center;

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = nearestGroup.Center.X - centerX;
                        move.Y = nearestGroup.Center.Y - centerY;
                        move.MaxSpeed = _game.TankSpeed*_game.SwampTerrainSpeedFactor;
                    });
                    break;
                }

                //case TornadoAction.MoveCenter:

                //    var topMyVehicles = GetVehicles(1, Ownership.ALLY);
                //    var topX = topMyVehicles.Select(v => v.X).Average();
                //    var topY = topMyVehicles.Select(v => v.Y).Average();

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.ClearAndSelect;
                //        move.Group = 1;
                //    });

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.Move;
                //        move.X = centerX - topX;
                //        move.Y = centerY - topY;
                //    });

                //    var bottomMyVehicles = GetVehicles(2, Ownership.ALLY);
                //    var bottomX = bottomMyVehicles.Select(v => v.X).Average();
                //    var bottomY = bottomMyVehicles.Select(v => v.Y).Average();

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.ClearAndSelect;
                //        move.Group = 2;
                //    });

                //    _delayedMoves.Enqueue(move =>
                //    {
                //        move.Action = ActionType.Move;
                //        move.X = centerX - bottomX;
                //        move.Y = centerY - bottomY;
                //    });

                //    break;
            }
        }

        /// <summary>
        ///     Основная логика стратегии
        /// </summary>
        private void Move()
        {
            // Каждые 300 тиков ...
            //if (_world.TickIndex % 300 == 0)
            //{
            // ... для каждого типа техники ...

            foreach (var groupIndex in _reversedGroupIndexes.Keys)
            {
                var vehicleType = _reversedGroupIndexes[groupIndex];
                // ... получаем центр формации ...
                var vehicles = GetVehicles(groupIndex, Ownership.ALLY);
                var x = vehicles.Any() ? vehicles.Select(v => v.X).Average() : double.NaN;
                var y = vehicles.Any() ? vehicles.Select(v => v.Y).Average() : double.NaN;

                var targetTypeEnemyVehicles = GetPreferredTargetVehilces(vehicleType);
                // ... если этот тип может атаковать ...
                if (targetTypeEnemyVehicles == null)
                {
                    continue;
                }

                // ... получаем центр формации противника или центр мира ...
                double targetX, targetY;
                if (targetTypeEnemyVehicles.Any())
                {
                    targetX = targetTypeEnemyVehicles.Select(v => v.X).Average();
                    targetY = targetTypeEnemyVehicles.Select(v => v.Y).Average();
                }
                else
                {
                    targetX = _world.Width/2;
                    targetY = _world.Height/2;
                }

                // .. и добавляем в очередь отложенные действия для выделения и перемещения техники.
                if (!double.IsNaN(x) && !double.IsNaN(y))
                {
                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.ClearAndSelect;
                        move.Group = groupIndex;
                    });


                    var destPoint = new Point(targetX - x, targetY - y);
                    if (!CanGoToPoint(new Point(targetX, targetY), vehicleType, vehicles))
                    {
                        UpdateDynamicAStar(groupIndex, vehicles);
                        var goalI = GetSquareI(targetX);
                        var goalJ = GetSquareJ(targetY);
                        var path = Calculator.GetPath(_startSquare, _table[goalI, goalJ], _squares);
                        if (path.Count >= 2)
                        {
                            var path1 = path[1] as ASquare;
                            destPoint = new Point(path1.X + SquareSize/2 - x, path1.Y + SquareSize/2 - y);
                        }
                    }

                    _delayedMoves.Enqueue(move =>
                    {
                        move.Action = ActionType.Move;
                        move.X = destPoint.X;
                        move.Y = destPoint.Y;
                    });
                }
            }

            //Также находим центр формации наших БРЭМ ...

            var arrvVehicles = GetVehicles(_groupIndexes[VehicleType.Arrv], Ownership.ALLY);
            var arrvX = arrvVehicles.Any() ? arrvVehicles.Select(v => v.X).Average() : double.NaN;
            var arrvY = arrvVehicles.Any() ? arrvVehicles.Select(v => v.Y).Average() : double.NaN;

            // .. и отправляем их помогать танкам
            if (!double.IsNaN(arrvX) && !double.IsNaN(arrvY))
            {
                UpdateDynamicAStar(_groupIndexes[VehicleType.Arrv], arrvVehicles);
                var tankVehicles = GetVehicles(_groupIndexes[VehicleType.Tank], Ownership.ALLY);

                if (!tankVehicles.Any()) return; //TODO

                var tankX = tankVehicles.Average(v => v.X);
                var tankY = tankVehicles.Average(v => v.Y);

                if (arrvX > tankX || arrvY > tankY) return;

                var goalI = GetSquareI(tankX);
                var goalJ = GetSquareJ(tankY);

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
                if (resSquare == null) return;

                var path = Calculator.GetPath(_startSquare, resSquare, _squares);

                if (path.Count >= 2)
                {
                    _delayedMoves.Enqueue(
                        move =>
                        {
                            move.Action = ActionType.ClearAndSelect;
                            move.Group = _groupIndexes[VehicleType.Arrv];
                        });

                    var path1 = path[1] as ASquare;
                    _delayedMoves.Enqueue(
                        move =>
                        {
                            move.Action = ActionType.Move;
                            move.X = path1.X + SquareSize/2 - arrvX;
                            move.Y = path1.Y + SquareSize/2 - arrvY;
                        });
                }
            }

            //    return;
            //}

            //var allMyVehicles = GetVehicles(Ownership.ALLY);
            //// Если ни один наш юнит не мог двигаться в течение 60 тиков ...
            //if (allMyVehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] > 60))
            //{
            //    // ... находим центр нашей формации ...
            //    var x = allMyVehicles.Any() ? allMyVehicles.Select(v => v.X).Average() : Double.NaN;
            //    var y = allMyVehicles.Any() ? allMyVehicles.Select(v => v.Y).Average() : Double.NaN;

            //    // ... и поворачиваем её на случайный угол.
            //    if (!Double.IsNaN(x) && !Double.IsNaN(y))
            //    {
            //        _move.Action = ActionType.Rotate;
            //        _move.X = x;
            //        _move.Y = y;
            //        _move.Angle = _random.NextDouble() > 0.5 ? Math.PI : -Math.PI;
            //    }
            //}
        }

        private Rectangle GetGroupRectangle(IList<Vehicle> vehicles)
        {
            var x1 = vehicles.Min(v => v.X);
            var x2 = vehicles.Max(v => v.X);
            var y1 = vehicles.Min(v => v.Y);
            var y2 = vehicles.Max(v => v.Y);

            return new Rectangle
            {
                Points = new List<Point>
                {
                    new Point(x1, y1),
                    new Point(x1, y2),
                    new Point(x2, y2),
                    new Point(x2, y1)
                }
            };
        }


        private double GetAnlge(Vector v1, Vector v2)
        {
            var scalarMult = v1.V.X*v2.V.X + v1.V.Y*v2.V.Y;
            var cos = scalarMult/v1.Length/v2.Length;
            var angle = Math.Acos(cos);
            var rotate = Rotate(v1.P1, v1.P2, v2.P2);
            return rotate < 0 ? angle : -angle;
        }

        /// <summary>
        /// </summary>
        /// <param name="a"></param>
        /// <param name="b"></param>
        /// <param name="c"></param>
        /// <returns>+, если точка справа. -, если точка слева</returns>
        private double Rotate(Point a, Point b, Point c)
        {
            //-, ибо другое расположение осей x-y
            return -((b.X - a.X)*(c.Y - b.Y) - (b.Y - a.Y)*(c.X - b.X));
        }

        private bool Intersect(Point a, Point b, Point c, Point d)
        {
            return Rotate(a, b, c)*Rotate(a, b, d) <= 0 && Rotate(c, d, a)*Rotate(c, d, b) < 0;
        }

        private bool PointLoc(Rectangle rect, Point a)
        {
            var n = rect.Points.Count;
            if (Rotate(rect.Points[0], rect.Points[1], a) < 0 || Rotate(rect.Points[0], rect.Points[n - 1], a) > 0)
                return false;
            var p = 1;
            var r = n - 1;
            var q = 0;
            while (r - p > 1)
            {
                q = (p + r)/2;
                if (Rotate(rect.Points[0], rect.Points[q], a) < 0)
                {
                    r = q;
                }
                else
                {
                    p = q;
                }
            }

            return !Intersect(rect.Points[0], a, rect.Points[p], rect.Points[r]);
        }


        private bool IsIntersects(Rectangle movingRectangle, Rectangle stayingRectangle, Point destPoint)
        {
            var points = stayingRectangle.Points.ToList();
            points.Add(new Point(points.Average(p => p.X), points.Average(p => p.Y)));

            var pathRectangle = new Rectangle {Points = new List<Point>()};
            if (destPoint.X > 0 && destPoint.Y > 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
            }
            else if (destPoint.X < 0 && destPoint.Y > 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X + destPoint.X,
                    movingRectangle.Points[0].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
            }
            else if (destPoint.X > 0 && destPoint.Y < 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[0].X, movingRectangle.Points[0].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
            }
            else if (destPoint.X < 0 && destPoint.Y < 0)
            {
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X, movingRectangle.Points[1].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X, movingRectangle.Points[2].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X, movingRectangle.Points[3].Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[3].X + destPoint.X,
                    movingRectangle.Points[3].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[1].X + destPoint.X,
                    movingRectangle.Points[1].Y + destPoint.Y));
                pathRectangle.Points.Add(new Point(movingRectangle.Points[2].X + destPoint.X,
                    movingRectangle.Points[2].Y + destPoint.Y));
            }

            foreach (var point in points)
            {
                if (PointLoc(pathRectangle, point)) return true;
            }
            return false;
        }

        private bool IsSameTypes(VehicleType vt1, VehicleType vt2)
        {
            if ((vt1 == VehicleType.Fighter || vt1 == VehicleType.Helicopter) && (vt2 == VehicleType.Fighter ||
                                                                                  vt2 == VehicleType.Helicopter))
                return true;
            if (vt1 != VehicleType.Fighter && vt1 != VehicleType.Helicopter && vt2 != VehicleType.Fighter &&
                vt2 != VehicleType.Helicopter)
                return true;
            return false;
        }

        private bool CanGoToPoint(Point destPoint, VehicleType vehicleType, IList<Vehicle> vehicles)
        {
            var groupIndex = _groupIndexes[vehicleType];
            var movingRectangle = GetGroupRectangle(vehicles);

            foreach (var index in _reversedGroupIndexes.Keys)
            {
                if (index == groupIndex) continue;
                if (!IsSameTypes(vehicleType, _reversedGroupIndexes[index])) continue; // они друг другу не мешают
                var currVehicles = GetVehicles(index, Ownership.ALLY);
                if (!currVehicles.Any()) continue; // все сдохли

                var rectangle = GetGroupRectangle(currVehicles);
                if (IsIntersects(movingRectangle, rectangle, destPoint)) return false;
            }

            return true;
        }

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

        private IList<Vehicle> GetVehicles(int groupIndex, Ownership ownership = Ownership.ANY)
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


        /// <summary>
        ///     Вспомогательный метод, позволяющий для указанного типа техники получить другой тип техники, такой, что первый
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

        private void MakeDinamycAStar(World world)
        {
            _squares = new List<ASquare>();
            _n = (int) (world.Width/SquareSize);
            _m = (int) (world.Height/SquareSize);

            _table = new ASquare[_n, _m];

            for (var i = 0; i < _n; ++i)
            {
                for (var j = 0; j < _m; ++j)
                {
                    var square = new ASquare(
                        SquareSize,
                        i*SquareSize,
                        j*SquareSize,
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

                    if (i != 0 && j != 0)
                    {
                        neighbors.Add(_table[i - 1, j - 1]);
                    }

                    if (i != _n - 1 && j != _m - 1)
                    {
                        neighbors.Add(_table[i + 1, j + 1]);
                    }

                    if (i != 0 && j != _m - 1)
                    {
                        neighbors.Add(_table[i - 1, j + 1]);
                    }

                    if (i != _n - 1 && j != 0)
                    {
                        neighbors.Add(_table[i + 1, j - 1]);
                    }

                    var square = _table[i, j];
                    square.Neighbors = neighbors;
                }
            }
        }

        private void UpdateDynamicAStar(int groupIndex, IList<Vehicle> vehicles)
        {
            var centerX = vehicles.Average(v => v.X);
            var centerY = vehicles.Average(v => v.Y);

            var myLeftX = centerX - SquareSize/2;
            var leftN = (int) (myLeftX/SquareSize);
            _startX = myLeftX - leftN*SquareSize;

            var myTopY = centerY - SquareSize/2;
            var topM = (int) (myTopY/SquareSize);
            _startY = myTopY - topM*SquareSize;

            for (var i = 0; i < _n; ++i)
            {
                for (var j = 0; j < _m; ++j)
                {
                    var square = _table[i, j];
                    square.X = _startX + i*SquareSize;
                    square.Y = _startY + j*SquareSize;
                    square.Weight = 1d;
                    //_table[i, j] = new Square(_squareSize, _startX + i*_squareSize, _startY + j*_squareSize, 1d, "");
                }
            }

            _startSquare = _table[leftN, topM];

            foreach (var index in _reversedGroupIndexes.Keys)
            {
                if (index == groupIndex) continue;
                if (!IsSameTypes(_reversedGroupIndexes[groupIndex], _reversedGroupIndexes[index]))
                    continue; // они друг другу не мешают

                var currVeichles = GetVehicles(index, Ownership.ALLY);
                if (!currVeichles.Any()) continue; // все сдохли

                var currCenterX = currVeichles.Average(v => v.X);
                var currCenterY = currVeichles.Average(v => v.Y);

                var currLeftX = currCenterX - SquareSize/2;
                var currLeftN = (int) (currLeftX/SquareSize);

                var currTopY = currCenterY - SquareSize/2;
                var currTopM = (int) (currTopY/SquareSize);

                _table[currLeftN, currTopM].Weight = BigWeight;
            }


            if (leftN > 0 && _table[leftN - 1, topM].Weight == BigWeight)
            {
                if (topM > 0) _table[leftN - 1, topM - 1].Weight = BigWeight;
                if (topM < _m - 1) _table[leftN - 1, topM + 1].Weight = BigWeight;
            }

            if (topM > 0 && _table[leftN, topM - 1].Weight == BigWeight)
            {
                if (leftN > 0) _table[leftN - 1, topM - 1].Weight = BigWeight;
                if (leftN < _n - 1) _table[leftN + 1, topM - 1].Weight = BigWeight;
            }

            if (leftN < _n - 1 && _table[leftN + 1, topM].Weight == BigWeight)
            {
                if (topM > 0) _table[leftN + 1, topM - 1].Weight = BigWeight;
                if (topM < _m - 1) _table[leftN + 1, topM + 1].Weight = BigWeight;
            }

            if (topM < _m - 1 && _table[leftN, topM + 1].Weight == BigWeight)
            {
                if (leftN > 0) _table[leftN - 1, topM + 1].Weight = BigWeight;
                if (leftN < _n - 1) _table[leftN + 1, topM + 1].Weight = BigWeight;
            }
        }

        private int GetSquareI(double x)
        {
            var res = (int) ((x - _startX)/SquareSize);
            if (res < 0) return 0;
            if (res > _n - 1) return _n - 1;
            return res;
        }

        private int GetSquareJ(double y)
        {
            var res = (int) ((y - _startY)/SquareSize);
            if (res < 0) return 0;
            if (res > _m - 1) return _m - 1;
            return res;
        }

        private enum Ownership
        {
            ANY,
            ALLY,
            ENEMY
        }

        private enum TornadoAction
        {
            None,
            Rotate,
            MoveCenter,
            RotateToEnemy,
            MoveToEnemy
        }
    }
}