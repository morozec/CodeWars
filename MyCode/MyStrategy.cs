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
        

        private Player _me;
        private Move _move;
       

        private Random _random;
       

        
        private TerrainType[][] _terrainTypeByCellXY;

        private int _startTornadoActionTick;
        private readonly IDictionary<long, int> _updateTickByVehicleId = new Dictionary<long, int>();

        private readonly IDictionary<long, Vehicle> _vehicleById = new Dictionary<long, Vehicle>();
        private WeatherType[][] _weatherTypeByCellXY;
        private World _world;

        private AStar _aStar;

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

            TornadoMove();

            ExecuteDelayedMove();
        }

        private void MoveCenter(IList<Vehicle> myVehicles)
        {
            _startTornadoActionTick = _world.TickIndex;
            _currentTornadoAction = TornadoAction.MoveCenter;

            var centerX = myVehicles.Select(v => v.X).Average();
            var centerY = myVehicles.Select(v => v.Y).Average();

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
        }

        private void Rotate(double centerX, double centerY)
        {
            _startTornadoActionTick = _world.TickIndex;
            _currentTornadoAction = TornadoAction.Rotate;

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
        }

        private void RotateToEnemy(double centerX, double centerY)
        {
            _startTornadoActionTick = _world.TickIndex;
            _currentTornadoAction = TornadoAction.RotateToEnemy;

            var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
            if (arrvs.Any())
            {

                var container = GetRotateToEnemyPointContainer(centerX, centerY, arrvs, true);
                var angle = MathHelper.GetAnlge(
                    new Vector(container.SameVehiclesPoint,
                        container.VehiclesPoint),
                    new Vector(container.SameVehiclesPoint, container.RotateToEnemyPoint));

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = 1;
                });

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Rotate;
                    move.X = container.SameVehiclesPoint.X;
                    move.Y = container.SameVehiclesPoint.Y;
                    move.Angle = angle;
                    move.MaxAngularSpeed = 0.005;
                });
            }


            var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
            if (helicopters.Any())
            {
                var container = GetRotateToEnemyPointContainer(centerX, centerY, helicopters, false);
                var angle = MathHelper.GetAnlge(
                    new Vector(container.SameVehiclesPoint,
                        container.VehiclesPoint),
                    new Vector(container.SameVehiclesPoint, container.RotateToEnemyPoint));

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Group = 2;
                });

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Rotate;
                    move.X = container.SameVehiclesPoint.X;
                    move.Y = container.SameVehiclesPoint.Y;
                    move.Angle = angle;
                    move.MaxAngularSpeed = 0.025;
                });
            }
        }

        private void MoveToEnemy(double centerX, double centerY)
        {
            _startTornadoActionTick = _world.TickIndex;
            _currentTornadoAction = TornadoAction.MoveToEnemy;

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);
            //_enemyPoint = nearestGroup.Center;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = nearestGroup.Center.X - centerX;
                move.Y = nearestGroup.Center.Y - centerY;
                move.MaxSpeed = _game.TankSpeed * _game.SwampTerrainSpeedFactor;
            });
        }

        private void TornadoMove()
        {
            var myVehicles = GetVehicles(Ownership.ALLY);
            var centerX = myVehicles.Select(v => v.X).Average();
            var centerY = myVehicles.Select(v => v.Y).Average();
            var radius = GetTornadoRadius();
            switch (_currentTornadoAction)
            {
                case TornadoAction.Rotate:
                    if (_world.TickIndex - _startTornadoActionTick == RotationTime ||
                        myVehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] >= 1))
                    {
                        MoveCenter(myVehicles);
                    }
                    break;

                case TornadoAction.MoveCenter:
                    if (_world.TickIndex == 0)
                    {
                        MoveCenter(myVehicles);
                    }

                    else if (_world.TickIndex - _startTornadoActionTick == MoveCenterTime ||
                        myVehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] >= 1))
                    {
                        _isVerticalCenterMove = !_isVerticalCenterMove;
                        if (radius <= OkTornadoRadius)
                        {
                            if (centerX < OkTornadoRadius || centerY < OkTornadoRadius)
                            {
                                MoveToEnemy(centerX, centerY);
                            }
                            else
                            {
                                RotateToEnemy(centerX, centerY);
                            }
                        }
                        else
                        {
                            Rotate(centerX, centerY);
                        }
                    }
                    
                    break;


                case TornadoAction.RotateToEnemy:
                {
                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var isOkArrvs = !arrvs.Any() || GetRotateToEnemyPointDist(centerX, centerY, arrvs, true) <= DistEps;

                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var isOkHelicopters = !helicopters.Any() ||
                                            GetRotateToEnemyPointDist(centerX, centerY, helicopters, false) <= DistEps;
                   

                    if (isOkArrvs && isOkHelicopters)
                    {
                        _delayedMoves.Enqueue(move =>
                        {
                            move.Action = ActionType.ClearAndSelect;
                            move.Right = _world.Width;
                            move.Bottom = _world.Height;
                        });
                        MoveToEnemy(centerX, centerY);
                    }
                    else if (_world.TickIndex - _startTornadoActionTick >= 6)
                    {
                        RotateToEnemy(centerX, centerY);
                    }
                    break;
                }


                case TornadoAction.MoveToEnemy:
                {
                    if (radius > OkTornadoRadius)
                    {
                        MoveCenter(myVehicles);
                        return;
                    }
                    
                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var isOkArrvs = !arrvs.Any() || GetRotateToEnemyPointDist(centerX, centerY, arrvs, true) <= DistEps;
                    
                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var isOkHelicopters = !helicopters.Any() ||
                                          GetRotateToEnemyPointDist(centerX, centerY, helicopters, false) <= DistEps;
                       

                    var isFarFromBorders = centerX >= OkTornadoRadius && centerY >= OkTornadoRadius;

                    if (isFarFromBorders && (!isOkHelicopters || !isOkArrvs))
                    {
                        RotateToEnemy(centerX, centerY);
                    }
                    else if (_world.TickIndex - _startTornadoActionTick >= 6)
                    {
                        MoveToEnemy(centerX, centerY);
                    }
                    break;
                }

            }
        }

        private RotateToEnemyPointContainer GetRotateToEnemyPointContainer(double centerX, double centerY,
            IList<Vehicle> vehicles, bool isGroud)
        {
            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);

            var vehiclesCenterX = vehicles.Select(v => v.X).Average();
            var vehiclesCenterY = vehicles.Select(v => v.Y).Average();
            var vehiclesCenterPoint = new Point(vehiclesCenterX, vehiclesCenterY);
            var sameVehicles = isGroud ? GetGroudVehicles(Ownership.ALLY) : GetAirVehicles(Ownership.ALLY);
            var sameVehiclesX = sameVehicles.Select(v => v.X).Average();
            var sameVehiclesY = sameVehicles.Select(v => v.Y).Average();
            var sameVehiclesPoint = new Point(sameVehiclesX, sameVehiclesY);

            var rotateToEnemyPoint =
                MathHelper.GetLineCircleBehindCrossPoint(sameVehiclesPoint,
                    nearestGroup.Center,
                    vehiclesCenterPoint);
            return new RotateToEnemyPointContainer()
            {
                RotateToEnemyPoint = rotateToEnemyPoint,
                SameVehiclesPoint = sameVehiclesPoint,
                VehiclesPoint = vehiclesCenterPoint
            };
        }


        private double GetRotateToEnemyPointDist(double centerX, double centerY, IList<Vehicle> vehicles, bool isGroud)
        {
            var container = GetRotateToEnemyPointContainer(centerX, centerY, vehicles, isGroud);
            return container.VehiclesPoint.GetDistance(container.RotateToEnemyPoint);
        }

        private double GetRotateToEnemyPointAngle(double centerX, double centerY, IList<Vehicle> vehicles, bool isGroud)
        {
            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerX, centerY);

            var vehiclesCenterX = vehicles.Select(v => v.X).Average();
            var vehiclesCenterY = vehicles.Select(v => v.Y).Average();
            var vehiclesCenterPoint = new Point(vehiclesCenterX, vehiclesCenterY);
            var sameVehicles = isGroud ? GetGroudVehicles(Ownership.ALLY) : GetAirVehicles(Ownership.ALLY);
            var sameVehiclesX = sameVehicles.Select(v => v.X).Average();
            var sameVehiclesY = sameVehicles.Select(v => v.Y).Average();

            var rotateToEnemyPoint =
                MathHelper.GetLineCircleBehindCrossPoint(new Point(sameVehiclesX, sameVehiclesY),
                    nearestGroup.Center,
                    vehiclesCenterPoint);
            return MathHelper.GetAnlge(
                new Vector(new Point(sameVehiclesX, sameVehiclesY),
                    new Point(vehiclesCenterX, vehiclesCenterY)),
                new Vector(new Point(sameVehiclesX, sameVehiclesY), rotateToEnemyPoint));
        }

        private double GetTornadoRadius()
        {
            var vehicles = GetVehicles(Ownership.ALLY);
            var centerX = vehicles.Select(v => v.X).Average();
            var centerY = vehicles.Select(v => v.Y).Average();
            var radius = vehicles.Max(v => v.GetDistanceTo(centerX, centerY));
            return radius;
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
                        _aStar.UpdateDynamicAStar(groupIndex, vehicles, _reversedGroupIndexes, this);
                        var goalI = _aStar.GetSquareI(targetX);
                        var goalJ = _aStar.GetSquareJ(targetY);
                        var path = _aStar.GetPath(goalI, goalJ);
                        if (path.Count >= 2)
                        {
                            var path1 = path[1] as ASquare;
                            destPoint = new Point(path1.X + AStar.SquareSize/2 - x, path1.Y + AStar.SquareSize/2 - y);
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
                _aStar.UpdateDynamicAStar(_groupIndexes[VehicleType.Arrv], arrvVehicles, _reversedGroupIndexes, this);
                var tankVehicles = GetVehicles(_groupIndexes[VehicleType.Tank], Ownership.ALLY);

                if (!tankVehicles.Any()) return; //TODO

                var tankX = tankVehicles.Average(v => v.X);
                var tankY = tankVehicles.Average(v => v.Y);

                if (arrvX > tankX || arrvY > tankY) return;

                var goalI = _aStar.GetSquareI(tankX);
                var goalJ = _aStar.GetSquareJ(tankY);

                var resSquare = _aStar.GetSupportAPoint(goalI, goalJ);
                if (resSquare == null) return;

                var path = _aStar.GetPath(goalI, goalJ);

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
                            move.X = path1.X + AStar.SquareSize/2 - arrvX;
                            move.Y = path1.Y + AStar.SquareSize / 2 - arrvY;
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

        private bool CanGoToPoint(Point destPoint, VehicleType vehicleType, IList<Vehicle> vehicles)
        {
            var groupIndex = _groupIndexes[vehicleType];
            var movingRectangle = MathHelper.GetGroupRectangle(vehicles);

            foreach (var index in _reversedGroupIndexes.Keys)
            {
                if (index == groupIndex) continue;
                if (!IsSameTypes(vehicleType, _reversedGroupIndexes[index])) continue; // они друг другу не мешают
                var currVehicles = GetVehicles(index, Ownership.ALLY);
                if (!currVehicles.Any()) continue; // все сдохли

                var rectangle = MathHelper.GetGroupRectangle(currVehicles);
                if (MathHelper.IsIntersects(movingRectangle, rectangle, destPoint)) return false;
            }

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

        public enum Ownership
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