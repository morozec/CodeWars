using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk;
using Com.CodeGame.CodeRacing2015.DevKit.CSharpCgdk.AStar;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.MyCode;
using IPA.AStar;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk
{
    public sealed class MyStrategy : IStrategy
    {
        private const double Tolerance = 1E-3;

        private const int RotationTime = 50;
        private const int MoveCenterTime = 100;
        private const double IsCompressedEnoughTimeDelta = 5;
        private const double RotationRadius = 50;
        private const double FarBorderDistance = 100;
        private const double DistEps = 5;
        private const double ShootingDistance = 64d;

        private const double NuclearStrikeDistDelta = 30;

        private const double GroupMaxRadius = 50;

        private const double MaxAngle = Math.PI/180*2;
        private const double AcceptableRadiusChange = 2d;
        private const int AcceptableVehiclesLoss = 10;
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
        private Player _enemy;
        private Move _move;
       

        private Random _random;
       

        
        private TerrainType[][] _terrainTypeByCellXY;

        private int _startTornadoActionTick;
        private readonly IDictionary<long, int> _updateTickByVehicleId = new Dictionary<long, int>();

        private readonly IDictionary<long, Vehicle> _vehicleById = new Dictionary<long, Vehicle>();
        private WeatherType[][] _weatherTypeByCellXY;
        private World _world;

        private AStar _aStar;

        private int _tornadoVehiclesCount = 500;
        private double _tornadoRadius = 100;

        private SandvichAction _sandvichAction = SandvichAction.Init;

        private IList<APoint> _groundAStarPath = null;
        private int _groundPathIndex = 0;
        private int _groundPointIndex = 0;
        private IDictionary<int, VehicleType> _groundPointsVehicleTypes;

        private IList<APoint> _airAStarPath = null;
        private int _airPathIndex = 0;
        private int _airPointIndex = 0;
        private IDictionary<int, VehicleType> _airPointsVehicleTypes;

        private IList<Point> _keyPoints = new List<Point>()
        {
            new Point(45, 119),
            new Point(119, 119),
            new Point(193, 119),
        };

        private const double SquardDelta = 6;
        private const double CompressinCoeff = 0.75;
        private double _endMovementTime;
        private double _airEndMovementTime;
        private double _currentAngle;
        private bool _isCompressed = false;
        private int _vehiclesCount = 500;

        private double _enemyNuclearStrikeX;
        private double _enemyNuclearStrikeY;

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

            //TornadoMove();
            SandvichMove();

            ExecuteDelayedMove();
        }

        private void MakeMoveToKeyPoint(VehicleType vehicleType, IList<Vehicle> vehicles)
        {
            var startI =
                (int)
                    ((GetVehiclesCenter(vehicles).X - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var startJ =
                (int)
                    ((GetVehiclesCenter(vehicles).Y - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var path = _aStar.GetPath(startI, startJ, _groundPointIndex, 1);
            _groundAStarPath = _aStar.GetStraightPath(path);
            _groundPathIndex = 1;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });

            var destX = (_groundAStarPath[_groundPathIndex] as ASquare).CenterX;
            var destY = (_groundAStarPath[_groundPathIndex] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;
                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + dist / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });
            
        }

        private void AirMakeMoveToKeyPoint(VehicleType vehicleType, IList<Vehicle> vehicles)
        {
            var startI =
                (int)
                ((GetVehiclesCenter(vehicles).X - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var startJ =
                (int)
                ((GetVehiclesCenter(vehicles).Y - 8 + AStar.SquareSize / 2) / AStar.SquareSize) - 1;
            var path = _aStar.GetPath(startI, startJ, _airPointIndex, 1);
            _airAStarPath = _aStar.GetStraightPath(path);
            _airPathIndex = 1;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
                move.VehicleType = vehicleType;
            });

            var destX = (_airAStarPath[_airPathIndex] as ASquare).CenterX;
            var destY = (_airAStarPath[_airPathIndex] as ASquare).CenterY;
            var dist = GetVehiclesCenter(vehicles).GetDistance(destX, destY);

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = destX - GetVehiclesCenter(vehicles).X;
                move.Y = destY - GetVehiclesCenter(vehicles).Y;

                _airEndMovementTime = Math.Max(_airEndMovementTime,
                    _world.TickIndex + dist / (_game.HelicopterSpeed * _game.RainWeatherSpeedFactor));
            });

        }

        private void Scale()
        {
            var groundVehicles = GetGroudVehicles(Ownership.ALLY);

            var minY = groundVehicles.Min(v => v.Y);
            for (var i = 0; i < 5; ++i)
            {
                var y = minY + SquardDelta * i;

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - _game.VehicleRadius;
                    move.Bottom = y + _game.VehicleRadius;
                    move.Left = 0;
                    move.Right = _world.Width;
                });


                var moveY = SquardDelta * 2 * (5 - i);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = -moveY;

                    _endMovementTime = Math.Max(_endMovementTime,
                        _world.TickIndex + moveY / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
                });
            }

            var maxY = groundVehicles.Max(v => v.Y);
            for (var i = 0; i < 4; ++i)
            {
                var y = maxY - SquardDelta * i;

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.ClearAndSelect;
                    move.Top = y - _game.VehicleRadius;
                    move.Bottom = y + _game.VehicleRadius;
                    move.Left = 0;
                    move.Right = _world.Width;
                });

                var moveY = SquardDelta * 2 * (4 - i);

                _delayedMoves.Enqueue(move =>
                {
                    move.Action = ActionType.Move;
                    move.X = 0;
                    move.Y = moveY;

                    _endMovementTime = Math.Max(_endMovementTime,
                        _world.TickIndex + moveY / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));

                });
            }
        }

        private void Shift()
        {
            //var vt0 = _groundPointsVehicleTypes[0];
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = 8 + AStar.SquareSize;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = -SquardDelta;

                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + SquardDelta / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });

            //var vt2 = _groundPointsVehicleTypes[2];
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Left = 8 + AStar.SquareSize * 2;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = 0;
                move.Y = SquardDelta;

                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + SquardDelta / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });

        }

        private void Compress()
        {
            //var vt0 = _groundPointsVehicleTypes[0];
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = 8 + AStar.SquareSize;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = AStar.SquareSize;
                move.Y = 0;

                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + AStar.SquareSize / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });

            //var vt2 = _groundPointsVehicleTypes[2];
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Left = 8 + AStar.SquareSize * 2;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = -AStar.SquareSize;
                move.Y = 0;

                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + AStar.SquareSize / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });

        }

        private void Compress2(double x, double y)
        {
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = CompressinCoeff;
                move.X = x;
                move.Y = y;
                _endMovementTime = Math.Max(_endMovementTime, _world.TickIndex + _game.TacticalNuclearStrikeDelay);
            });

            _isCompressed = true;
        }

        private void RotateToEnemy()
        {
            var myVehilces = GetVehicles(Ownership.ALLY);
            var centerPoint = GetVehiclesCenter(myVehilces);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            //TODO: возможно, не стоит лишний раз выделять
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            var newAngle = MathHelper.GetAnlge(
                new Vector(centerPoint,
                    new Point(centerPoint.X + 100, centerPoint.Y)),
                new Vector(centerPoint, nearestGroup.Center));
            var turnAngle = newAngle - _currentAngle;

            var radius = myVehilces.Max(v => v.GetDistanceTo(centerPoint.X, centerPoint.Y));
            var speed = _game.TankSpeed * _game.SwampTerrainSpeedFactor;
            var angularSpeed = speed / radius;

            var turnTime = turnAngle / angularSpeed;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Rotate;
                move.X = centerPoint.X;
                move.Y = centerPoint.Y;
                move.Angle = turnAngle;
                _currentAngle = newAngle;
                _endMovementTime = _world.TickIndex + turnTime;
                move.MaxAngularSpeed = angularSpeed;
            });
            
        }

        private void MoveToEnemy()
        {
            var myVehilces = GetVehicles(Ownership.ALLY);
            var centerPoint = GetVehiclesCenter(myVehilces);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            //TODO: возможно, не стоит лишний раз выделять
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            var isFarFromBorder = centerPoint.X >= FarBorderDistance && centerPoint.Y >= FarBorderDistance &&
                                  centerPoint.X <= _world.Width - FarBorderDistance &&
                                  centerPoint.Y <= _world.Height - FarBorderDistance;
            var dx = isFarFromBorder ? ShootingDistance * Math.Cos(_currentAngle) : 0d;
            var dy = isFarFromBorder ? ShootingDistance * Math.Sin(_currentAngle) : 0d;

            var dist = centerPoint.GetDistance(nearestGroup.Center.X, nearestGroup.Center.Y);
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = nearestGroup.Center.X - centerPoint.X - dx;
                move.Y = nearestGroup.Center.Y - centerPoint.Y - dy;
                move.MaxSpeed = _game.TankSpeed * _game.SwampTerrainSpeedFactor;
                _endMovementTime = Math.Max(_endMovementTime,
                    _world.TickIndex + dist / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
            });
        }

        private bool MakeNuclearStrike()
        {
            var myVehilces = GetVehicles(Ownership.ALLY);
            var centerPoint = GetVehiclesCenter(myVehilces);

            var enemyGroups = GetEnemyVehicleGroups();
            var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

            var canStrikeMyVehilces = myVehilces.Where(v =>
                v.VisionRange >= v.GetDistanceTo(nearestGroup.Center.X, nearestGroup.Center.Y) + NuclearStrikeDistDelta);

            if (!canStrikeMyVehilces.Any()) return false;

            var faarestMyVehicles = canStrikeMyVehilces
                .OrderBy(v => v.GetDistanceTo(nearestGroup.Center.X, nearestGroup.Center.Y)).Last();

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.TacticalNuclearStrike;
                move.X = nearestGroup.Center.X;
                move.Y = nearestGroup.Center.Y;
                move.VehicleId = faarestMyVehicles.Id;
                _endMovementTime = _world.TickIndex + _game.TacticalNuclearStrikeDelay;
            });

            return true;
        }

        private void Uncompress()
        {
            //TODO: возможно, не стоит лишний раз выделять
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.ClearAndSelect;
                move.Bottom = _world.Height;
                move.Right = _world.Width;
            });

            _enemyNuclearStrikeX = _enemy.NextNuclearStrikeX;
            _enemyNuclearStrikeY = _enemy.NextNuclearStrikeY;

            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Scale;
                move.Factor = 10;
                move.X = _enemy.NextNuclearStrikeX;
                move.Right = _enemy.NextNuclearStrikeY;
                _endMovementTime = _enemy.NextNuclearStrikeTickIndex;
            });

        }

        private void SandvichMove()
        {
            if (_sandvichAction != SandvichAction.Uncompress && _enemy.NextNuclearStrikeTickIndex > -1)
            {
                _sandvichAction = SandvichAction.Uncompress;
                Uncompress();
                return;
            }

            switch (_sandvichAction)
            {
                case SandvichAction.Init:
                {
                    var vehicles = new Dictionary<VehicleType, IList<Vehicle>>()
                    {
                        {VehicleType.Arrv, GetVehicles(Ownership.ALLY, VehicleType.Arrv)},
                        {VehicleType.Ifv, GetVehicles(Ownership.ALLY, VehicleType.Ifv)},
                        {VehicleType.Tank, GetVehicles(Ownership.ALLY, VehicleType.Tank)},
                    };

                    var point0Type =
                        vehicles.Keys.OrderBy(key => GetVehiclesCenter(vehicles[key]).GetDistance(_keyPoints[0]))
                            .First();
                    var point1Type =
                        vehicles.Keys.Where(key => key != point0Type)
                            .OrderBy(key => GetVehiclesCenter(vehicles[key]).GetDistance(_keyPoints[1]))
                            .First();
                    var point2Type =
                        vehicles.Keys.Single(key => key != point0Type && key != point1Type);

                    _groundPointsVehicleTypes = new Dictionary<int, VehicleType>()
                    {
                        {0, point0Type},
                        {1, point1Type},
                        {2, point2Type},
                    };

                    _groundPointIndex = 0;
                    var currentType = _groundPointsVehicleTypes[_groundPointIndex];
                    var currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                    while (Math.Abs(GetVehiclesCenter(currentVehicles).GetDistance(_keyPoints[_groundPointIndex])) <=
                           Tolerance && ++_groundPointIndex < 3)
                    {
                        currentType = _groundPointsVehicleTypes[_groundPointIndex];
                        currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                    }

                    var airVehilces = new Dictionary<VehicleType, IList<Vehicle>>()
                    {
                        {VehicleType.Fighter, GetVehicles(Ownership.ALLY, VehicleType.Fighter)},
                        {VehicleType.Helicopter, GetVehicles(Ownership.ALLY, VehicleType.Helicopter)},
                    };

                    var airPoint0Type =
                        airVehilces.Keys.OrderBy(key => GetVehiclesCenter(airVehilces[key]).GetDistance(_keyPoints[0]))
                            .First();
                    var airPoint1Type =
                        airVehilces.Keys.Single(key => key != airPoint0Type);

                    _airPointsVehicleTypes = new Dictionary<int, VehicleType>()
                    {
                        {0, airPoint0Type},
                        {1, airPoint1Type},
                    };

                    _airPointIndex = 0;
                    var airCurrentType = _airPointsVehicleTypes[_airPointIndex];
                    var airCurrentVehicles = GetVehicles(Ownership.ALLY, airCurrentType);
                    while (Math.Abs(GetVehiclesCenter(airCurrentVehicles).GetDistance(_keyPoints[_airPointIndex])) <=
                           Tolerance && ++_airPointIndex < 2)
                    {
                        airCurrentType = _airPointsVehicleTypes[_airPointIndex];
                        airCurrentVehicles = GetVehicles(Ownership.ALLY, airCurrentType);
                    }

                    if (_groundPointIndex == 3 && _airPointIndex == 2)
                    {
                        _sandvichAction = SandvichAction.Scaling;
                        Scale();
                    }
                    else
                    {
                        _sandvichAction = SandvichAction.AStarMove;
                        MakeMoveToKeyPoint(currentType, currentVehicles);
                        AirMakeMoveToKeyPoint(airCurrentType, airCurrentVehicles);
                    }

                    break;
                }

                case SandvichAction.AStarMove:
                {
                    //TODO: данный тип уже может отсутствовать

                    

                    var canScaleGround = false;

                    if (_world.TickIndex >= _endMovementTime)
                    {
                        if (_groundPathIndex < _groundAStarPath.Count - 1)
                        {
                            var vehicleType = _groundPointsVehicleTypes[_groundPointIndex];
                            var currVehicles = GetVehicles(Ownership.ALLY, vehicleType);

                            _groundPathIndex++;

                            _delayedMoves.Enqueue(move =>
                            {
                                move.Action = ActionType.ClearAndSelect;
                                move.Bottom = _world.Height;
                                move.Right = _world.Width;
                                move.VehicleType = vehicleType;
                            });

                            var destX = (_groundAStarPath[_groundPathIndex] as ASquare).CenterX;
                            var destY = (_groundAStarPath[_groundPathIndex] as ASquare).CenterY;
                            var dist = GetVehiclesCenter(currVehicles).GetDistance(destX, destY);

                            _delayedMoves.Enqueue(move =>
                            {
                                move.Action = ActionType.Move;
                                move.X = destX - GetVehiclesCenter(currVehicles).X;
                                move.Y = destY - GetVehiclesCenter(currVehicles).Y;

                                _endMovementTime = Math.Max(_endMovementTime,
                                    _world.TickIndex + dist / (_game.TankSpeed * _game.SwampTerrainSpeedFactor));
                            });
                        }
                        else if (_groundPointIndex < 2)
                        {
                            _groundPointIndex++;

                            var currentType = _groundPointsVehicleTypes[_groundPointIndex];
                            var currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                            while (Math.Abs(GetVehiclesCenter(currentVehicles)
                                       .GetDistance(_keyPoints[_groundPointIndex])) <= Tolerance && ++_groundPointIndex < 3)
                            {
                                currentType = _groundPointsVehicleTypes[_groundPointIndex];
                                currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                            }

                            if (_groundPointIndex == 3)
                            {
                                canScaleGround = true;
                            }
                            else
                            {
                                MakeMoveToKeyPoint(currentType, currentVehicles);
                            }
                        }
                        else
                        {
                            canScaleGround = true;
                        }
                    }

                    
                    var canScaleAir = false;

                    if (_world.TickIndex >= _airEndMovementTime)
                    {
                        if (_airPathIndex < _airAStarPath.Count - 1)
                        {
                            var airVehicleType = _airPointsVehicleTypes[_airPointIndex];
                            var airCurrVehicles = GetVehicles(Ownership.ALLY, airVehicleType);

                             _airPathIndex++;

                            _delayedMoves.Enqueue(move =>
                            {
                                move.Action = ActionType.ClearAndSelect;
                                move.Bottom = _world.Height;
                                move.Right = _world.Width;
                                move.VehicleType = airVehicleType;
                            });

                            var destX = (_airAStarPath[_airPathIndex] as ASquare).CenterX;
                            var destY = (_airAStarPath[_airPathIndex] as ASquare).CenterY;
                            var dist = GetVehiclesCenter(airCurrVehicles).GetDistance(destX, destY);

                            _delayedMoves.Enqueue(move =>
                            {
                                move.Action = ActionType.Move;
                                move.X = destX - GetVehiclesCenter(airCurrVehicles).X;
                                move.Y = destY - GetVehiclesCenter(airCurrVehicles).Y;

                                _airEndMovementTime = Math.Max(_airEndMovementTime,
                                    _world.TickIndex + dist / (_game.HelicopterSpeed * _game.RainWeatherSpeedFactor));
                            });
                                
                        }
                        else if (_airPointIndex < 1)
                        {
                            _airPointIndex++;

                            var currentType = _airPointsVehicleTypes[_airPointIndex];
                            var currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                            while (Math.Abs(GetVehiclesCenter(currentVehicles)
                                        .GetDistance(_keyPoints[_airPointIndex])) <= Tolerance && ++_airPointIndex < 2)
                            {
                                currentType = _airPointsVehicleTypes[_airPointIndex];
                                currentVehicles = GetVehicles(Ownership.ALLY, currentType);
                            }

                            if (_airPointIndex == 2)
                            {
                                canScaleAir = true;
                            }
                            else
                            {
                                AirMakeMoveToKeyPoint(currentType, currentVehicles);
                            }
                        }
                        else
                        {
                            canScaleAir = true;
                        }
                    }

                    if (canScaleGround && canScaleAir)
                    {
                        _sandvichAction = SandvichAction.Scaling;
                        Scale();
                    }

                    break;
                }

                case SandvichAction.Scaling:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.Shifting;
                        Shift();
                    }
                    break;

                case SandvichAction.Shifting:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.Compressing;
                        Compress();
                    }
                    break;

                case SandvichAction.Compressing:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.Rotating;
                        RotateToEnemy();
                    }
                    break;

                case SandvichAction.Compressing2:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.MovingToEnemy;
                        MoveToEnemy();
                    }
                    break;
                case SandvichAction.Rotating:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        if (_isCompressed)
                        {
                            _sandvichAction = SandvichAction.MovingToEnemy;
                            MoveToEnemy();
                        }
                        else
                        {
                            _sandvichAction = SandvichAction.Compressing2;
                            var centerPoint = GetVehiclesCenter(GetVehicles(Ownership.ALLY));
                            Compress2(centerPoint.X, centerPoint.Y);
                        }
                    }
                    break;
                case SandvichAction.MovingToEnemy:
                {
                    var vehicles = GetVehicles(Ownership.ALLY);
                    var centerPoint = GetVehiclesCenter(vehicles);
                    var enemyGroups = GetEnemyVehicleGroups();
                    var nearestGroup = GetNearestEnemyGroup(enemyGroups, centerPoint.X, centerPoint.Y);

                    var angle = MathHelper.GetAnlge(
                                    new Vector(centerPoint,
                                        new Point(centerPoint.X + 100, centerPoint.Y)),
                                    new Vector(centerPoint, nearestGroup.Center));

                    //if (centerPoint.GetDistance(nearestGroup.Center) < 50)
                    //{
                    //    _sandvichAction = SandvichAction.ScaleToEnemy;
                    //    ScaleToEnemy();
                    //}
                    //if (_vehiclesCount - vehicles.Count >= AcceptableVehiclesLoss)
                    //{
                    //    _vehiclesCount = vehicles.Count;
                    //    _sandvichAction = SandvichAction.Compressing2;
                    //    Compress2();
                    //}

                    if (_me.RemainingNuclearStrikeCooldownTicks == 0 && MakeNuclearStrike())
                    {
                        _sandvichAction = SandvichAction.NuclearStrike;
                    }
                    else if (Math.Abs(_currentAngle - angle) > MaxAngle)
                    {
                        _sandvichAction = SandvichAction.Rotating;
                        RotateToEnemy();
                    }
                    else
                    {
                        MoveToEnemy();
                    }
                    
                    break;
                }
                case SandvichAction.NuclearStrike:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.MovingToEnemy;
                        MoveToEnemy();
                    }
                    break;
                case SandvichAction.Uncompress:
                    if (_world.TickIndex >= _endMovementTime)
                    {
                        _sandvichAction = SandvichAction.Compressing2;
                        Compress2(_enemyNuclearStrikeX, _enemyNuclearStrikeY);
                    }
                    break;

            }



            //IList<Vehicle> point1NearestGroup = arrvs;
            //var minDist = arrvsCenter.GetDistance(point1);
            //if (ifvsCenter.GetDistance(point1) < minDist)
            //{
            //    minDist = ifvsCenter.GetDistance(point1);
            //    point1NearestGroup = ifvs;
            //}

            //if (tanksCenter.GetDistance(point1) < minDist)
            //{
            //    minDist = tanksCenter.GetDistance(point1);
            //    point1NearestGroup = tanks;
            //}



        }

        private void MoveCenter(IList<Vehicle> myVehicles)
        {
            
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
         
            _delayedMoves.Enqueue(move =>
            {
                move.Action = ActionType.Move;
                move.X = centerX - centerX2;
                move.Y = centerY - centerY2;
                _startTornadoActionTick = _world.TickIndex;
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
                _startTornadoActionTick = _world.TickIndex;
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
                    _startTornadoActionTick = _world.TickIndex;
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
                    _startTornadoActionTick = _world.TickIndex;
                });
            }
        }

        private void MoveToEnemy(double centerX, double centerY)
        {
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
                _startTornadoActionTick = _world.TickIndex;
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
                        var isCompressedEnough = _world.TickIndex - _startTornadoActionTick <= IsCompressedEnoughTimeDelta;
                            //myVehicles.Count(v => _updateTickByVehicleId[v.Id] >= _startTornadoActionTick) <= 10;
                        if (isCompressedEnough)
                        {
                            _tornadoRadius = radius;
                            _tornadoVehiclesCount = myVehicles.Count;
                            if (centerX < RotationRadius || centerY < RotationRadius)
                            {
                                _delayedMoves.Enqueue(move =>
                                {
                                    move.Action = ActionType.ClearAndSelect;
                                    move.Right = _world.Width;
                                    move.Bottom = _world.Height;
                                });
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
                    else
                    {
                        RotateToEnemy(centerX, centerY);
                    }
                    break;
                }


                case TornadoAction.MoveToEnemy:
                {
                    if (radius - _tornadoRadius >= AcceptableRadiusChange ||
                        _tornadoVehiclesCount - myVehicles.Count >= AcceptableVehiclesLoss)
                    {
                        MoveCenter(myVehicles);
                        return;
                    }

                    var arrvs = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                    var isOkArrvs = !arrvs.Any() || GetRotateToEnemyPointDist(centerX, centerY, arrvs, true) <= DistEps;
                    
                    var helicopters = GetVehicles(Ownership.ALLY, VehicleType.Helicopter);
                    var isOkHelicopters = !helicopters.Any() ||
                                          GetRotateToEnemyPointDist(centerX, centerY, helicopters, false) <= DistEps;
                       

                    var isFarFromBorders = centerX >= RotationRadius && centerY >= RotationRadius;

                    if (isFarFromBorders && (!isOkHelicopters || !isOkArrvs))
                    {
                        RotateToEnemy(centerX, centerY);
                    }
                    else 
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
                        var path = _aStar.GetPath(0, 0, goalI, goalJ);
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

                var path = _aStar.GetPath(0, 0, goalI, goalJ);

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

        public Point GetVehiclesCenter(IList<Vehicle> vehicles)
        {
            return new Point(vehicles.Select(v => v.X).Average(), vehicles.Select(v => v.Y).Average());
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

        private enum SandvichAction
        {
            Init,
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
    }
}