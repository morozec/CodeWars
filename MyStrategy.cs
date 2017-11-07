using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk.Model;

namespace Com.CodeGame.CodeWars2017.DevKit.CSharpCgdk {
    public sealed class MyStrategy : IStrategy
    {
        private enum Ownership
        {
            ANY,
            ALLY,
            ENEMY
        }

        private Random _random;
        private TerrainType[][] _terrainTypeByCellXY;
        private WeatherType[][] _weatherTypeByCellXY;

        private Player _me;
        private World _world;
        private Game _game;
        private Move _move;

        private IDictionary<long, Vehicle> _vehicleById = new Dictionary<long, Vehicle>();
        private IDictionary<long, int> _updateTickByVehicleId = new Dictionary<long, int>();
        private Queue<Action<Move>> _delayedMoves = new Queue<Action<Move>>();

        /// <summary>
        /// �������� ����� ���������, �������������� ���������� ������. ���������� ������ ���.
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

            Move();

            ExecuteDelayedMove();
        }

        /// <summary>
        /// ������������� ���������.
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
            }
        }

        /// <summary>
        /// ��������� ��� ������� ������ � ����� ������ ��� ��������� ������� � ���, � ����� ������������� �������� � ������
        /// ������� � ������� ���������� ��������� � ���������.
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
        /// ������ ���������� �������� �� ������� � ��������� ���.
        /// </summary>
        /// <returns>���������� true, ���� � ������ ���� ���������� �������� ���� ������� � ���������.</returns>
        private bool ExecuteDelayedMove()
        {
            if (!_delayedMoves.Any()) return false;

            var delayedMove = _delayedMoves.Dequeue();
            delayedMove.Invoke(_move);

            return true;
        }

        /// <summary>
        /// �������� ������ ���������
        /// </summary>
        private void Move()
        {
            // ������ 300 ����� ...
            if (_world.TickIndex % 300 == 0)
            {
                // ... ��� ������� ���� ������� ...
                var vehicleTypes = Enum.GetValues(typeof(VehicleType)).Cast<VehicleType>();
                foreach (var vehicleType in vehicleTypes)
                {
                    var targetType = GetPreferredTargetType(vehicleType);
                    // ... ���� ���� ��� ����� ��������� ...
                    if (targetType == null)
                    {
                        continue;
                    }

                    // ... �������� ����� �������� ...
                    var vehicles = GetVehicles(Ownership.ALLY, vehicleType);
                    var x = vehicles.Any() ? vehicles.Select(v => v.X).Average() : Double.NaN;
                    var y = vehicles.Any() ? vehicles.Select(v => v.Y).Average() : Double.NaN;

                    // ... �������� ����� �������� ���������� ��� ����� ���� ...
                    var targetTypeEnemyVehicles = GetVehicles(Ownership.ENEMY, targetType);
                    double targetX, targetY;
                    if (targetTypeEnemyVehicles.Any())
                    {
                        targetX = targetTypeEnemyVehicles.Select(v => v.X).Average();
                        targetY = targetTypeEnemyVehicles.Select(v => v.Y).Average();
                    }
                    else
                    {
                        var enemyVehicles = GetVehicles(Ownership.ENEMY);
                        targetX = enemyVehicles.Any() ? enemyVehicles.Select(v => v.X).Average() : Double.NaN;
                        targetY = enemyVehicles.Any() ? enemyVehicles.Select(v => v.Y).Average() : Double.NaN;
                    }

                    // .. � ��������� � ������� ���������� �������� ��� ��������� � ����������� �������.
                    if (!Double.IsNaN(x) && !Double.IsNaN(y))
                    {
                        _delayedMoves.Enqueue(move =>
                        {
                            move.Action = ActionType.ClearAndSelect;
                            move.Right = _world.Width;
                            move.Bottom = _world.Height;
                            move.VehicleType = vehicleType;
                        });

                        _delayedMoves.Enqueue(move =>
                        {
                            move.Action = ActionType.Move;
                            move.X = targetX - x;
                            move.Y = targetY - y;
                        });
                    }
                }

                // ����� ������� ����� �������� ����� ���� ...

                var arrvVehicles = GetVehicles(Ownership.ALLY, VehicleType.Arrv);
                var arrvX = arrvVehicles.Any() ? arrvVehicles.Select(v => v.X).Average() : Double.NaN;
                var arrvY = arrvVehicles.Any() ? arrvVehicles.Select(v => v.Y).Average() : Double.NaN;

                // .. � ���������� �� � ����� ����.
                if (!Double.IsNaN(arrvX) && !Double.IsNaN(arrvY))
                {
                    _delayedMoves.Enqueue(
                        move =>
                        {
                            move.Action = ActionType.ClearAndSelect;
                            move.Right = _world.Width;
                            move.Bottom = _world.Height;
                            move.VehicleType = VehicleType.Arrv;
                        });

                    _delayedMoves.Enqueue(
                        move =>
                        {
                            move.Action = ActionType.Move;
                            move.X = _world.Width / 2.0D - arrvX;
                            move.Y = _world.Height / 2.0D - arrvY;
                        });
                }

                return;
            }

            var allMyVehicles = GetVehicles(Ownership.ALLY);
            // ���� �� ���� ��� ���� �� ��� ��������� � ������� 60 ����� ...
            if (allMyVehicles.All(v => _world.TickIndex - _updateTickByVehicleId[v.Id] > 60))
            {
                // ... ������� ����� ����� �������� ...
                var x = allMyVehicles.Any() ? allMyVehicles.Select(v => v.X).Average() : Double.NaN;
                var y = allMyVehicles.Any() ? allMyVehicles.Select(v => v.Y).Average() : Double.NaN;

                // ... � ������������ � �� ��������� ����.
                if (!Double.IsNaN(x) && !Double.IsNaN(y))
                {
                    _move.Action = ActionType.Rotate;
                    _move.X = x;
                    _move.Y = y;
                    _move.Angle = _random.NextDouble() > 0.5 ? Math.PI : -Math.PI;
                }
            }

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


        /// <summary>
        /// ��������������� �����, ����������� ��� ���������� ���� ������� �������� ������ ��� �������, �����, ��� ������
        /// �������� ���������� ������ �������.
        /// </summary>
        /// <param name="vehicleType"></param>
        /// <returns></returns>
        private static VehicleType? GetPreferredTargetType(VehicleType vehicleType)
        {
            switch (vehicleType)
            {
                case VehicleType.Fighter:
                    return VehicleType.Helicopter;
                case VehicleType.Helicopter:
                    return VehicleType.Tank;
                case VehicleType.Ifv:
                    return VehicleType.Helicopter;
                case VehicleType.Tank:
                    return VehicleType.Ifv;
                default:
                    return null;
            }
        }
    }
}