using System.Collections.Generic;
using System.Globalization;
using Sandbox.ModAPI.Ingame;
using SpaceEngineers.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace SpaceEngineers.GraviEngine
{
    public class Program : MyGridProgram
    {
        //COPY START
        private const double GravityGeneratorAccelerationMultiplier = 9.81;
        private const double DefaultConstantAccelerationTargetValue = 1;
        private const int GravityGeneratorMaxFieldSize = 150;

        private IMyShipController _controller;
        private readonly List<IMyGravityGenerator> _gravityGenerators;
        private readonly List<IMyArtificialMassBlock> _artificialMassBlocks;
        private readonly List<IMyGyro> _gyroscopes;

        private readonly Dictionary<IMyGravityGenerator, List<IMyArtificialMassBlock>> _generatorsFieldMap;

        private EngineState _engineState;
        private double _targetAcceleration;
        private double _shipMass;

        public Program()
        {
            _engineState = EngineState.Initializing;
            _targetAcceleration = DefaultConstantAccelerationTargetValue;
            _gravityGenerators = new List<IMyGravityGenerator>();
            _artificialMassBlocks = new List<IMyArtificialMassBlock>();
            _gyroscopes = new List<IMyGyro>();
            _generatorsFieldMap = new Dictionary<IMyGravityGenerator, List<IMyArtificialMassBlock>>();
        }

        public void Main(string argument, UpdateType updateSource)
        {
            var argParts = argument.Split(' ');
            var command = argParts.Length > 0 ? argParts[0] : "";
            double.TryParse(argParts[1], NumberStyles.Float, CultureInfo.CurrentCulture, out var commandArgument);
            switch (updateSource)
            {
                case UpdateType.Terminal:
                case UpdateType.Trigger:
                    switch (command)
                    {
                        case "Start":
                            Start();
                            break;
                        case "Stop":
                            Stop();
                            break;
                        case "SetAcceleration":
                            SetAcceleration(commandArgument);
                            break;
                    }

                    break;
                case UpdateType.Update1:
                    Update1();
                    break;
                default:
                    return;
            }
        }

        private void Start()
        {
            _engineState = EngineState.Collecting;
            var tempList = new List<IMyTerminalBlock>();
            if (CollectBlocks(tempList))
            {
                _engineState = EngineState.Tuning;
                TuneEngine(tempList);
                Runtime.UpdateFrequency = UpdateFrequency.Update1;
                _engineState = EngineState.Working;
            }
            else
            {
                Echo("Unable to start engine, critical part(s) is missing\n");
            }
        }

        private bool CollectBlocks(List<IMyTerminalBlock> tempList)
        {
            _controller = GetShipController(tempList);
            if (_controller == null)
            {
                return false;
            }

            var success = true;
            success &= CollectGravityGenerators(tempList);
            success &= CollectArtificialMassBlocks(tempList);
            success &= CollectGyroscopes(tempList);
            return success;
        }

        private IMyShipController GetShipController(List<IMyTerminalBlock> tempList)
        {
            GridTerminalSystem.GetBlocksOfType<IMyShipController>(tempList);
            if (tempList.Count > 1)
            {
                foreach (var terminalBlock in tempList)
                {
                    if (terminalBlock is IMyShipController shipController && shipController.IsMainCockpit)
                    {
                        return shipController;
                    }
                }
            }
            else
            {
                return tempList[0] as IMyShipController;
            }

            Echo("Error: ship controller not found\n");

            return null;
        }

        private bool CollectGravityGenerators(List<IMyTerminalBlock> tempList)
        {
            GridTerminalSystem.GetBlocksOfType<IMyGravityGenerator>(tempList, FilterGridBlock);
            var gravityAxes = new Vector3I(0);
            foreach (var terminalBlock in tempList)
            {
                _gravityGenerators.Add(terminalBlock as IMyGravityGenerator);
                gravityAxes += GetRelativeBlockOrientation(terminalBlock, _controller);
            }

            if (gravityAxes.X == 0)
            {
                Echo("Error: not found gravity generators for X axis\n");
            }

            if (gravityAxes.Y == 0)
            {
                Echo("Error: not found gravity generators for Y axis\n");
            }

            if (gravityAxes.Z == 0)
            {
                Echo("Error: not found gravity generators for Z axis\n");
            }

            return gravityAxes.Volume() > 0;
        }

        private bool CollectArtificialMassBlocks(List<IMyTerminalBlock> tempList)
        {
            GridTerminalSystem.GetBlocksOfType<IMyArtificialMassBlock>(tempList, FilterGridBlock);
            var gravityAxes = Vector3I.Zero;
            foreach (var artificialMass in tempList)
            {
                var inBounds = false;
                foreach (var gravityGenerator in _gravityGenerators)
                {
                    if (IsBlockInPotentialGravityField(artificialMass, gravityGenerator))
                    {
                        if(!_generatorsFieldMap.ContainsKey(gravityGenerator))
                            _generatorsFieldMap.Add(gravityGenerator, new List<IMyArtificialMassBlock>());
                        _generatorsFieldMap[gravityGenerator].Add(artificialMass as IMyArtificialMassBlock);
                        gravityAxes += GetRelativeBlockOrientation(gravityGenerator, _controller);
                        inBounds = true;
                    }
                }

                if (inBounds)
                {
                    _artificialMassBlocks.Add(artificialMass as IMyArtificialMassBlock);
                }
                else
                {
                    Echo("Warning: artificial mass block with name '" + artificialMass.CustomName +
                         "' is not in bounds of any gravity generator\n");
                }
            }

            if (gravityAxes.X == 0)
            {
                Echo("Error: not found artificial mass blocks for X axis\n");
            }

            if (gravityAxes.Y == 0)
            {
                Echo("Error: not found artificial mass blocks for Y axis\n");
            }

            if (gravityAxes.Z == 0)
            {
                Echo("Error: not found artificial mass blocks for Z axis\n");
            }

            return gravityAxes.Volume() > 0;
        }

        private bool CollectGyroscopes(List<IMyTerminalBlock> tempList)
        {
            GridTerminalSystem.GetBlocksOfType(_gyroscopes, FilterGridBlock);
            if (_gyroscopes.Count < 1)
            {
                Echo("Error: gyroscopes not found\n");
            }

            return _gyroscopes.Count > 0;
        }

        private bool FilterGridBlock(IMyTerminalBlock terminalBlock)
        {
            return terminalBlock.IsFunctional && _controller.IsSameConstructAs(terminalBlock);
        }

        private void TuneEngine(List<IMyTerminalBlock> tempList)
        {
            tempList.Clear();
            foreach (var generator in _gravityGenerators)
            {
                var fieldSize = Vector3.Zero;
                foreach (var artificialMass in _artificialMassBlocks)
                {
                    var distance = Vector3I.Abs(artificialMass.Position - generator.Position);
                    if (distance.AbsMax() > GravityGeneratorMaxFieldSize / 2) continue;
                    if (distance.X > fieldSize.X)
                    {
                        fieldSize.X = distance.X;
                    }

                    if (distance.Y > fieldSize.Y)
                    {
                        fieldSize.Y = distance.Y;
                    }

                    if (distance.Y > fieldSize.Y)
                    {
                        fieldSize.Y = distance.Y;
                    }
                }

                if (fieldSize != Vector3.Zero)
                {
                    generator.FieldSize = fieldSize;
                }
                else
                {
                    Echo("Warning: gravity generator with name '" + generator.CustomName +
                         "' has no artificial mass blocks in his gravity field range\n");
                    tempList.Add(generator);
                }
            }

            foreach (var block in tempList)
            {
                _gravityGenerators.Remove(block as IMyGravityGenerator);
            }
        }

        private void Stop()
        {
            SetGravityEngineEnabled(false);
            Runtime.UpdateFrequency = UpdateFrequency.None;
            _engineState = EngineState.Stopped;
        }

        private void SetAcceleration(double acceleration)
        {
            _targetAcceleration = acceleration;
        }

        private void Update1()
        {
            if (_engineState != EngineState.Working) return;
            var force = _controller.MoveIndicator.AbsMax() > float.Epsilon ? GetControllerWantedForce() :
                _controller.DampenersOverride ? GetDampenersForce() : Vector3D.Zero;
            
            SetGravityEngineEnabled(force.LengthSquared() > double.Epsilon);
            ApplyGravityEngineForce(force);
        }

        private Vector3D GetControllerWantedForce()
        {
        }

        private Vector3D GetDampenersForce()
        {
        }

        private void ApplyGravityEngineForce(Vector3D force)
        {
            foreach (var generator in _gravityGenerators)
            {
                var orientation = new Vector3D(GetRelativeBlockOrientation(generator, _controller));
                var axisForce = force.Dot(orientation);
                SetGravityGeneratorForce(generator, axisForce);
                CompensateRotation(generator, orientation, axisForce);
            }
        }

        private void SetGravityEngineEnabled(bool enabled)
        {
            SetBlocksEnabled(_gravityGenerators, enabled);
            SetBlocksEnabled(_artificialMassBlocks, enabled);
        }

        private void CompensateRotation(IMyGravityGenerator generator, Vector3D orientation, double axisForce)
        {
            var artificialMasses = _generatorsFieldMap[generator];
            
        }
        
        private static void SetGravityGeneratorForce(IMyGravityGeneratorBase generator, double force)
        {
            generator.GravityAcceleration = (float) (force * GravityGeneratorAccelerationMultiplier);
        }

        private static Vector3I GetRelativeBlockOrientation(IMyCubeBlock block, IMyCubeBlock relativeTo)
        {
            if (block.Orientation.Up == relativeTo.Orientation.Forward) return Vector3I.Forward;
            if (block.Orientation.Up == Base6Directions.GetOppositeDirection(relativeTo.Orientation.Forward))
                return Vector3I.Backward;
            if (block.Orientation.Up == relativeTo.Orientation.Left) return Vector3I.Left;
            if (block.Orientation.Up == Base6Directions.GetOppositeDirection(relativeTo.Orientation.Left))
                return Vector3I.Right;
            return block.Orientation.Up == relativeTo.Orientation.Up ? Vector3I.Up : Vector3I.Down;
        }

        private static bool IsBlockInPotentialGravityField(IMyCubeBlock block, IMyCubeBlock gravityGenerator)
        {
            var min = gravityGenerator.Position - GravityGeneratorMaxFieldSize / 2;
            var max = gravityGenerator.Position + GravityGeneratorMaxFieldSize / 2;
            return block.Position.IsInsideInclusiveEnd(ref min, ref max);
        }

        private static void SetBlocksEnabled(IEnumerable<IMyTerminalBlock> blocks, bool enabled)
        {
            foreach (var block in blocks)
            {
                SetBlockEnabled(block, enabled);
            }
        }

        private static void SetBlockEnabled(IMyTerminalBlock block, bool enabled)
        {
            block.ApplyAction(enabled ? "OnOff_On" : "OnOff_Off");
        }

        private enum EngineState
        {
            Initializing,
            Collecting,
            Tuning,
            Working,
            Stopped
        }
    }
}