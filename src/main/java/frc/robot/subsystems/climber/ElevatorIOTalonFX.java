package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

import java.util.Queue;

public class ElevatorIOTalonFX implements ElevatorIO{
    private final TalonFX elevatorTalon = new TalonFX(2);
    //idk put a random ID here

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);
    private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
            new PositionTorqueCurrentFOC(0.0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    public Queue<Double> timestampQueue;

    public static Queue<Double> elevatorPositionQueue;

    public StatusSignal<Angle> elevatorPosition;

    public StatusSignal<AngularVelocity> elevatorVelocity;
    public StatusSignal<Current> elevatorCurrent;

    public StatusSignal<Voltage> elevatorAppliedVolts;

    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOTalonFX() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        elevatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorConfig.CurrentLimits.StatorCurrentLimit = 60;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //Configures the motors to have limits.

        elevatorPosition = elevatorTalon.getPosition();
        elevatorVelocity = elevatorTalon.getVelocity();
        elevatorCurrent = elevatorTalon.getStatorCurrent();
        elevatorAppliedVolts = elevatorTalon.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                elevatorPosition,
                elevatorVelocity,
                elevatorAppliedVolts,
                elevatorCurrent);
        ParentDevice.optimizeBusUtilizationForAll(elevatorTalon);
        PhoenixUtil.tryUntilOk(5, () -> elevatorTalon.setPosition(0));
    }
        public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        var elevatorStatus =
                    BaseStatusSignal.refreshAll(elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);
            inputs.elevatorConnected = elevatorConnectedDebounce.calculate(elevatorStatus.isOK());
            inputs.elevatorMotorPositionRad = Units.rotationsToRadians(elevatorPosition.getValueAsDouble());
            inputs.elevatorMotorVelocityRadPerSec = Units.rotationsToRadians(elevatorVelocity.getValueAsDouble());
            inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValueAsDouble();
            inputs.elevatorCurrentAmps = elevatorCurrent.getValueAsDouble();
        }

        @Override
        public void setElevatorMotorVoltage(double volts) {
            double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
            elevatorTalon.set(appliedVolts);
        }

        @Override
        public void disable() {
            elevatorTalon.set(0);
        }
    }

