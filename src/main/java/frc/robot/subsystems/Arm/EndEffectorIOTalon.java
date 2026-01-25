package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climber.ElevatorIO;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOTalon implements EndEffectorIO {
    private final TalonFX endTalon = new TalonFX(1);
    boolean isOpen = false;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    public StatusSignal<Angle> endPosition;
    public StatusSignal<AngularVelocity> endVelocity;
    public StatusSignal<Current> endCurrent;
    public StatusSignal<Voltage> endAppliedVolts;

    private final Debouncer endConnectedDebounce = new Debouncer(0.5);

    EndEffectorIOTalon() {
        TalonFXConfiguration endConfig = new TalonFXConfiguration();
        endConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        endConfig.CurrentLimits.StatorCurrentLimit = 90;
        endConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        endConfig.CurrentLimits.SupplyCurrentLimit = 40;
        endConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        //Configures the motors to have limits.
        endTalon.getConfigurator().apply(endConfig);

        endPosition = endTalon.getPosition();
        endVelocity = endTalon.getVelocity();
        endCurrent = endTalon.getStatorCurrent();
        endAppliedVolts = endTalon.getMotorVoltage();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                endPosition,
                endVelocity,
                endAppliedVolts,
                endCurrent);
        ParentDevice.optimizeBusUtilizationForAll(endTalon);
        PhoenixUtil.tryUntilOk(5, () -> endTalon.setPosition(0));
    }

    public void updateInputs(ElevatorIO.ElevatorIOInputs inputs) {
        var elevatorStatus =
                BaseStatusSignal.refreshAll(endPosition, endVelocity, endAppliedVolts, endCurrent);
        inputs.elevatorConnected = endConnectedDebounce.calculate(elevatorStatus.isOK());
        inputs.elevatorMotorPositionRad = Units.rotationsToRadians(endPosition.getValueAsDouble());
        inputs.elevatorMotorVelocityRadPerSec = Units.rotationsToRadians(endVelocity.getValueAsDouble());
        inputs.elevatorAppliedVolts = endAppliedVolts.getValueAsDouble();
        inputs.elevatorCurrentAmps = endCurrent.getValueAsDouble();
    }
        //ASK WTH THIS IS I JUST GOT IT FROM ADVANTAGE KIT AND WAS TOLD IT WAS NEEDED
        @Override
        public void setIsOpen (boolean setting){
        this.isOpen = setting;
        }
        @Override
        public void setEndVoltage (double volts){
            double appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
            endTalon.setControl(voltageRequest.withOutput(appliedVolts));
        }
        @Override
        public  void setEndVelocity (double velocity){
            endTalon.set(velocity);
        }
        @Override
        public void disable () {
            endTalon.disable();
        }
    }
