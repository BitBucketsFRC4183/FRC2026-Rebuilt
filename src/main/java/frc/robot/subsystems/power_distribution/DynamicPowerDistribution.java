package frc.robot.subsystems.power_distribution;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.hopper.HopperIOTalonFX;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;

public class DynamicPowerDistribution extends SubsystemBase {
    private final int driveState = 1;
    private final int actionState = 2;
    private int currentState = driveState;

    private final int voltageMax = 8;

    public void periodic(ShooterIOTalonFX shooter, HopperIOTalonFX hopper, IntakeIOTalonFX intake, ModuleIOTalonFX drive) {
        if(shooter.flywheelMotor.getMotorVoltage().getValueAsDouble() > 0
                || hopper.conveyorMotor.getMotorVoltage().getValueAsDouble() > 0
                || intake.intakeMotor.getMotorVoltage().getValueAsDouble() > 0

        ) {currentState = actionState;}
        else {currentState = driveState;}

        if(currentState == actionState) {
            drive.driveTalon.setVoltage(Math.min(drive.driveTalon.getMotorVoltage().getValueAsDouble(), voltageMax));
        }
    }
}
