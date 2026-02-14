package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;

public class HopperIOSim implements HopperIO {

    private double conveyorPercent = 0.0;

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        // In sim, applied output is just the commanded percent
        inputs.conveyorAppliedOutput = conveyorPercent;
    }

    @Override
    public void setConveyorPercent(double percent) {
        conveyorPercent = MathUtil.clamp(percent, -1.0, 1.0);
    }

    @Override
    public void stopConveyor() {
        conveyorPercent = 0.0;
    }
}
