package frc.lib.io.intakearm;

import frc.lib.hardware.IntakeArmHardware;
import frc.lib.math.GearRatio;

/**
 * Simplistic simulation of IntakeArmIO.
 * Instantly moves to commanded positions with no physics.
 */
public class IntakeArmIOSim implements IntakeArmIO {

    private final GearRatio ratio = IntakeArmHardware.RATIO;

    // Internal motor state (motor rotations)
    private double bluMotorRot = 0.0;
    private double yelMotorRot = 0.0;

    // Power mode (optional)
    private double appliedPower = 0.0;
    private boolean powerMode = false;

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        // Publish current state directly
        inputs.bluMotorRot = bluMotorRot;
        inputs.yelMotorRot = yelMotorRot;

        inputs.bluPositionDeg = ratio.sensorToReal(bluMotorRot);
        inputs.yelPositionDeg = ratio.sensorToReal(yelMotorRot);

        // Fake values
        inputs.bluCurrentA = 0.0;
        inputs.yelCurrentA = 0.0;
        inputs.bluTempC = 25.0;
        inputs.yelTempC = 25.0;

        // No limit switches in simple sim
        inputs.bluReverseLimit = false;
        inputs.yelReverseLimit = false;
    }

    @Override
    public void setAngle(double degrees) {
        powerMode = false;

        // Instantly move to the target
        double motorRot = ratio.realToSensor(degrees);
        bluMotorRot = motorRot;
        yelMotorRot = motorRot;
    }

    @Override
    public void runPower(double power) {
        powerMode = true;
        appliedPower = power;

        // Optional: nudge position slightly for realism
        bluMotorRot += appliedPower * 0.01;
        yelMotorRot = bluMotorRot;
    }

    @Override
    public void stop() {
        powerMode = false;
        appliedPower = 0.0;
    }
}
