// //  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
// //  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
// //  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
// //  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
// //  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
// //   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
// //                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

import frc.lib.hardware.HopperHardware;
import frc.lib.math.GearRatio;
import frc.robot.Constants.kHopper;

/**
 * Simplistic simulation of HopperIO.
 * Instantly moves to commanded positions and reports hopperExtended.
 */
public class HopperIOSim implements HopperIO {

    private final GearRatio ratio = HopperHardware.RATIO;

    // Internal simulated state
    private double motorRot = 0.0;   // sensor units
    private double positionInches = 0.0;

    // Optional power mode
    private double appliedPower = 0.0;
    private boolean powerMode = false;

    @Override
    public void updateInputs(HopperInputs inputs) {
        // Publish current state
        inputs.motorRot = motorRot;
        inputs.position = positionInches;

        // Fake values
        inputs.currentA = 0.0;
        inputs.temperatureC = 25.0;

        // Hopper is extended if above threshold
        inputs.hopperExtended = positionInches > kHopper.SAFE_EXTENDED;
    }

    @Override
    public void setPosition(double degrees) {
        // In real code, "degrees" is actually inches.
        powerMode = false;

        double targetInches = degrees;
        motorRot = ratio.realToSensor(targetInches);
        positionInches = targetInches;
    }

    @Override
    public boolean getMagSensor() {
        // Always false in SIM unless you want to simulate homing
        return false;
    }

    @Override
    public void runPower(double power) {
        powerMode = true;
        appliedPower = power;

        // Optional: nudge position slightly for realism
        positionInches += appliedPower * 0.05;
        motorRot = ratio.realToSensor(positionInches);
    }

    @Override
    public void hopperStop() {
        powerMode = false;
        appliedPower = 0.0;
    }
}
