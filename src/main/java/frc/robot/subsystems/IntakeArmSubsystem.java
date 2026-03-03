//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.intakearm.IntakeArmIO;
import frc.lib.io.intakearm.IntakeArmInputs;
import frc.robot.Constants.kIntake;
/**
 * Thin, IO-based intake arm subsystem.
 */
public class IntakeArmSubsystem extends SubsystemBase {

    private final IntakeArmIO io;
    private final IntakeArmInputs inputs = new IntakeArmInputs();

    public IntakeArmSubsystem(IntakeArmIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPosition(double degrees) {
        io.setPositionDeg(degrees);
    }

    public IntakeArmInputs getInputs() {
        return inputs;
    }

    public double getBLUPositionDegrees() {
        return inputs.bluPositionDeg;
    }

    public double getYELPositionDegrees() {
        return inputs.yelPositionDeg;
    }

    public boolean getBLUReverseLimit() {
        return inputs.bluReverseLimit;
    }

    public boolean getYELReverseLimit() {
        return inputs.yelReverseLimit;
    }

    public boolean isAtPosition(double targetDeg, double toleranceDeg) {
        return Math.abs(inputs.bluPositionDeg - targetDeg) < toleranceDeg &&
               Math.abs(inputs.yelPositionDeg - targetDeg) < toleranceDeg;
    }
    public void runPower(double power) {
        if(power > 0.5) {
            io.runPower(kIntake.kArm.MANUAL_POSITIVE_POWER);
        }
        else if(power< -0.5){
            io.runPower(kIntake.kArm.MANUAL_NEGATIVE_POWER);
        }
        else {
            io.runPower(0);
        }
    }
    public void stop() {
        io.stop();
    }
}
