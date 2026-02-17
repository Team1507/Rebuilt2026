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

    public double getBLUPositionDegrees() {
        return inputs.bluPositionDeg;
    }

    public double getYELPositionDegrees() {
        return inputs.yelPositionDeg;
    }

    public boolean isAtPosition(double targetDeg, double toleranceDeg) {
        return Math.abs(inputs.bluPositionDeg - targetDeg) < toleranceDeg &&
               Math.abs(inputs.yelPositionDeg - targetDeg) < toleranceDeg;
    }

    public void stop() {
        io.stop();
    }
}
