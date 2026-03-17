//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.intakearm.*;

import frc.robot.Constants.kIntake.kArm;
import frc.robot.Constants.kIntake.kArm.kStall;

/**
 * Thin, IO-based subsystem for the Intake Arm mechanism.
 *
 * <p>This subsystem:
 * <ul>
 *   <li>Reads raw sensor data from the IO layer</li>
 *   <li>Provides mechanism-space accessors (angles, limits, etc.)</li>
 *   <li>Implements stall detection using velocity, voltage, and timing</li>
 *   <li>Exposes simple control methods for commands (setAngle, runPower, stop)</li>
 * </ul>
 *
 * <p>All hardware interaction is delegated to {@link IntakeArmIO}, keeping this
 * subsystem deterministic, testable, and free of hardware-specific logic.
 */
public class IntakeArmSubsystem extends SubsystemBase {

    private final IntakeArmIO io;
    private final IntakeArmInputs inputs;

    // Stall detection state
    private double stallStartTime = -1.0;
    private boolean stalled = false;

    /**
     * Creates a new IntakeArmSubsystem using the provided IO implementation.
     *
     * @param io the IO layer responsible for hardware interaction
     */
    public IntakeArmSubsystem(IntakeArmIO io) {
        this.io = io;
        this.inputs = new IntakeArmInputs();
    }

    /**
     * Periodic update loop.
     *
     * <p>This method:
     * <ul>
     *   <li>Refreshes all sensor inputs from the IO layer</li>
     *   <li>Evaluates stall conditions based on velocity and applied voltage</li>
     *   <li>Debounces stall detection using a time threshold</li>
     * </ul>
     *
     * <p>Stall detection logic:
     * <ul>
     *   <li>Low velocity → mechanism is not moving</li>
     *   <li>High effort → motors are trying to move</li>
     *   <li>Both conditions must persist for {@code kStall.TIME_SEC}</li>
     * </ul>
     */
    @Override
    public void periodic() {
        io.updateInputs(inputs);

        boolean lowVelocity =
            Math.abs(inputs.bluVelocityDegPerSec) < kStall.VELOCITY_THRESHOLD &&
            Math.abs(inputs.yelVelocityDegPerSec) < kStall.VELOCITY_THRESHOLD;

        boolean highEffort =
            Math.abs(inputs.bluAppliedVolts) > kStall.EFFORT_THRESHOLD ||
            Math.abs(inputs.yelAppliedVolts) > kStall.EFFORT_THRESHOLD;

        boolean stallCondition = lowVelocity && highEffort;

        double now = Timer.getFPGATimestamp();

        if (stallCondition) {
            // Start timing if this is the first stall frame
            if (stallStartTime < 0) {
                stallStartTime = now;
            }
            // Stall is valid only if it persists long enough
            stalled = (now - stallStartTime) >= kStall.TIME_SEC;
        } else {
            // Reset stall state when conditions clear
            stallStartTime = -1.0;
            stalled = false;
        }
    }

    /**
     * Commands the arm to move to a target angle in mechanism degrees.
     *
     * <p>The angle is clamped to {@link kArm#MAX_ANGLE_DEGREES} to prevent
     * overextension.
     *
     * @param degrees desired arm angle in mechanism degrees
     */
    public void setAngle(double degrees) {
        double safeDeg = Math.min(degrees, kArm.MAX_ANGLE_DEGREES);
        io.setAngle(safeDeg);
    }

    /**
     * @return the latest cached sensor inputs for the intake arm
     */
    public IntakeArmInputs getInputs() {
        return inputs;
    }

    /** @return BLU motor mechanism-space position in degrees */
    public double getBLUPositionDegrees() {
        return inputs.bluPositionDeg;
    }

    /** @return YEL motor mechanism-space position in degrees */
    public double getYELPositionDegrees() {
        return inputs.yelPositionDeg;
    }

    /** @return true if the BLU motor's reverse limit switch is pressed */
    public boolean getBLUReverseLimit() {
        return inputs.bluReverseLimit;
    }

    /** @return true if the YEL motor's reverse limit switch is pressed */
    public boolean getYELReverseLimit() {
        return inputs.yelReverseLimit;
    }

    /**
     * Checks whether both motors are within a tolerance of a target angle.
     *
     * @param targetDeg    desired angle in degrees
     * @param toleranceDeg allowable error in degrees
     * @return true if both motors are within tolerance
     */
    public boolean isAtPosition(double targetDeg, double toleranceDeg) {
        return Math.abs(inputs.bluPositionDeg - targetDeg) < toleranceDeg &&
               Math.abs(inputs.yelPositionDeg - targetDeg) < toleranceDeg;
    }

    /**
     * Indicates whether the subsystem has detected a mechanical stall.
     *
     * <p>A stall occurs when:
     * <ul>
     *   <li>Velocity is below {@link kStall#VELOCITY_THRESHOLD}</li>
     *   <li>Applied voltage exceeds {@link kStall#EFFORT_THRESHOLD}</li>
     *   <li>These conditions persist for {@link kStall#TIME_SEC}</li>
     * </ul>
     *
     * @return true if the arm is currently stalled
     */
    public boolean isStalled() {
        return stalled;
    }

    /**
     * Runs the arm motors in open-loop power mode.
     *
     * <p>Includes safety checks:
     * <ul>
     *   <li>Prevents upward movement past {@link kArm#MAX_ANGLE_DEGREES}</li>
     *   <li>Maps joystick power to fixed safe power levels</li>
     * </ul>
     *
     * @param power requested power (-1.0 to 1.0)
     */
    public void runPower(double power) {
        double safePower = power;

        if ((inputs.bluPositionDeg > kArm.MAX_ANGLE_DEGREES) ||
            (inputs.yelPositionDeg > kArm.MAX_ANGLE_DEGREES) && power > 0) {
            safePower = 0;
        }

        if (safePower > 0.5) {
            io.runPower(kArm.MANUAL_POSITIVE_POWER);
        } else if (safePower < -0.5) {
            io.runPower(kArm.MANUAL_NEGATIVE_POWER);
        } else {
            io.runPower(0);
        }
    }

    /**
     * Immediately stops all arm motor output.
     */
    public void stop() {
        io.stop();
    }
}
