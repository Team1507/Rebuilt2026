//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.swerve.SwerveIO;
import frc.lib.io.swerve.SwerveInputs;

/**
 * Thin, IO-based swerve subsystem.
 *
 * <p>This subsystem contains no vendor-specific code. All hardware
 * interaction is delegated to the SwerveIO layer (SwerveIOReal or
 * SwerveIOSim). This keeps the subsystem clean, testable, and
 * consistent with the architecture used for the shooter.
 */
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveIO io;
    private final SwerveInputs inputs = new SwerveInputs();

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        // Optional: publish telemetry here
        // Telemetry.log("Swerve/Pose", inputs.pose);
        // Telemetry.log("Swerve/Speeds", inputs.speeds);
    }

    // ==========================================================
    // Control API
    // ==========================================================

    /** Drive the robot using chassis speeds (vx, vy, omega). */
    public void drive(ChassisSpeeds speeds) {
        io.drive(speeds);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        io.driveRobotRelative(speeds);
    }

    /** Directly set module states (used by autos / PathPlanner). */
    public void setModuleStates(SwerveModuleState[] states) {
        io.setModuleStates(states);
    }

    /** Stop all motion. */
    public void stop() {
        io.stop();
    }

    // ==========================================================
    // Pose + Odometry
    // ==========================================================

    /** Get the current estimated robot pose. */
    public Pose2d getPose() {
        return inputs.pose;
    }

    /** Get the current heading of the gyro */
    public Rotation2d getHeading() {
        return io.getHeading();
    }

    /** Reset the robot pose. */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Adds a vision measurement (delegates to IO). */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void seedPoseFromVision(Pose2d visionPose) {
        // Reset pose estimator
        io.resetPose(visionPose);

        // Align gyro to match the vision heading
        io.alignGyro(visionPose.getRotation());
    }

    // ==========================================================
    // Access to raw inputs (optional)
    // ==========================================================

    public SwerveInputs getInputs() {
        return inputs;
    }
}
