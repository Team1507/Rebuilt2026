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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;

import frc.lib.io.swerve.SwerveIO;
import frc.lib.io.swerve.SwerveInputs;
import frc.robot.framework.base.Subsystems1507;
/**
 * Thin, IO-based swerve subsystem.
 *
 * <p>This subsystem contains no vendor-specific code. All hardware
 * interaction is delegated to the SwerveIO layer (SwerveIOReal or
 * SwerveIOSim). This keeps the subsystem clean, testable, and
 * consistent with the architecture used for the shooter.
 */
public class SwerveSubsystem extends Subsystems1507 {

    private final SwerveIO io;
    private final SwerveInputs inputs;

    public SwerveSubsystem(SwerveIO io) {
        this.io = io;
        this.inputs = new SwerveInputs();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
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

    /** Changes the position of the swerve modules to lock the robot in place */
    public void lock() {
        io.setControl(new SwerveDriveBrake());
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

    /** Reset the robot pose (odometry only). */
    public void resetPose(Pose2d pose) {
        io.resetPose(pose);
    }

    /** Resets the pose estimator and vision heading history. */
    public void resetLocalization(Pose2d pose) {
        io.resetPose(pose);
    }

    public void setTemporaryTargetPose(Pose2d tempPose) {
        inputs.tempPose = tempPose;
    }

    public Pose2d getTemporaryTargetPose() {
        return inputs.tempPose;
    }

    /** Adds a vision measurement (delegates to IO). */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        io.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    public void setPose(Pose2d pose) {
        // Reset pose estimator
        io.resetPose(pose);

        // Align gyro to match the vision heading
        io.alignGyro(pose.getRotation());
    }

    /** Returns the current robot-relative chassis speeds. */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return inputs.speeds;
    }

    /** Returns the current field-relative chassis speeds. */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(
            inputs.speeds,
            getHeading()
        );
    }

    // ==========================================================
    // Access to raw inputs (optional)
    // ==========================================================

    public SwerveInputs getInputs() {
        return inputs;
    }
}
