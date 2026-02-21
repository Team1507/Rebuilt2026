//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.util.CommandBuilder;

import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import frc.robot.Constants.kMoveThroughPose;
import frc.robot.Constants.kMoveToPose;

public final class DriveCommands {

    private DriveCommands() {}

    // ==========================================================
    // Maintain Heading to Target
    // ==========================================================
    public static Command maintainHeadingToTarget(
        SwerveSubsystem swerve,
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {
        PIDController headingController = new PIDController(6.0, 0.0, 0.1);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return new CommandBuilder(swerve)
            .named("MaintainHeadingToTarget")
            .onExecute(() -> {
                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                double rotRate = headingController.calculate(
                    swerve.getHeading().getRadians(),
                    desiredHeading.getRadians()
                );

                swerve.drive(new ChassisSpeeds(
                    xSupplier.get(),
                    ySupplier.get(),
                    rotRate
                ));
            })
            .onEnd(swerve::stop);
    }

    public static Command pointToTarget(
        SwerveSubsystem swerve,
        Supplier<Pose2d> targetPoseSupplier
    ) {
        PIDController headingController = new PIDController(6.0, 0.0, 0.1);
        headingController.enableContinuousInput(-Math.PI, Math.PI);

        return new CommandBuilder(swerve)
            .named("MaintainHeadingToTarget")
            .onExecute(() -> {
                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                double rotRate = headingController.calculate(
                    swerve.getHeading().getRadians(),
                    desiredHeading.getRadians()
                );

                swerve.drive(new ChassisSpeeds(
                    0.0,
                    0.0,
                    rotRate
                ));
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                double angleError = Math.abs(
                    current.getRotation().minus(desiredHeading).getRadians()
                );

                return angleError < Math.toRadians(20.0); // 5 degree tolerance
            })
            
            .onEnd(swerve::stop);
    }

    private static Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    // ==========================================================
    // Move Through Pose (no slowdown)
    // ==========================================================
    public static Command moveThroughPose(
        SwerveSubsystem swerve,
        Pose2d targetPose,
        double maxSpeed,
        double maxAngularSpeed,
        double passRadius
    ) {
        PIDController thetaPID = new PIDController(
            kMoveThroughPose.THETA_KP,
            kMoveThroughPose.THETA_KI,
            kMoveThroughPose.THETA_KD
        );
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        final double[] last = new double[3]; // x, y, time

        return new CommandBuilder(swerve)
            .named("MoveThroughPose_Vector")
            .onInitialize(() -> {
                Pose2d p = swerve.getPose();
                last[0] = p.getX();
                last[1] = p.getY();
                last[2] = Timer.getFPGATimestamp();
                thetaPID.reset();
            })
            .onExecute(() -> {

                Pose2d current = swerve.getPose();

                // -----------------------------------------
                // 1. Compute direction vector to target
                // -----------------------------------------
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                // Unit direction vector
                double dirX = dx / (distance + 1e-9);
                double dirY = dy / (distance + 1e-9);

                // -----------------------------------------
                // 2. Constant-speed translation (NO slowdown)
                // -----------------------------------------
                double vx_field = dirX * maxSpeed;
                double vy_field = dirY * maxSpeed;

                // -----------------------------------------
                // 3. Rotation PID
                // -----------------------------------------
                double thetaSpeed = thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );
                thetaSpeed = MathUtil.clamp(thetaSpeed, -maxAngularSpeed, maxAngularSpeed);

                // -----------------------------------------
                // 4. Convert to robot-relative
                // -----------------------------------------
                ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                    vx_field,
                    vy_field,
                    thetaSpeed,
                    current.getRotation()
                );

                swerve.driveRobotRelative(robotRelative);
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();
                double now = Timer.getFPGATimestamp();

                // -----------------------------------------
                // 1. Pass-through condition
                // -----------------------------------------
                if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                    return true;

                // -----------------------------------------
                // 2. Stall detection
                // -----------------------------------------
                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                boolean stalled =
                    dx < kMoveThroughPose.STALL_THRESHOLD &&
                    dy < kMoveThroughPose.STALL_THRESHOLD &&
                    (now - last[2]) > kMoveThroughPose.STALL_TIMEOUT;

                if (stalled)
                    return true;

                // Update last movement if we ARE moving
                if (dx > kMoveThroughPose.STALL_THRESHOLD || dy > kMoveThroughPose.STALL_THRESHOLD) {
                    last[0] = current.getX();
                    last[1] = current.getY();
                    last[2] = now;
                }

                return false;
            })
            .onEnd(swerve::stop);
    }

    // ==========================================================
    // Move To Pose (full PID control)
    // ==========================================================
    public static Command moveToPose(
        SwerveSubsystem swerve,
        Pose2d targetPose,
        double maxSpeed,
        double maxAngularSpeed
    ) {

        PIDController distancePID = new PIDController(
            kMoveToPose.XY_KP,
            kMoveToPose.XY_KI,
            kMoveToPose.XY_KD
        );

        PIDController thetaPID = new PIDController(
            kMoveToPose.THETA_KP,
            kMoveToPose.THETA_KI,
            kMoveToPose.THETA_KD
        );
        thetaPID.enableContinuousInput(-Math.PI, Math.PI);

        return new CommandBuilder(swerve)
            .named("MoveToPose_VectorPID")
            .onInitialize(() -> {
                distancePID.reset();
                thetaPID.reset();
            })
            .onExecute(() -> {

                Pose2d current = swerve.getPose();

                // -----------------------------
                // 1. Vector to target
                // -----------------------------
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                double dirX = dx / (distance + 1e-9);
                double dirY = dy / (distance + 1e-9);

                // -----------------------------
                // 2. PID on distance (correct direction)
                // -----------------------------
                double pidOut = distancePID.calculate(0.0, distance);

                // PID output is allowed to be large; clamp later
                double speed = pidOut;

                // -----------------------------
                // 3. Slowdown curve (dominant)
                // -----------------------------
                if (distance < kMoveToPose.SLOWDOWN_START) {
                    double t = distance / kMoveToPose.SLOWDOWN_START;
                    double slowdown = MathUtil.clamp(
                        MathUtil.interpolate(1.0, kMoveToPose.SLOWDOWN_MIN, 1.0 - t),
                        kMoveToPose.SLOWDOWN_MIN,
                        1.0
                    );
                    speed *= slowdown;
                }

                // -----------------------------
                // 4. Clamp to max speed
                // -----------------------------
                speed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);

                // -----------------------------
                // 5. Apply MIN_SPEED floor
                // -----------------------------
                if (Math.abs(speed) < kMoveToPose.MIN_SPEED) {
                    speed = Math.copySign(kMoveToPose.MIN_SPEED, speed);
                }

                // -----------------------------
                // 6. Convert scalar → XY
                // -----------------------------
                double xSpeed = dirX * speed;
                double ySpeed = dirY * speed;

                // -----------------------------
                // 7. Rotation PID
                // -----------------------------
                double thetaSpeed = thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );
                thetaSpeed = MathUtil.clamp(thetaSpeed, -maxAngularSpeed, maxAngularSpeed);

                // -----------------------------
                // 8. Field → robot
                // -----------------------------
                ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    thetaSpeed,
                    current.getRotation()
                );

                swerve.driveRobotRelative(robotRelative);
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();

                boolean atPos =
                    current.getTranslation().getDistance(targetPose.getTranslation()) <
                    kMoveToPose.POSITION_TOLERANCE_METERS;

                boolean atAngle =
                    Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians()) <
                    kMoveToPose.ANGLE_TOLERANCE_RADIANS;

                return atPos && atAngle;
            })
            .onEnd(swerve::stop);
    }
}