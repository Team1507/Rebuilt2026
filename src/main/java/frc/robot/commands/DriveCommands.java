//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        double passRadius
    ) {
        PIDController xController = new PIDController(kMoveThroughPose.X_KP, kMoveThroughPose.X_KI, kMoveThroughPose.X_KD);
        PIDController yController = new PIDController(kMoveThroughPose.Y_KP, kMoveThroughPose.Y_KI, kMoveThroughPose.Y_KD);

        final double[] last = new double[3];

        return new CommandBuilder(swerve)
            .named("MoveThroughPose")
            .onInitialize(() -> {
                Pose2d p = swerve.getPose();
                last[0] = p.getX();
                last[1] = p.getY();
                last[2] = Timer.getFPGATimestamp();
                xController.reset();
                yController.reset();
            })
            .onExecute(() -> {
                Pose2d current = swerve.getPose();

                double xSpeed = xController.calculate(current.getX(), targetPose.getX());
                double ySpeed = yController.calculate(current.getY(), targetPose.getY());

                xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
                ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), ySpeed);

                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, 0.0, current.getRotation()
                );

                swerve.drive(speeds);
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();

                if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                    return true;

                double now = Timer.getFPGATimestamp();
                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                if (dx < kMoveThroughPose.STALL_THRESHOLD && dy < kMoveThroughPose.STALL_THRESHOLD && (now - last[2]) > kMoveThroughPose.STALL_TIMEOUT)
                    return true;

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
        PIDController xController = new PIDController(kMoveToPose.X_KP, kMoveToPose.X_KI, kMoveToPose.X_KD);
        PIDController yController = new PIDController(kMoveToPose.Y_KP, kMoveToPose.Y_KI, kMoveToPose.Y_KD);
        PIDController thetaController = new PIDController(kMoveToPose.THETA_KP, kMoveToPose.THETA_KI, kMoveToPose.THETA_KD);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        final double[] last = new double[3];

        return new CommandBuilder(swerve)
            .named("MoveToPose")
            .onInitialize(() -> {
                Pose2d p = swerve.getPose();
                last[0] = p.getX();
                last[1] = p.getY();
                last[2] = Timer.getFPGATimestamp();
                xController.reset();
                yController.reset();
                thetaController.reset();
            })
            .onExecute(() -> {
                Pose2d current = swerve.getPose();

                double xSpeed = xController.calculate(current.getX(), targetPose.getX());
                double ySpeed = yController.calculate(current.getY(), targetPose.getY());
                double thetaSpeed = thetaController.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
                ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), ySpeed);
                thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), maxAngularSpeed), thetaSpeed);

                if (Math.abs(targetPose.getX() - current.getX()) < kMoveToPose.DEADBAND_ERROR) xSpeed = 0.0;
                if (Math.abs(targetPose.getY() - current.getY()) < kMoveToPose.DEADBAND_ERROR) ySpeed = 0.0;

                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, thetaSpeed, current.getRotation()
                );

                swerve.drive(speeds);
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();
                double now = Timer.getFPGATimestamp();

                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                if (dx < kMoveToPose.STALL_THRESHOLD && dy < kMoveToPose.STALL_THRESHOLD && (now - last[2]) > kMoveToPose.STALL_TIMEOUT)
                    return true;

                if (dx > kMoveToPose.STALL_THRESHOLD || dy > kMoveToPose.STALL_THRESHOLD) {
                    last[0] = current.getX();
                    last[1] = current.getY();
                    last[2] = now;
                }

                boolean atPosition =
                    Math.abs(current.getX() - targetPose.getX()) < kMoveToPose.POSITION_TOLERANCE_METERS &&
                    Math.abs(current.getY() - targetPose.getY()) < kMoveToPose.POSITION_TOLERANCE_METERS;

                boolean atAngle =
                    Math.abs(current.getRotation().getRadians() - targetPose.getRotation().getRadians())
                    < kMoveToPose.ANGLE_TOLERANCE_RADIANS;

                return atPosition && atAngle;
            })
            .onEnd(swerve::stop);
    }
}
