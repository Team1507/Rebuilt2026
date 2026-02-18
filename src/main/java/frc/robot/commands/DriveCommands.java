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
        PIDController thetaController = new PIDController(
            kMoveThroughPose.THETA_KP,
            kMoveThroughPose.THETA_KI,
            kMoveThroughPose.THETA_KD
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        final double[] last = new double[3];

        return new CommandBuilder(swerve)
            .named("MoveThroughPose")
            .onInitialize(() -> {
                Pose2d p = swerve.getPose();
                last[0] = p.getX();
                last[1] = p.getY();
                last[2] = Timer.getFPGATimestamp();
                thetaController.reset();
            })
            .onExecute(() -> {

                Pose2d current = swerve.getPose();

                // --- CONSTANT-SPEED VECTOR TOWARD TARGET ---
                Translation2d error = targetPose.getTranslation().minus(current.getTranslation());
                double direction = Math.atan2(error.getY(), error.getX());

                double vx_field = maxSpeed * Math.cos(direction);
                double vy_field = maxSpeed * Math.sin(direction);

                // --- CONVERT TO ROBOT-RELATIVE ---
                Rotation2d heading = current.getRotation();
                double vx_robot =  vx_field * heading.getCos() + vy_field * heading.getSin();
                double vy_robot = -vx_field * heading.getSin() + vy_field * heading.getCos();

                // --- ROTATION PID ---
                double thetaSpeed = thetaController.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );
                thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), maxAngularSpeed), thetaSpeed);

                // --- DRIVE ---
                swerve.driveRobotRelative(new ChassisSpeeds(vx_robot, vy_robot, thetaSpeed));
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();

                // PASS THROUGH CONDITION
                if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                    return true;

                // STALL DETECTION
                double now = Timer.getFPGATimestamp();
                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                if (dx < kMoveThroughPose.STALL_THRESHOLD &&
                    dy < kMoveThroughPose.STALL_THRESHOLD &&
                    (now - last[2]) > kMoveThroughPose.STALL_TIMEOUT)
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
        PIDController xyController = new PIDController(
            kMoveToPose.XY_KP, 
            kMoveToPose.XY_KI, 
            kMoveToPose.XY_KD);
        PIDController thetaController = new PIDController(
            kMoveToPose.THETA_KP,
            kMoveToPose.THETA_KI,
            kMoveToPose.THETA_KD
        );
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        final double[] last = new double[3];

        return new CommandBuilder(swerve)
            .named("MoveToPose")
            .onInitialize(() -> {
                Pose2d p = swerve.getPose();
                last[0] = p.getX();
                last[1] = p.getY();
                last[2] = Timer.getFPGATimestamp();
                thetaController.reset();
            })
            .onExecute(() -> {

                 // 1. Get current robot pose from drivetrain's state estimator
                Pose2d current = swerve.getPose();

                // 2. Calculate velocity commands using PID controllers
                // Each controller compares current vs target and outputs a speed
                double xSpeed = xyController.calculate(
                current.getX(), 
                targetPose.getX()
                );

                double ySpeed = xyController.calculate(
                current.getY(), 
                targetPose.getY()
                );

                double thetaSpeed = thetaController.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                // 3. Cap speeds to prevent runaway values
                // (keeps motion safe and predictable during tuning)
                double minSpeed = 0.6; // or whatever feels right

                if (Math.abs(xSpeed) > 0.0)
                    xSpeed = Math.copySign(Math.max(Math.abs(xSpeed), minSpeed), xSpeed);

                if (Math.abs(ySpeed) > 0.0)
                    ySpeed = Math.copySign(Math.max(Math.abs(ySpeed), minSpeed), ySpeed);

                thetaSpeed = Math.copySign(Math.min(Math.abs(thetaSpeed), maxAngularSpeed), thetaSpeed);

                // 4. Deadband small dithers near target
                // If error is below DEADBAND_ERROR, zero out tiny corrections
                // This prevents jittery "hunting" around the goal
                if (Math.abs(targetPose.getX() - current.getX()) < kMoveToPose.DEADBAND_ERROR) xSpeed = 0.0;
                if (Math.abs(targetPose.getY() - current.getY()) < kMoveToPose.DEADBAND_ERROR) ySpeed = 0.0;

                // 5. Convert field-relative X/Y to robot-relative using current heading
                // (PID math is in field coordinates, but drivetrain expects robot-relative)
                ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed,
                    ySpeed,
                    thetaSpeed,
                    current.getRotation()
                );

                // 6. Send robot-relative speeds to CTRE drivetrain
                swerve.driveRobotRelative(robotRelativeSpeeds);
            })
            .isFinished(() -> {
                Pose2d current = swerve.getPose();
                double now = Timer.getFPGATimestamp();

                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                if (dx < kMoveToPose.STALL_THRESHOLD &&
                    dy < kMoveToPose.STALL_THRESHOLD &&
                    (now - last[2]) > kMoveToPose.STALL_TIMEOUT)
                    return true;

                if (dx > kMoveToPose.STALL_THRESHOLD || dy > kMoveToPose.STALL_THRESHOLD) {
                    last[0] = current.getX();
                    last[1] = current.getY();
                    last[2] = now;
                }

                boolean atPosition =
                    current.getTranslation().getDistance(targetPose.getTranslation()) <
                    kMoveToPose.POSITION_TOLERANCE_METERS;

                boolean atAngle =
                    Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians()) <
                    kMoveToPose.ANGLE_TOLERANCE_RADIANS;

                return atPosition && atAngle;
            })
            .onEnd(swerve::stop);
    }
}