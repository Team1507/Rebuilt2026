//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.atomic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import frc.lib.core.util.CommandBuilder;
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
        PIDController headingController = new PIDController(2.4, 0.0, 0.03);
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

     public static Command pointToTarget(
        SwerveSubsystem swerve,
        Pose2d targetPose
    ) {
        PIDController headingController = new PIDController(6.0, 0.0, 0.1);
        headingController.enableContinuousInput(-Math.PI, Math.PI);
        return new CommandBuilder(swerve)
            .named("PointToTarget")
            .onExecute(() -> {
                Pose2d current = swerve.getPose();

                Rotation2d desiredHeading = computeHeadingToTarget(current, targetPose);

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
                Pose2d target = targetPose;

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                double angleError = Math.abs(
                    current.getRotation().minus(desiredHeading).getRadians()
                );

                return angleError < Math.toRadians(20.0); // 5 degree tolerance
            })
            
            .onEnd(swerve::stop);
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

        // Stall detection
        final double[] stallStartTime = { -1.0 };

        return new CommandBuilder(swerve)
            .named("MoveToPose_VectorPID_NoMinSpeed")
            .onInitialize(() -> {
                distancePID.reset();
                thetaPID.reset();
                stallStartTime[0] = -1.0;
            })
            .onExecute(() -> {

                Pose2d current = swerve.getPose();

                // -----------------------------------------
                // 1. Vector to target
                // -----------------------------------------
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                if (distance < 1e-6) {
                    swerve.stop();
                    return;
                }

                double dirX = dx / distance;
                double dirY = dy / distance;

                // -----------------------------------------
                // 2. PID on distance (inverted form)
                // -----------------------------------------
                double pidOut = distancePID.calculate(0.0, distance);
                double speed = pidOut;

                // -----------------------------------------
                // 3. Terminal slowdown ramp (correct fix)
                // -----------------------------------------
                double slowdownStart = kMoveToPose.SLOWDOWN_START; // e.g. 0.30 m
                double minSpeedAtZero = 0.0; // allow true stop

                double maxAllowedSpeed;
                if (distance > slowdownStart) {
                    maxAllowedSpeed = maxSpeed;
                } else {
                    double t = distance / slowdownStart; // 1 → 0 as you approach
                    maxAllowedSpeed = MathUtil.interpolate(minSpeedAtZero, maxSpeed, t);
                }

                speed = MathUtil.clamp(speed, -maxAllowedSpeed, maxAllowedSpeed);

                // -----------------------------------------
                // 4. Final deadband (no oscillation)
                // -----------------------------------------
                if (distance < kMoveToPose.POSITION_TOLERANCE_METERS &&
                    Math.abs(speed) < 0.05) {
                    speed = 0.0;
                }

                // -----------------------------------------
                // 5. Convert scalar → XY
                // -----------------------------------------
                double xSpeed = dirX * speed;
                double ySpeed = dirY * speed;

                // -----------------------------------------
                // 6. Rotation PID + angular slowdown
                // -----------------------------------------
                double thetaSpeed = thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                // Angular slowdown near target
                double angleScale = MathUtil.clamp(distance / slowdownStart, 0.25, 1.0);
                thetaSpeed *= angleScale;

                thetaSpeed = MathUtil.clamp(thetaSpeed, -maxAngularSpeed, maxAngularSpeed);

                // -----------------------------------------
                // 7. Field → robot
                // -----------------------------------------
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

                double dist =
                    current.getTranslation().getDistance(targetPose.getTranslation());

                double angleError =
                    Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

                boolean atPos = dist < kMoveToPose.POSITION_TOLERANCE_METERS;
                boolean atAngle = angleError < kMoveToPose.ANGLE_TOLERANCE_RADIANS;

                if (atPos && atAngle) return true;

                // Stall detection near target
                boolean nearTarget = dist < 1.5 * kMoveToPose.POSITION_TOLERANCE_METERS;
                if (!nearTarget) {
                    stallStartTime[0] = -1.0;
                    return false;
                }

                ChassisSpeeds speeds = swerve.getInputs().speeds;
                double speedMag = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

                boolean effectivelyStopped = speedMag < kMoveToPose.STALL_THRESHOLD;
                double now = Timer.getFPGATimestamp();

                if (!effectivelyStopped) {
                    stallStartTime[0] = -1.0;
                    return false;
                }

                if (stallStartTime[0] < 0.0) {
                    stallStartTime[0] = now;
                    return false;
                }

                return (now - stallStartTime[0]) > kMoveToPose.STALL_TIMEOUT;
            })
            .onEnd(swerve::stop);
    }

    public static Command lock(SwerveSubsystem swerve) {
        return new CommandBuilder(swerve)
            .named("SwerveLock")
            .onExecute(swerve::lock)
            .onEnd(swerve::stop);
    }
}