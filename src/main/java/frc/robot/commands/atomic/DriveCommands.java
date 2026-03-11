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

/**
 * Collection of atomic drive commands for the swerve subsystem.
 *
 * <p>This class provides:
 * <ul>
 *   <li>Simple heading-maintenance and pointing commands (non-PID)</li>
 *   <li>Vector-based movement commands</li>
 *   <li>Full PID pose movement commands</li>
 *   <li>Utility commands such as drive-forward and lock</li>
 * </ul>
 *
 * <p>All commands are stateless factories and do not retain internal state.</p>
 */
public final class DriveCommands {

    private DriveCommands() {}

    // -------------------------------------------------------------------------
    // Helper: Simple proportional heading controller
    // -------------------------------------------------------------------------

    /**
     * Computes an angular velocity (omega) that rotates the robot from its
     * current heading toward a desired heading using a simple proportional
     * controller. No integral or derivative terms are used.
     *
     * <p>This matches the rotation logic used in {@link #driveToPoint} and
     * ensures consistent heading behavior across all non-PID commands.</p>
     *
     * @param current the robot's current rotation
     * @param desired the desired rotation
     * @return angular velocity in rad/s
     */
    private static double computeOmega(Rotation2d current, Rotation2d desired) {
        double error = desired.minus(current).getRadians();
        error = MathUtil.angleModulus(error); // wrap to [-π, π]

        double kP = 5.5; // rad/s per rad of error
        return error * kP;
    }

    // -------------------------------------------------------------------------
    // Helper: Compute heading from robot → target
    // -------------------------------------------------------------------------

    /**
     * Computes the direction the robot should face to look directly at a target pose.
     *
     * @param robot  the robot's current pose
     * @param target the target pose
     * @return a Rotation2d pointing from robot → target
     */
    private static Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
        double dx = target.getX() - robot.getX();
        double dy = target.getY() - robot.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    // =========================================================================
    // Maintain Heading to Target
    // =========================================================================

    /**
     * Drives the robot using driver-supplied X/Y translation while automatically
     * rotating to face a supplied target pose. Uses simple proportional heading control.
     *
     * @param swerve the swerve subsystem
     * @param targetPoseSupplier supplier for the target pose
     * @param xSupplier driver X translation input
     * @param ySupplier driver Y translation input
     * @return a command that maintains heading while allowing translation
     */
    public static Command maintainHeadingToTarget(
        SwerveSubsystem swerve,
        Supplier<Pose2d> targetPoseSupplier,
        Supplier<Double> xSupplier,
        Supplier<Double> ySupplier
    ) {

        return new CommandBuilder(swerve)
            .named("MaintainHeadingToTarget")
            .onExecute(() -> {

                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                // Compute desired facing direction
                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                // Simple proportional heading correction
                double omega = computeOmega(current.getRotation(), desiredHeading);

                // Drive with driver translation + auto rotation
                swerve.drive(new ChassisSpeeds(
                    xSupplier.get(),
                    ySupplier.get(),
                    omega
                ));
            })
            .onEnd(swerve::stop);
    }

    // =========================================================================
    // Point To Target (Supplier version)
    // =========================================================================

    /**
     * Rotates the robot in place until it is facing a supplied target pose.
     * Uses simple proportional heading control.
     *
     * @param swerve the swerve subsystem
     * @param targetPoseSupplier supplier for the target pose
     * @return a command that rotates until aligned with the target
     */
    public static Command pointToTarget(
        SwerveSubsystem swerve,
        Supplier<Pose2d> targetPoseSupplier
    ) {

        return new CommandBuilder(swerve)
            .named("PointToTarget")
            .onExecute(() -> {

                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);
                double omega = computeOmega(current.getRotation(), desiredHeading);

                // Rotate in place
                swerve.drive(new ChassisSpeeds(0.0, 0.0, omega));
            })
            .isFinished(() -> {

                Pose2d current = swerve.getPose();
                Pose2d target = targetPoseSupplier.get();

                Rotation2d desiredHeading = computeHeadingToTarget(current, target);

                // Stop when within tolerance
                double error = Math.abs(
                    MathUtil.angleModulus(
                        current.getRotation().minus(desiredHeading).getRadians()
                    )
                );

                return error < Math.toRadians(5.0);
            })
            .onEnd(swerve::stop);
    }

    // =========================================================================
    // Point To Target (Fixed Pose version)
    // =========================================================================

    /**
     * Rotates the robot in place until it is facing a fixed target pose.
     * Uses simple proportional heading control.
     *
     * @param swerve the swerve subsystem
     * @param targetPose the fixed target pose
     * @return a command that rotates until aligned with the target
     */
    public static Command pointToTarget(
        SwerveSubsystem swerve,
        Pose2d targetPose
    ) {

        return new CommandBuilder(swerve)
            .named("PointToTarget")
            .onExecute(() -> {

                Pose2d current = swerve.getPose();
                Rotation2d desiredHeading = computeHeadingToTarget(current, targetPose);

                double omega = computeOmega(current.getRotation(), desiredHeading);

                swerve.drive(new ChassisSpeeds(0.0, 0.0, omega));
            })
            .isFinished(() -> {

                Pose2d current = swerve.getPose();
                Rotation2d desiredHeading = computeHeadingToTarget(current, targetPose);

                double error = Math.abs(
                    MathUtil.angleModulus(
                        current.getRotation().minus(desiredHeading).getRadians()
                    )
                );

                return error < Math.toRadians(20.0);
            })
            .onEnd(swerve::stop);
    }

    // =========================================================================
    // Move Through Pose (vector-based, no slowdown)
    // =========================================================================

    /**
     * Drives through a target pose at constant speed without slowing down.
     * Uses PID for rotation only. Translation is purely vector-based.
     *
     * <p>Useful for autos where the robot should pass through a waypoint
     * without stopping.</p>
     *
     * @param swerve the swerve subsystem
     * @param targetPose the pose to pass through
     * @param maxSpeed maximum translation speed
     * @param maxAngularSpeed maximum rotation speed
     * @param passRadius distance threshold for considering the pose "passed"
     * @return a command that drives through the pose
     */
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

                // Compute direction vector
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                double dirX = dx / (distance + 1e-9);
                double dirY = dy / (distance + 1e-9);

                // Constant-speed translation
                double vx_field = dirX * maxSpeed;
                double vy_field = dirY * maxSpeed;

                // Rotation PID
                double thetaSpeed = thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );
                thetaSpeed = MathUtil.clamp(thetaSpeed, -maxAngularSpeed, maxAngularSpeed);

                // Convert to robot-relative
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

                // Pass-through condition
                if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius)
                    return true;

                // Stall detection
                double dx = Math.abs(current.getX() - last[0]);
                double dy = Math.abs(current.getY() - last[1]);

                boolean stalled =
                    dx < kMoveThroughPose.STALL_THRESHOLD &&
                    dy < kMoveThroughPose.STALL_THRESHOLD &&
                    (now - last[2]) > kMoveThroughPose.STALL_TIMEOUT;

                if (stalled)
                    return true;

                // Update last movement if moving
                if (dx > kMoveThroughPose.STALL_THRESHOLD || dy > kMoveThroughPose.STALL_THRESHOLD) {
                    last[0] = current.getX();
                    last[1] = current.getY();
                    last[2] = now;
                }

                return false;
            })
            .onEnd(swerve::stop);
    }

    // =========================================================================
    // Move To Pose (full PID control)
    // =========================================================================

    /**
     * Moves the robot to a target pose using full PID control on both translation
     * and rotation. Includes slowdown ramps, deadbands, and stall detection.
     *
     * <p>This is the most precise pose movement command in the system.</p>
     *
     * @param swerve the swerve subsystem
     * @param targetPose the desired pose
     * @param maxSpeed maximum translation speed
     * @param maxAngularSpeed maximum rotation speed
     * @return a command that drives to the pose
     */
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

                // Vector to target
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                if (distance < 1e-6) {
                    swerve.stop();
                    return;
                }

                double dirX = dx / distance;
                double dirY = dy / distance;

                // PID on distance
                double pidOut = distancePID.calculate(0.0, distance);
                double speed = pidOut;

                // Terminal slowdown
                double slowdownStart = kMoveToPose.SLOWDOWN_START;
                double minSpeedAtZero = 0.0;

                double maxAllowedSpeed;
                if (distance > slowdownStart) {
                    maxAllowedSpeed = maxSpeed;
                } else {
                    double t = distance / slowdownStart;
                    maxAllowedSpeed = MathUtil.interpolate(minSpeedAtZero, maxSpeed, t);
                }

                speed = MathUtil.clamp(speed, -maxAllowedSpeed, maxAllowedSpeed);

                // Final deadband
                if (distance < kMoveToPose.POSITION_TOLERANCE_METERS &&
                    Math.abs(speed) < 0.05) {
                    speed = 0.0;
                }

                // Convert scalar → XY
                double xSpeed = dirX * speed;
                double ySpeed = dirY * speed;

                // Rotation PID + angular slowdown
                double thetaSpeed = thetaPID.calculate(
                    current.getRotation().getRadians(),
                    targetPose.getRotation().getRadians()
                );

                double angleScale = MathUtil.clamp(distance / slowdownStart, 0.25, 1.0);
                thetaSpeed *= angleScale;

                thetaSpeed = MathUtil.clamp(thetaSpeed, -maxAngularSpeed, maxAngularSpeed);

                // Field → robot
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

    // =========================================================================
    // Drive To Point (vector-based, simple heading control)
    // =========================================================================

    /**
     * Drives the robot toward a target point at a fixed velocity while rotating
     * to match the target pose's heading. Uses simple proportional heading control.
     *
     * @param swerve the swerve subsystem
     * @param targetPose the target pose
     * @param velocity translation speed in m/s
     * @param stopAtEnd whether to stop the robot when finished
     * @return a command that drives toward the point
     */
    public static Command driveToPoint(
        SwerveSubsystem swerve,
        Pose2d targetPose,
        double velocity,
        boolean stopAtEnd
    ) {

        return new CommandBuilder(swerve)
            .named("DriveToPoint")
            .onExecute(() -> {

                Pose2d current = swerve.getPose();
                Rotation2d currentHeading = current.getRotation();

                // Translation vector
                double dx = targetPose.getX() - current.getX();
                double dy = targetPose.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                double ux = 0.0;
                double uy = 0.0;

                if (distance > 0.05) {   // 5 cm threshold
                    ux = dx / distance;
                    uy = dy / distance;
                }

                // -----------------------------------------
                // 2. Compute heading error (wrapped)
                // -----------------------------------------
                Rotation2d finalHeading = targetPose.getRotation();

                double headingError = finalHeading.minus(currentHeading).getRadians();
                headingError = MathUtil.angleModulus(headingError); // wrap to [-π, π]

                // C++ used: ROT_kP = 5 rad/s per rad
                double ROT_kP = 5.0;
                double omega = headingError * ROT_kP;

                // -----------------------------------------
                // 3. Build chassis speeds
                // -----------------------------------------
                double vx = velocity * ux;
                double vy = velocity * uy;

                ChassisSpeeds robotRelative =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx,
                        vy,
                        omega,
                        currentHeading
                    );

                swerve.driveRobotRelative(robotRelative);
            })
            .isFinished(() -> {

                Pose2d current = swerve.getPose();

                double dist = current.getTranslation()
                                    .getDistance(targetPose.getTranslation());

                boolean closeEnough = dist < 0.05; // 5 cm
                return closeEnough;
            })
            .onEnd(() -> {
                if (stopAtEnd) {
                    swerve.stop();
                }
            });
    }

    // =========================================================================
    // Drive Forward Meters (relative to current heading)
    // =========================================================================

    /**
     * Drives the robot forward a specified distance in meters, based on the robot's
     * current heading at the moment the command starts. Translation is performed
     * using a constant-speed vector drive, and heading is maintained using simple
     * proportional correction.
     *
     * <p>This command:
     * <ul>
     *   <li>Captures the robot's starting pose</li>
     *   <li>Computes a target pose exactly {@code distanceMeters} forward</li>
     *   <li>Drives toward that pose at a fixed velocity</li>
     *   <li>Maintains the original heading</li>
     *   <li>Finishes when within 5 cm of the target</li>
     * </ul>
     *
     * @param swerve the swerve subsystem
     * @param distanceMeters distance to drive forward (meters)
     * @param velocity constant translation speed (m/s)
     * @param stopAtEnd whether to stop the robot when the command ends
     * @return a command that drives forward by the specified distance
     */
    public static Command driveForwardMeters(
        SwerveSubsystem swerve,
        double distanceMeters,
        double velocity,
        boolean stopAtEnd
    ) {

        return new CommandBuilder(swerve)
            .named("DriveForwardMeters")

            // -----------------------------------------------------
            // Initialize: compute and store the target pose
            // -----------------------------------------------------
            .onInitialize(() -> {

                Pose2d current = swerve.getPose();
                Rotation2d heading = current.getRotation();

                // Compute target position distanceMeters forward
                double dx = distanceMeters * heading.getCos();
                double dy = distanceMeters * heading.getSin();

                Pose2d target = new Pose2d(
                    current.getX() + dx,
                    current.getY() + dy,
                    heading // maintain original heading
                );

                swerve.setTemporaryTargetPose(target);
            })

            // -----------------------------------------------------
            // Execute: drive toward the stored target pose
            // -----------------------------------------------------
            .onExecute(() -> {

                Pose2d current = swerve.getPose();
                Pose2d target = swerve.getTemporaryTargetPose();
                Rotation2d currentHeading = current.getRotation();

                // 1. Translation vector toward target
                double dx = target.getX() - current.getX();
                double dy = target.getY() - current.getY();
                double distance = Math.hypot(dx, dy);

                double ux = 0.0;
                double uy = 0.0;

                // Normalize direction if far enough away
                if (distance > 0.05) {
                    ux = dx / distance;
                    uy = dy / distance;
                }

                // 2. Heading correction (simple proportional)
                double headingError = target.getRotation().minus(currentHeading).getRadians();
                headingError = MathUtil.angleModulus(headingError);

                double ROT_kP = 5.0;
                double omega = headingError * ROT_kP;

                // 3. Build chassis speeds
                double vx = velocity * ux;
                double vy = velocity * uy;

                ChassisSpeeds robotRelative =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                        vx,
                        vy,
                        omega,
                        currentHeading
                    );

                swerve.driveRobotRelative(robotRelative);
            })

            // -----------------------------------------------------
            // Finish when within 5 cm of the target pose
            // -----------------------------------------------------
            .isFinished(() -> {

                Pose2d current = swerve.getPose();
                Pose2d target = swerve.getTemporaryTargetPose();

                double dist = current.getTranslation()
                                    .getDistance(target.getTranslation());

                return dist < 0.05; // 5 cm threshold
            })

            // -----------------------------------------------------
            // Stop if requested
            // -----------------------------------------------------
            .onEnd(() -> {
                if (stopAtEnd) {
                    swerve.stop();
                }
            });
    }


    // =========================================================================
    // Swerve Lock
    // =========================================================================

    /**
     * Locks the swerve modules into an "X" configuration to resist external forces.
     *
     * <p>This is typically used when:
     * <ul>
     *   <li>Holding position while shooting</li>
     *   <li>Preventing drift on slopes</li>
     *   <li>Stabilizing the robot during interactions</li>
     * </ul>
     *
     * <p>The command runs continuously until interrupted.</p>
     *
     * @param swerve the swerve subsystem
     * @return a command that locks the swerve modules
     */
    public static Command lock(SwerveSubsystem swerve) {
        return new CommandBuilder(swerve)
            .named("SwerveLock")
            .onExecute(swerve::lock)
            .onEnd(swerve::stop);
    }
}