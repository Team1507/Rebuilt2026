//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

// Java
import java.util.ArrayList;
import java.util.List;

// WPI Lib
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

// Framework
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;

// Commands
import frc.robot.commands.atomic.*;
import frc.robot.commands.coordinators.*;

// Core
import frc.lib.core.math.FieldFlip;
import frc.lib.core.util.Alliance;

// Constants
import frc.robot.Constants.kAgitator;
import frc.robot.Constants.kIntake.*;

/**
 * AutoSequence
 *
 * Fluent builder for constructing autonomous routines.
 * Each method appends a Command to an internal list, and {@link #build()}
 * assembles them into a sequential command.
 *
 * This class is intentionally readable and teachable — students can build
 * autos by chaining high-level actions like:
 *
 *   new AutoSequence(...).resetPose(...).driveTo(...).shoot().build();
 */
public class AutoSequence {

    // ------------------------------------------------------------
    // Core Fields
    // ------------------------------------------------------------

    private final List<Command> steps = new ArrayList<>();

    private final SubsystemsRecord record;
    private final CoordinatorRecord coordinator;

    private final double MaxSpeed;
    private final double MaxAngularRate;

    private Double nextSpeedOverride = null;
    private Double nextAngularOverride = null;

    private final Timer autoTimer = new Timer();


    // ------------------------------------------------------------
    // Constructor
    // ------------------------------------------------------------

    /**
     * Creates a new AutoSequence builder.
     *
     * @param record         SubsystemsRecord containing all robot subsystems.
     * @param coordinator    CoordinatorRecord for shooter/agitator control.
     * @param MaxSpeed       Default translational speed for movement.
     * @param MaxAngularRate Default rotational speed for movement.
     */
    public AutoSequence(SubsystemsRecord record, CoordinatorRecord coordinator,
                        double MaxSpeed, double MaxAngularRate) {
        this.record = record;
        this.coordinator = coordinator;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
    }


    // ------------------------------------------------------------
    // Speed Override Utilities
    // ------------------------------------------------------------

    /** Applies a temporary translational speed override for the next movement step. */
    public AutoSequence withSpeed(double speed) {
        return withSpeed(speed, MaxAngularRate);
    }

    /** Applies a temporary translational + angular speed override for the next movement step. */
    public AutoSequence withSpeed(double speed, double angularRate) {
        this.nextSpeedOverride = speed;
        this.nextAngularOverride = angularRate;
        return this;
    }

    /** Convenience: slow, precise movement. */
    public AutoSequence creep() {
        this.nextSpeedOverride = 0.3 * MaxSpeed;
        this.nextAngularOverride = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        return this;
    }

    /** Convenience: moderately slow movement. */
    public AutoSequence slow() {
        this.nextSpeedOverride = 0.5 * MaxSpeed;
        this.nextAngularOverride = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        return this;
    }


    // ------------------------------------------------------------
    // Movement Commands
    // ------------------------------------------------------------

    /** Drives forward a fixed distance. */
    public AutoSequence driveDistance(double meters) {
        double speed = (nextSpeedOverride != null) ? nextSpeedOverride : MaxSpeed;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.driveForwardMeters(
            record.swerve(), meters, speed, true
        ));
        return this;
    }

    /** Drives to a specific pose and stops on it. */
    public AutoSequence driveTo(Pose2d target) {
        double speed = (nextSpeedOverride != null) ? nextSpeedOverride : MaxSpeed;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.driveToPoint(
            record.swerve(), target, speed, true
        ));
        return this;
    }

    /** Drives through a pose without stopping — useful for smooth path shaping. */
    public AutoSequence moveThrough(Pose2d target, double passRadius) {
        double speed = (nextSpeedOverride != null) ? nextSpeedOverride : MaxSpeed;
        double angular = (nextAngularOverride != null) ? nextAngularOverride : MaxAngularRate;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.moveThroughPose(
            record.swerve(), target, speed, angular, passRadius
        ));
        return this;
    }

    /** Rotates the robot to match the heading of a target pose. */
    public AutoSequence changeHeading(Pose2d targetPose) {
        steps.add(DriveCommands.changeHeading(record.swerve(), targetPose));
        return this;
    }

    /** Points the robot toward a specific target pose. */
    public AutoSequence pointToTarget(Pose2d targetPose) {
        steps.add(DriveCommands.pointToTarget(record.swerve(), () -> targetPose));
        return this;
    }


    // ------------------------------------------------------------
    // Hopper Commands
    // ------------------------------------------------------------

    /** Extends the hopper. */
    public AutoSequence hopperExtend() {
        steps.add(HopperCommands.extend(record.hopper()));
        return this;
    }

    /** Retracts the hopper. */
    public AutoSequence hopperRetract() {
        steps.add(HopperCommands.retract(record.hopper()));
        return this;
    }

    // ------------------------------------------------------------
    // Intake Commands
    // ------------------------------------------------------------

    /** Deploys the intake and runs it to collect a note. */
    public AutoSequence intakeDeploy() {
        steps.add(IntakeSequences.deployAndRun(
            record.hopper(), record.intakeArm(), record.intakeRoller()
        ));
        return this;
    }

    /** Stows the intake arm. */
    public AutoSequence intakeRetract() {
        steps.add(IntakeSequences.stow(
            record.intakeArm(), record.intakeRoller()
        ));
        return this;
    }

    /** Runs intake roller at low speed. */
    public AutoSequence intakeLow() {
        steps.add(IntakeRollerCommands.lowRollerSpeed(record.intakeRoller()));
        return this;
    }

    /** Runs intake roller at auto speed. */
    public AutoSequence intakeHigh() {
        steps.add(IntakeRollerCommands.autoRollerSpeed(record.intakeRoller()));
        return this;
    }

    /** Runs intake roller at idle speed. */
    public AutoSequence intakeIdle() {
        steps.add(IntakeRollerCommands.idleRollerSpeed(record.intakeRoller()));
        return this;
    }

    // ------------------------------------------------------------
    // Shooter Commands
    // ------------------------------------------------------------

    /** Fires a note using the model-based shooter controller. */
    public AutoSequence shoot() {
        steps.add(ShooterControllers.shootModelBased(
            coordinator, kAgitator.AGITATE_TO_SHOOTER_DUTY, kRoller.DUTY_HIGH
        ));
        return this;
    }

    /** Shoots until the auto timer reaches a given time. */
    public AutoSequence shootUntil(double endTime) {
        steps.add(Commands.race(
            ShooterControllers.shootModelBased(
                coordinator, kAgitator.AGITATE_TO_SHOOTER_DUTY, kRoller.DUTY_HIGH
            ),
            Commands.waitUntil(() -> autoTimer.get() >= endTime)
        ));
        return this;
    }

    /** Shoots at a fixed RPM until the auto timer reaches a given time. */
    public AutoSequence shootRPMUntil(double endTime, double RPM) {
        steps.add(Commands.race(
            ShooterControllers.shootFixedRPM(
                coordinator, RPM, kAgitator.AGITATE_TO_SHOOTER_DUTY, kRoller.DUTY_HIGH
            ),
            Commands.waitUntil(() -> autoTimer.get() >= endTime)
        ));
        return this;
    }

    /** Sets shooter target pose for both shooter subsystems. */
    public AutoSequence setShooterTarget(Pose2d pose) {
        steps.add(Commands.runOnce(() -> record.BLUshooter().setShooterTarget(pose)));
        steps.add(Commands.runOnce(() -> record.YELshooter().setShooterTarget(pose)));
        return this;
    }

    /** Points the robot at the same target that the shooter is targeting. */
    public AutoSequence pointToShoot() {
        steps.add(DriveCommands.pointToTarget(
            record.swerve(), record.BLUshooter()::getTargetPose
        ));
        return this;
    }

    // ------------------------------------------------------------
    // Pose Reset (QuestNav + Swerve)
    // ------------------------------------------------------------

    /**
     * Requests a pose reset and waits for QuestNavController to confirm it.
     *
     * @param resetPose Field pose to reset to.
     */
    public AutoSequence resetPose(Pose2d resetPose) {
        steps.add(Commands.runOnce(() ->
            record.questNavController().requestPoseReset(resetPose)
        ));

        steps.add(
            Commands.waitUntil(() -> record.questNavController().isResetComplete())
                    .withTimeout(0.5)
        );

        return this;
    }


    // ------------------------------------------------------------
    // Parallel / Race / Deadline Groups
    // ------------------------------------------------------------

    /** Runs multiple AutoSequences in parallel, ending when ANY finishes. */
    public AutoSequence race(AutoSequenceBuilder... builders) {
        List<Command> commands = new ArrayList<>();

        for (AutoSequenceBuilder builder : builders) {
            AutoSequence sub = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            commands.add(sub.build());
        }

        steps.add(Commands.race(commands.toArray(Command[]::new)));
        return this;
    }

    /** Runs multiple AutoSequences in parallel, ending when ALL finish. */
    public AutoSequence parallel(AutoSequenceBuilder... builders) {
        List<Command> commands = new ArrayList<>();

        for (AutoSequenceBuilder builder : builders) {
            AutoSequence sub = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            commands.add(sub.build());
        }

        steps.add(Commands.parallel(commands.toArray(Command[]::new)));
        return this;
    }

    /** Runs a deadline AutoSequence with additional parallel sequences. */
    public AutoSequence deadline(AutoSequenceBuilder deadlineBuilder,
                                 AutoSequenceBuilder... others) {

        AutoSequence deadlineSeq =
            new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
        deadlineBuilder.build(deadlineSeq);

        List<Command> otherCMD = new ArrayList<>();
        for (AutoSequenceBuilder builder : others) {
            AutoSequence sub =
                new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            otherCMD.add(sub.build());
        }

        steps.add(Commands.deadline(
            deadlineSeq.build(),
            otherCMD.toArray(Command[]::new)
        ));

        return this;
    }


    // ------------------------------------------------------------
    // Timer Utilities
    // ------------------------------------------------------------

    /** Starts the autonomous timer. */
    public AutoSequence startTimer() {
        steps.add(Commands.runOnce(() -> {
            autoTimer.reset();
            autoTimer.start();
        }));
        return this;
    }

    /** Returns a command that ends when the timer reaches a given time. */
    public Command endAtTime(double endTime) {
        return Commands.waitUntil(() -> autoTimer.get() >= endTime);
    }

    /** Waits a fixed number of seconds. */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }


    // ------------------------------------------------------------
    // Build Final Command
    // ------------------------------------------------------------

    /** Builds the final sequential autonomous command. */
    public Command build() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }


    // ------------------------------------------------------------
    // Builder Interface
    // ------------------------------------------------------------

    @FunctionalInterface
    public interface AutoSequenceBuilder {
        void build(AutoSequence sequence);
    }


    // ------------------------------------------------------------
    // Utility
    // ------------------------------------------------------------

    /** Applies field flip for RED alliance. */
    public Pose2d translate(Pose2d original) {
        return Alliance.isRed()
            ? FieldFlip.overDiagonal(original)
            : original;
    }
}
