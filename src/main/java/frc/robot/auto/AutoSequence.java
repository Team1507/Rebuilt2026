//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.auto;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

// Java
import java.util.ArrayList;
import java.util.List;

// WPI Lib
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.math.geometry.Pose2d;

// Robot Commands
import frc.robot.commands.agitate.*;
import frc.robot.commands.climb.*;
import frc.robot.commands.drive.*;
import frc.robot.commands.feed.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shoot.*;

// Utilities
import frc.robot.utilities.SubsystemsRecord;

/**
 * AutoSequence
 *
 * Mentor-provided base class for building autonomous routines using a fluent API.
 * Students will extend this by adding new high-level actions (score, intake, etc.)
 * that internally add Commands to the 'steps' list.
 */
public class AutoSequence {

    // Internal list of steps that will be executed in order
    private final List<Command> steps = new ArrayList<>();

    // Required subsystem reference for movement commands
    private final SubsystemsRecord record;
    private final double MaxSpeed;
    private final double MaxAngularRate;

    private Double nextSpeedOverride = null;
    private Double nextAngularOverride = null;

    /**
     * Creates a new AutoSequence builder.
     *
     * @param drivetrain        The drivetrain subsystem used for all movement commands.
     * @param MaxSpeed          The default translational speed used for move actions.
     * @param MaxAngularRate    The default rotational rate used for move actions.
     *
     * <p>This constructor defines the baseline speed profile for the entire
     * autonomous routine. Individual steps may override these values using
     * {@link #withSpeed(double)} or {@link #withSpeed(double, double)}.</p>
     */
    public AutoSequence(SubsystemsRecord record, double MaxSpeed, double MaxAngularRate) {
        this.record = record;
        this.MaxSpeed = MaxSpeed;
        this.MaxAngularRate = MaxAngularRate;
    }

    /**
     * Applies a temporary translational speed override for the next movement
     * command only. The angular speed remains unchanged.
     *
     * @param speed  The translational speed to use for the next move.
     * @return       This AutoSequence for fluent chaining.
     *
     * <p>This override is consumed by the next call to {@code moveTo()} and
     * automatically resets afterward.</p>
     */
    public AutoSequence withSpeed(double speed) {
        return withSpeed(speed, MaxAngularRate);
    }

    /**
     * Applies a temporary translational and angular speed override for the next
     * movement command only.
     *
     * @param speed         The translational speed to use for the next move.
     * @param angularRate   The rotational rate to use for the next move.
     * @return              This AutoSequence for fluent chaining.
     *
     * <p>Overrides apply to a single movement step and then clear automatically,
     * ensuring later steps return to the default speed profile unless explicitly
     * overridden again.</p>
     */
    public AutoSequence withSpeed(double speed, double angularRate) {
        this.nextSpeedOverride = speed;
        this.nextAngularOverride = angularRate;
        return this;
    }

    public AutoSequence creep(){
        this.nextSpeedOverride = 0.3 * MaxSpeed;
        this.nextAngularOverride = RotationsPerSecond.of(0.50).in(RadiansPerSecond);
        return this;
    }

    public AutoSequence slow(){
        this.nextSpeedOverride = 0.5 * MaxSpeed;
        this.nextAngularOverride = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
        return this;
    }

   /**
     * Adds a movement step that drives the robot to the specified target pose.
     *
     * <p>This method automatically applies any temporary speed overrides set by
     * {@link #withSpeed(double)} or {@link #withSpeed(double, double)}. If no
     * override is active, the default maxSpeed and maxAngularRate provided in the
     * constructor are used.</p>
     *
     * <p>Speed overrides apply to only this movement command and are cleared
     * immediately afterward, ensuring later steps return to the default speed
     * profile unless explicitly overridden again.</p>
     *
     * @param target The desired end pose for the robot.
     * @return This AutoSequence instance for fluent chaining.
     *
     * <p><strong>Note:</strong> Students should not modify this method. It defines
     * the core behavior of movement steps within the autonomous builder.</p>
     */
    public AutoSequence moveTo(Pose2d target) {

        double speedToUse = (nextSpeedOverride != null)
            ? nextSpeedOverride
            : MaxSpeed;

        double angularToUse = (nextAngularOverride != null)
            ? nextAngularOverride
            : MaxAngularRate;

        // Clear override after one use
        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(new CmdMoveToPose(record.drivetrain(), target, speedToUse, angularToUse));
        return this;
    }

    /**
     * This should add a Command that performs the robot's scoring routine.
     * Example:
     * steps.add(new ScoreCommand(shooterSubsystem));
     */
    public AutoSequence shoot() {
        steps.add(new CmdShoot(
            500.0, 
            200.0, 
            200.0, 
            record.agitator(), 
            record.BLUfeeder(), 
            record.YELfeeder(), 
            record.BLUshooter()));
        return this;
    }

    public AutoSequence hopperExtend() {
        steps.add(new CmdHopperExtension(67.0, record.hopper()));
        return this;
    }

    /**
     * This should add a Command that runs the intake to collect a game piece.
     * Example:
     * steps.add(new IntakeCommand(intakeSubsystem));
     */
    public AutoSequence intakeDeploy() {
        steps.add(new CmdIntakeArmDown(record.intakeArm()));
        steps.add(new CmdIntakeRollerIntake(record.intakeRoller()));
        return this;
    }

    public AutoSequence intakeRetract(){
        steps.add(new CmdIntakeArmUp(record.intakeArm()));
        steps.add(new CmdIntakeRollerStop(record.intakeRoller()));
        return this;
    }

    /**
     * Wait for a number of seconds before continuing.
     * This is already implemented for students.
     */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }

    // may add more actions here:
    // - shoot()
    // - moveThrough(Pose2d...)
    // - parallel(Command...)
    // - race(Command...)
    // - outtake()
    // - align()
    // - moveRRT()
    // etc.


    /**
     * Adds a parallel race group to the autonomous sequence.
     *
     * <p>Each provided {@link AutoSequenceBuilder} creates its own temporary
     * {@link AutoSequence}. All resulting commands are run in parallel, and the
     * entire group ends as soon as any command finishes. All other commands in the
     * group are interrupted.</p>
     *
     * <p>This is useful for behaviors such as "drive until intake detects a note,"
     * where one command acts as the terminating condition for the others.</p>
     *
     * @param builders  One or more builders that define the commands to run in
     *                  parallel as part of the race group.
     * @return          This AutoSequence for fluent chaining.
     *
     * <p><strong>Note:</strong> Each builder receives a fresh AutoSequence instance,
     * ensuring its steps are isolated from the parent sequence.</p>
     */
    public AutoSequence race(AutoSequenceBuilder... builders) {
        List<Command> commands = new ArrayList<>();

        for (AutoSequenceBuilder builder : builders) {
            AutoSequence sub = new AutoSequence(record, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            commands.add(sub.build());
        }
        steps.add(Commands.race(commands.toArray(Command[]::new)));
        return this;
    }

    /**
     * Adds a parallel command group to the autonomous sequence.
     *
     * <p>Each provided {@link AutoSequenceBuilder} constructs a temporary
     * {@link AutoSequence}. All resulting commands run simultaneously, and the
     * group completes only when all commands have finished.</p>
     *
     * <p>This is useful for actions such as "drive while running intake," where
     * multiple robot subsystems operate concurrently until each completes its
     * task.</p>
     *
     * @param builders  One or more builders that define the commands to run in
     *                  parallel.
     * @return          This AutoSequence for fluent chaining.
     *
     * <p><strong>Note:</strong> Each builder receives its own isolated AutoSequence
     * instance, preventing interference with the parent sequence.</p>
     */
    public AutoSequence parallel(AutoSequenceBuilder... builders) {
        List<Command> commands = new ArrayList<>();

        for (AutoSequenceBuilder builder : builders) {
            AutoSequence sub = new AutoSequence(record, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            commands.add(sub.build());
        }
        steps.add(Commands.parallel(commands.toArray(Command[]::new)));
        return this;
    }

    /**
     * Adds a deadline command group to the autonomous sequence.
     *
     * <p>The deadlineBuilder defines the "deadline" command -- the command that
     * controls when the group ends. All other builders define commands that run in
     * parallel alongside the deadline. The group finishes when the deadline command
     * completes, and all other commands are interrupted.</p>
     *
     * <p>This is useful for behaviors such as "run intake and agitator until the
     * drive command reaches its target," where one command dictates the duration
     * of the others.</p>
     *
     * @param deadlineBuilder  The builder that defines the deadline command.
     * @param others           Additional builders whose commands run in parallel
     *                         but do not control group termination.
     * @return                 This AutoSequence for fluent chaining.
     *
     * <p><strong>Note:</strong> Each builder receives a fresh AutoSequence instance,
     * ensuring its steps remain isolated from the parent sequence.</p>
     */
    public AutoSequence deadline(AutoSequenceBuilder deadlineBuilder, AutoSequenceBuilder... others) {
        AutoSequence deadlineSeq = new AutoSequence(record, MaxSpeed, MaxAngularRate);
        deadlineBuilder.build(deadlineSeq);

        List<Command> otherCMD = new ArrayList<>();
        for (AutoSequenceBuilder builder : others) {
            AutoSequence sub = new AutoSequence(record, MaxSpeed, MaxAngularRate);
            builder.build(sub);
            otherCMD.add(sub.build());
        }
        steps.add(Commands.deadline(deadlineSeq.build(), otherCMD.toArray(Command[]::new)));
        return this;
    }

    /**
     * Finalizes the autonomous routine by assembling all queued steps into a
     * single sequential command.
     *
     * <p>Each step added through the fluent API (such as {@code moveTo()},
     * {@code waitSeconds()}, or future high-level actions) is executed in the
     * order it was added. The resulting command is ready to be scheduled by the
     * robot during the autonomous period.</p>
     *
     * @return A fully constructed {@link Command} representing the autonomous
     *         sequence.
     *
     * <p><strong>Note:</strong> Students should not modify this method. It defines
     * the execution model for all autonomous routines built with this class.</p>
     */
    public Command build() {
        return Commands.sequence(steps.toArray(Command[]::new));
    }

    @ FunctionalInterface
    public interface AutoSequenceBuilder {
        void build(AutoSequence sequence);
    }
}