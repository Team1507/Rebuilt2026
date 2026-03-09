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
import frc.robot.subsystems.IntakeRollerSubsystem;
// Commands
import frc.robot.commands.tuning.MoveLog;
import frc.robot.commands.atomic.*;
import frc.robot.commands.coordinators.*;

// Constants
import frc.robot.Constants.kAgitator;
import frc.robot.Constants.kIntake;

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

    private final SubsystemsRecord record;
    private final CoordinatorRecord coordinator;
    private final double MaxSpeed;
    private final double MaxAngularRate;

    private Double nextSpeedOverride = null;
    private Double nextAngularOverride = null;

    private final Timer autoTimer = new Timer();

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
    public AutoSequence(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {
        this.record = record;
        this.coordinator = coordinator;
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

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.moveToPose(
            record.swerve(),
            target,
            speedToUse,
            angularToUse
        ));

        return this;
    }

     public AutoSequence driveDistance(double target) {

        double speedToUse = (nextSpeedOverride != null)
            ? nextSpeedOverride
            : MaxSpeed;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.driveForwardMeters(
            record.swerve(),
            target,
            speedToUse,
            false
        ));

        return this;
    }
    
    public AutoSequence driveTo(Pose2d target) {

        double speedToUse = (nextSpeedOverride != null)
            ? nextSpeedOverride
            : MaxSpeed;

        double angularToUse = (nextAngularOverride != null)
            ? nextAngularOverride
            : MaxAngularRate;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.driveToPoint(
            record.swerve(),
            target,
            speedToUse,
            false
        ));

        return this;
    }

    /**
     * Adds a movement step that drives the robot *through* the specified pose
     * without slowing down or attempting to stop precisely on the point.
     *
     * <p>This is useful for path shaping, waypoint passing, and smooth motion
     * through intermediate nodes where stopping is not desired.</p>
     *
     * <p>The command ends when the robot comes within the configured pass-through
     * radius of the target pose, or when stall detection triggers.</p>
     *
     * <p><strong>Speed Overrides:</strong><br>
     * If {@link #withSpeed(double)} or {@link #withSpeed(double, double)} was
     * called immediately before this method, the override applies only to this
     * movement step. After the command is added, the override is cleared and the
     * default speed profile resumes.</p>
     *
     * @param target The pose the robot should pass near.
     * @param passRadius The distance (in meters) at which the robot is considered
     *                   to have passed the target.
     * @return This AutoSequence instance for fluent chaining.
     */
    public AutoSequence moveThrough(Pose2d target, double passRadius) {

        double speedToUse = (nextSpeedOverride != null)
            ? nextSpeedOverride
            : MaxSpeed;

        double angularToUse = (nextAngularOverride != null)
            ? nextAngularOverride
            : MaxAngularRate;

        // Angular rate is irrelevant for moveThrough, but we clear it anyway
        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(DriveCommands.moveThroughPose(
            record.swerve(),
            target,
            speedToUse,
            angularToUse,
            passRadius
        ));

        return this;
    }

    public AutoSequence hopperExtend() {
        steps.add(HopperCommands.extend(record.hopper()));
        return this;
    }

    public AutoSequence hopperRetract() {
        steps.add(HopperCommands.retract(record.hopper()));
        return this;
    }

    /**
     * This adds a Command that runs the intake to collect a game piece.
     */
    public AutoSequence intakeDeploy() {
        steps.add(
            IntakeSequences.deployAndRun(
                record.hopper(),
                record.intakeArm(),
                record.intakeRoller()
            )
        );
        return this;
    }

    /**
     * This adds a Command that retracts the intake arm.
     */
    public AutoSequence intakeRetract() {
        steps.add(
            IntakeSequences.stow(
                record.intakeArm(),
                record.intakeRoller()
            )
        );
        return this;
    }

    public AutoSequence intakeLow(){
        IntakeRollerCommands.lowRollerSpeed(record.intakeRoller());
        return this;
    }

    public AutoSequence intakeHigh(){
        IntakeRollerCommands.highRollerSpeed(record.intakeRoller());
        return this;
    }

    /**
     * This should add a Command that performs the robot's scoring routine.
     * Example:
     * steps.add(new ScoreCommand(shooterSubsystem));
     */
    public AutoSequence shoot() {
        steps.add(ShooterControllers.shootModelBased(
            coordinator, kAgitator.AGITATE_TO_SHOOTER_DUTY, kIntake.INTAKE_ROLLER_DUTY_HIGH
        ));
        return this;
    }

    /**
     * This will start shooting until an elapsed time has happened.
     * Auto runs for 15 seconds. So at the start of auto a timer
     * will begin counting. When the timer has reached endTime then
     * the shooting sequence will stop.
     * 
     * @param endTime the elapsed time to stop shooting
     */
    public AutoSequence shootUntil(double endTime) {
        steps.add(
            Commands.race(
                ShooterControllers.shootModelBased(
                    coordinator, kAgitator.AGITATE_TO_SHOOTER_DUTY, kIntake.INTAKE_ROLLER_DUTY_HIGH
                ),
                Commands.waitUntil(() -> autoTimer.get() >= endTime)
            )
        );
        return this;
    }

    public AutoSequence shootRPMUntil(double endTime, double RPM) {
        steps.add(
            Commands.race(
                ShooterControllers.shootFixedRPM(
                    coordinator, RPM, kAgitator.AGITATE_TO_SHOOTER_DUTY, kIntake.INTAKE_ROLLER_DUTY_HIGH
                ),
                Commands.waitUntil(() -> autoTimer.get() >= endTime)
            )
        );
        return this;
    }

    public AutoSequence pointToTarget() {
        steps.add(DriveCommands.pointToTarget(record.swerve(), record.BLUshooter()::getTargetPose));
        return this;
    }

    public AutoSequence pointToShoot() {
        steps.add(DriveCommands.pointToTarget(record.swerve(), record.BLUshooter()::getTargetPose));
        return this;
    }

    public AutoSequence headingToTarget(Pose2d targetPose) {
        steps.add(DriveCommands.pointToTarget(record.swerve(), () -> targetPose));
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

    public AutoSequence startTimer() {
        steps.add(Commands.runOnce(() -> {
            autoTimer.reset();
            autoTimer.start();
        }));
        return this;
    }

    public Command endAtTime(double endTime) {
        return Commands.waitUntil(() -> autoTimer.get() >= endTime);
    }

    public AutoSequence startLog(Pose2d targetPose) {
        steps.add(MoveLog.startLog(record.swerve(), targetPose));
        return this;
    }

    public AutoSequence endLog() {
        steps.add(MoveLog.endLog());
        return this;
    }

    public AutoSequence recordLog() {
        steps.add(MoveLog.record());
        return this;
    }

    public AutoSequence analyzeAllLogs() {
        steps.add(MoveLog.analyzeAllLogs());
        return this;
    }

    public AutoSequence logMoveTo(Pose2d target) {
        return this
        .startLog(target)
        .deadline(
            seq -> seq.moveTo(target),   // deadline
            seq -> seq.recordLog()       // runs until moveTo finishes
        )
        .endLog();
    }

    /**
     * Wait for a number of seconds before continuing.
     * This is already implemented for students.
     */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(Commands.waitSeconds(seconds));
        return this;
    }

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
            AutoSequence sub = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
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
            AutoSequence sub = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
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
        AutoSequence deadlineSeq = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
        deadlineBuilder.build(deadlineSeq);

        List<Command> otherCMD = new ArrayList<>();
        for (AutoSequenceBuilder builder : others) {
            AutoSequence sub = new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate);
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