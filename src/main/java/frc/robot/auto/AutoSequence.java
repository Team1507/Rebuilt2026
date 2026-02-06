//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * AutoSequence
 *
 * A fluent, subsystem‑agnostic builder for autonomous routines. This class
 * defines the *structure* of an autonomous routine, while the *implementation*
 * of each action is supplied externally through an {@link AutoCapabilities}
 * object created in RobotContainer.
 *
 * <p>Each call to a high‑level action (moveTo, intake, score, waitSeconds)
 * appends a {@code Supplier<Command>} to an internal list. When {@link #build()}
 * is invoked, these suppliers are resolved into a sequential command group.</p>
 *
 * <p>This architecture cleanly separates:
 * <ul>
 *   <li>Student‑authored autonomous scripts</li>
 *   <li>Subsystem wiring and command construction</li>
 * </ul>
 * and ensures AutoSequence remains reusable, testable, and easy to extend.</p>
 */
public class AutoSequence {

    /** Ordered list of command factories representing the autonomous steps. */
    private final List<Supplier<Command>> steps = new ArrayList<>();

    /** Capabilities describing which actions are available. */
    private final AutoCapabilities caps;

    /** Optional per-step speed overrides (applied only to movement commands). */
    private Double nextSpeedOverride = null;
    private Double nextAngularOverride = null;

    /** Default speed profile for movement commands. */
    private final double defaultSpeed;
    private final double defaultAngular;

    /**
     * Creates a new AutoSequence builder using the provided capabilities.
     *
     * @param caps            The set of available autonomous actions.
     * @param defaultSpeed    Default translational speed for movement.
     * @param defaultAngular  Default rotational speed for movement.
     *
     * <p>RobotContainer is responsible for constructing the {@link AutoCapabilities}
     * instance and wiring each action to the appropriate subsystem.</p>
     */
    public AutoSequence(AutoCapabilities caps, double defaultSpeed, double defaultAngular) {
        this.caps = caps;
        this.defaultSpeed = defaultSpeed;
        this.defaultAngular = defaultAngular;
    }

    /**
     * Applies a temporary translational speed override for the next movement
     * command only.
     */
    public AutoSequence withSpeed(double speed) {
        return withSpeed(speed, defaultAngular);
    }

    /**
     * Applies temporary translational and rotational speed overrides for the
     * next movement command only.
     */
    public AutoSequence withSpeed(double speed, double angularRate) {
        this.nextSpeedOverride = speed;
        this.nextAngularOverride = angularRate;
        return this;
    }

    /**
     * Adds a movement step that drives the robot to the specified target pose.
     *
     * @param target  The desired end pose for the robot.
     * @return        This AutoSequence for fluent chaining.
     */
    public AutoSequence moveTo(Pose2d target) {

        if (caps.moveTo() == null)
            throw new IllegalStateException("This AutoSequence does not support movement.");

        double speed = nextSpeedOverride != null ? nextSpeedOverride : defaultSpeed;
        double angular = nextAngularOverride != null ? nextAngularOverride : defaultAngular;

        nextSpeedOverride = null;
        nextAngularOverride = null;

        steps.add(() -> caps.moveTo().apply(target));
        return this;
    }

    /**
     * Adds an intake action to the autonomous sequence.
     */
    public AutoSequence intake() {
        if (caps.intake().isEmpty())
            throw new IllegalStateException("This AutoSequence does not support intake.");

        steps.add(caps.intake().get());
        return this;
    }

    /**
     * Adds a scoring action to the autonomous sequence.
     */
    public AutoSequence score() {
        if (caps.score().isEmpty())
            throw new IllegalStateException("This AutoSequence does not support scoring.");

        steps.add(caps.score().get());
        return this;
    }

    /**
     * Adds a timed wait action to the autonomous sequence.
     */
    public AutoSequence waitSeconds(double seconds) {
        steps.add(() -> caps.waitSeconds().apply(seconds));
        return this;
    }

    /**
     * Builds the final autonomous command by resolving all stored suppliers
     * into a sequential command group.
     */
    public Command build() {
        return Commands.sequence(
            steps.stream()
                 .map(Supplier::get)
                 .toArray(Command[]::new)
        );
    }
}
