//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.atomic;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.CommandBuilder;
import frc.robot.subsystems.IntakeArmSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.kIntake.kArm.*;

public final class IntakeArmCommands {

    private IntakeArmCommands() {}

    /** Move arm to the RETRACTED angle (stow). */
    public static Command up(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmUp")
            .onExecute(() -> arm.setAngle(RETRACTED_ANGLE_DEGREES))

            // Normal completion
            .isFinished(() -> arm.isAtPosition(RETRACTED_ANGLE_DEGREES, 2))

            // Stall detection
            .stallFinish(arm::isStalled)

            // Hard timeout fallback
            .timeout(2.0)

            // Unified end handler
            .onEnd((interrupted, timedOut, stalled) -> {
                arm.stop();

                if (stalled || timedOut) {
                    // Mechanical fault → back off to deployed
                    arm.setAngle(DEPLOYED_ANGLE_DEGREES);
                }
                // Driver release → do nothing (stow command will take over)
            });
    }

    /** Move arm to the DEPLOYED angle. */
    public static Command down(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmDown")
            .onExecute(() -> arm.setAngle(DEPLOYED_ANGLE_DEGREES))

            // Normal completion
            .isFinished(() -> arm.isAtPosition(DEPLOYED_ANGLE_DEGREES, 2.0))

            // Stall detection
            .stallFinish(arm::isStalled)

            // Hard timeout fallback
            .timeout(2.0)

            .onEnd((interrupted, timedOut, stalled) -> {
                arm.stop();

                if (stalled || timedOut) {
                    // Mechanical fault → back off to retracted
                    arm.setAngle(RETRACTED_ANGLE_DEGREES);
                }
                // Driver release → do nothing
            });
    }

    /** Move arm to an arbitrary angle. */
    public static Command moveTo(IntakeArmSubsystem arm, double degrees) {
        return new CommandBuilder(arm)
            .named("IntakeArmMoveTo(" + degrees + ")")
            .onExecute(() -> arm.setAngle(degrees))

            .isFinished(() -> arm.isAtPosition(degrees, 5.0))

            // Optional: stall detection for arbitrary moves
            .stallFinish(arm::isStalled)

            .onEnd((interrupted, timedOut, stalled) -> {
                arm.stop();
                // For generic moves, we do NOT auto‑recover.
                // Caller decides what to do.
            });
    }

    /** Manual control (Elastic UI / SmartDashboard). */
    public static Command manualAngle(IntakeArmSubsystem arm, Supplier<Double> angleSupplier) {
        return new CommandBuilder(arm)
            .named("IntakeArmManualAngle")
            .onExecute(() -> arm.setAngle(angleSupplier.get()))
            .stallFinish(arm::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> arm.stop());
    }

    /** Manual control (Joystick). */
    public static Command manualPower(IntakeArmSubsystem arm, Supplier<Double> powerSupplier) {
        return new CommandBuilder(arm)
            .named("IntakeArmManualPower")
            .onExecute(() -> arm.runPower(powerSupplier.get()))
            .stallFinish(arm::isStalled)
            .onEnd((interrupted, timedOut, stalled) -> arm.stop());
    }

    /** Immediately stop the arm. */
    public static Command stop(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmStop")
            .onInitialize(arm::stop)
            .isFinished(true);
    }
}