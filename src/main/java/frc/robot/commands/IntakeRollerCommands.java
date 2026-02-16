//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CommandBuilder;
import frc.robot.Constants.kIntake;
import frc.robot.subsystems.IntakeRollerSubsystem;

import java.util.function.Supplier;

/**
 * Factory class for all Intake Roller commands.
 *
 * These commands are lightweight, allocation-free, and built using
 * the CommandBuilder to minimize cyclic load and maximize clarity.
 */
public final class IntakeRollerCommands {

    private IntakeRollerCommands() {}

    /** Run roller forward to intake game pieces. */
    public static Command intake(IntakeRollerSubsystem roller) {
        return new CommandBuilder(roller)
            .named("IntakeRollerIntake")
            .onInitialize(() -> roller.run(kIntake.INTAKE_ROLLER_DUTY))
            .isFinished(true); // one-shot command
    }

    /** Run roller backward to eject game pieces. */
    public static Command outtake(IntakeRollerSubsystem roller) {
        return new CommandBuilder(roller)
            .named("IntakeRollerOuttake")
            .onInitialize(() -> roller.run(kIntake.OUTTAKE_ROLLER_DUTY))
            .isFinished(true);
    }

    /** Manual roller control (Elastic UI / SmartDashboard). */
    public static Command manual(IntakeRollerSubsystem roller, Supplier<Double> dutySupplier) {
        return new CommandBuilder(roller)
            .named("IntakeRollerManual")
            .onExecute(() -> roller.run(dutySupplier.get()))
            .onEnd(roller::stop);
    }

    /** Immediately stop the roller. */
    public static Command stop(IntakeRollerSubsystem roller) {
        return new CommandBuilder(roller)
            .named("IntakeRollerStop")
            .onInitialize(roller::stop)
            .isFinished(true);
    }
}
