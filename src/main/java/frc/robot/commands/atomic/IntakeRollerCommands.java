//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.atomic;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.lib.core.util.CommandBuilder;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.Constants.kIntake;


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
            .onInitialize(() -> roller.run())
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
    public static Command highRollerSpeed(IntakeRollerSubsystem roller) {
        return new CommandBuilder(roller) 
            .named("IntakeRollerSetDutyHigh")
            .onInitialize(()->roller.setDutyCycle(kIntake.INTAKE_ROLLER_DUTY_HIGH))
            .isFinished(true);
    }
    public static Command lowRollerSpeed(IntakeRollerSubsystem roller) {
        return new CommandBuilder(roller) 
            .named("IntakeRollerSetDutyLow")
            .onInitialize(()->roller.setDutyCycle(kIntake.INTAKE_ROLLER_DUTY_LOW))
            .isFinished(true);
    }
}
