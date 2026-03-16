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
import frc.robot.Constants.kAgitator;
import frc.robot.subsystems.AgitatorSubsystem;

import java.util.function.Supplier;

/**
 * Factory class for all Agitator commands.
 *
 * These commands are lightweight, allocation-free, and built using
 * the CommandBuilder to minimize cyclic load and maximize clarity.
 */
public final class 
AgitatorCommands {

    private AgitatorCommands() {} // static-only class

    /** Run agitator toward intake direction. */
    public static Command toIntake(AgitatorSubsystem agitator) {
        return new CommandBuilder(agitator)
            .named("AgitateToIntake")
            .onExecute(() -> agitator.run(kAgitator.AGITATE_TO_INTAKE_DUTY))
            .onEnd((interrupted, timedOut) -> agitator.stop());
    }

    /** Run agitator toward shooter direction. */
    public static Command toShooter(AgitatorSubsystem agitator) {
        return new CommandBuilder(agitator)
            .named("AgitateToShooter")
            .onExecute(() -> agitator.run(kAgitator.AGITATE_TO_SHOOTER_DUTY))
            .onEnd((interrupted, timedOut) -> agitator.stop());
    }

    /** Manual control of agitator using a duty-cycle supplier. */
    public static Command manual(AgitatorSubsystem agitator, Supplier<Double> dutySupplier) {
        return new CommandBuilder(agitator)
            .named("AgitatorManual")
            .onExecute(() -> agitator.run(dutySupplier.get()))
            .onEnd((interrupted, timedOut) -> agitator.stop());
    }
}
