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
import frc.robot.Constants.kHopper;
import frc.robot.subsystems.HopperSubsystem;

import java.util.function.Supplier;

public final class HopperCommands {

    private HopperCommands() {}

    /** Move hopper to a specific angle (degrees). */
    public static Command moveTo(HopperSubsystem hopper, double degrees) {
        return new CommandBuilder(hopper)
            .named("HopperMoveTo(" + degrees + ")")
            .onExecute(() -> hopper.setPosition(degrees))
            .onEnd(hopper::stop);
    }

    /** Fully extend hopper (use your chosen angle). */
    public static Command extend(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperExtend")
            .onExecute(() -> hopper.setPosition(kHopper.LOAD_POS))
            .onEnd(hopper::stop);
    }

    /** Fully retract hopper (use your chosen angle). */
    public static Command retract(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperRetract")
            .onExecute(() -> hopper.setPosition(kHopper.SHOOT_POS))
            .onEnd(hopper::stop);
    }

    /** Manual hopper control (Elastic UI / SmartDashboard). */
    public static Command manual(HopperSubsystem hopper, Supplier<Double> angleSupplier) {
        return new CommandBuilder(hopper)
            .named("HopperManual")
            .onExecute(() -> hopper.setPosition(angleSupplier.get()))
            .onEnd(hopper::stop);
    }

    /** Immediately stop hopper motion. */
    public static Command stop(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperStop")
            .onInitialize(hopper::stop)
            .isFinished(true);
    }
}
