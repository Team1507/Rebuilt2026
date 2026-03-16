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
            .onEnd((interrupted, timedOut) -> hopper.stop());
    }

    /** Fully extend hopper (use your chosen angle). */
    public static Command extend(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperExtend")
            .onExecute(() -> hopper.setPosition(kHopper.EXTENDED_POS))
            .isFinished(hopper::isHopperFullyExtended)
            .onEnd((interrupted, timedOut) -> hopper.stop());
    }

    /** Fully retract hopper (use your chosen angle). */
    public static Command retract(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperRetract")
            .onExecute(() -> hopper.setPosition(kHopper.RETRACTED_POS))
            .onEnd((interrupted, timedOut) -> hopper.stop());

    }

    /** Manual hopper control (Elastic UI / SmartDashboard). */
    public static Command manualPosition(HopperSubsystem hopper, Supplier<Double> position) {
        return new CommandBuilder(hopper)
            .named("HopperManual")
            .onExecute(() -> hopper.setPosition(position.get()))
            .onEnd((interrupted, timedOut) -> hopper.stop());
    }

    /** Manual hopper control (Joystick). */
    public static Command manualPower(HopperSubsystem hopper, Supplier<Double> power) {
        return new CommandBuilder(hopper)
            .named("HopperManual")
            .onExecute(() -> hopper.runPower(power.get()))
            .onEnd((interrupted, timedOut) -> hopper.stop());
    }

    /** Immediately stop hopper motion. */
    public static Command stop(HopperSubsystem hopper) {
        return new CommandBuilder(hopper)
            .named("HopperStop")
            .onInitialize(hopper::stop)
            .isFinished(true);
    }
}
