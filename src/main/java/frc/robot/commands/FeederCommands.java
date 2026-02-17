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
import frc.robot.Constants.kFeeder;
import frc.robot.subsystems.FeederSubsystem;

import java.util.function.Supplier;

/**
 * Factory class for all Feeder commands.
 */
public final class FeederCommands {

    private FeederCommands() {}

    /** Run feeder forward at FEED_RPM. */
    public static Command feed(FeederSubsystem feeder) {
        return new CommandBuilder(feeder)
            .named("FeederFeed")
            .onExecute(() -> feeder.runRPM(kFeeder.FEED_RPM))
            .onEnd(feeder::stop);
    }

    /** Run feeder backward at VOMIT_RPM. */
    public static Command vomit(FeederSubsystem feeder) {
        return new CommandBuilder(feeder)
            .named("FeederVomit")
            .onExecute(() -> feeder.runRPM(kFeeder.VOMIT_RPM))
            .onEnd(feeder::stop);
    }

    /** Manual feeder control using a supplier (RPM from dashboard or joystick). */
    public static Command manual(FeederSubsystem feeder, Supplier<Double> rpmSupplier) {
        return new CommandBuilder(feeder)
            .named("FeederManual")
            .onExecute(() -> feeder.runRPM(rpmSupplier.get()))
            .onEnd(feeder::stop);
    }
}
