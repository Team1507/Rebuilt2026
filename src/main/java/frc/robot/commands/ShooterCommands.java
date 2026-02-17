//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.CommandBuilder;

import frc.robot.subsystems.ShooterSubsystem;

public final class ShooterCommands {

    private ShooterCommands() {}

    // ------------------------------------------------------------
    // 1. FIXED‑RPM SHOOTING (ML training + pit testing)
    // ------------------------------------------------------------
    public static Command setRPM(ShooterSubsystem shooter, double rpm) {
        return new CommandBuilder(shooter)
            .named("ShooterSetRPM(" + rpm + ")")
            .onInitialize(() -> shooter.setTargetRPM(rpm))
            .isFinished(true);
    }

    // ------------------------------------------------------------
    // 2. MANUAL SHOOTER CONTROL (Elastic UI)
    // ------------------------------------------------------------
    public static Command manual(ShooterSubsystem shooter, Supplier<Double> rpmSupplier) {
        return new CommandBuilder(shooter)
            .named("ShooterManual")
            .onExecute(() -> shooter.setTargetRPM(rpmSupplier.get()))
            .onEnd(shooter::stop);
    }

    // ------------------------------------------------------------
    // 3. STOP SHOOTER IMMEDIATELY
    // ------------------------------------------------------------
    public static Command stop(ShooterSubsystem shooter) {
        return new CommandBuilder(shooter)
            .named("ShooterStop")
            .onInitialize(shooter::stop)
            .isFinished(true);
    }
}
