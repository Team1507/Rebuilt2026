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

import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.kClimber;

import java.util.function.Supplier;

public final class ClimberCommands {

    private ClimberCommands() {}

    // ------------------------------------------------------------
    // Manual control (Elastic UI / SmartDashboard)
    // ------------------------------------------------------------
    public static Command manual(ClimberSubsystem climber, Supplier<Double> positionSupplier) {
        return new CommandBuilder(climber)
            .named("ClimberManual")
            .onExecute(() -> climber.setPosition(positionSupplier.get()))
            .onEnd(() -> climber.setPosition(climber.getPosition()));
    }

    // ------------------------------------------------------------
    // Atomic position commands
    // ------------------------------------------------------------
    public static Command setPosition(ClimberSubsystem climber, double position) {
        return new CommandBuilder(climber)
            .named("ClimberSetPosition(" + position + ")")
            .onInitialize(() -> climber.setPosition(position))
            .isFinished(() -> climber.isAtPosition());
    }

    // ------------------------------------------------------------
    // Convenience commands for robot up/down
    // ------------------------------------------------------------
    public static Command robotUp(ClimberSubsystem climber) {
        return setPosition(climber, kClimber.ROBOT_UP);
    }

    public static Command robotDown(ClimberSubsystem climber) {
        return setPosition(climber, kClimber.ROBOT_DOWN);
    }

    // ------------------------------------------------------------
    // Servo control (ratchet lock)
    // ------------------------------------------------------------
    public static Command setServo(ClimberSubsystem climber, double servoPos) {
        return new CommandBuilder(climber)
            .named("ClimberSetServo(" + servoPos + ")")
            .onInitialize(() -> climber.setServo(servoPos))
            .isFinished(true);
    }

    // Optional convenience helpers
    public static Command lockRatchet(ClimberSubsystem climber) {
        return setServo(climber, 1.0); // adjust as needed
    }

    public static Command unlockRatchet(ClimberSubsystem climber) {
        return setServo(climber, 0.0); // adjust as needed
    }

    // ------------------------------------------------------------
    // Stop command
    // ------------------------------------------------------------
    public static Command stop(ClimberSubsystem climber) {
        return new CommandBuilder(climber)
            .named("ClimberStop")
            .onInitialize(() -> climber.setPosition(climber.getPosition()))
            .isFinished(true);
    }
}
