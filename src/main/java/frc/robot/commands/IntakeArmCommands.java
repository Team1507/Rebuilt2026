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
import frc.robot.subsystems.IntakeArmSubsystem;

import java.util.function.Supplier;

import static frc.robot.Constants.kIntake.kArm.*;

public final class IntakeArmCommands {

    private IntakeArmCommands() {}

    /** Move arm to the RETRACTED angle. */
    public static Command up(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmUp")
            .onExecute(() -> arm.setPosition(RETRACTED_ANGLE_DEGREES))
            .isFinished(() -> arm.isAtPosition(RETRACTED_ANGLE_DEGREES, 5.0))
            .onEnd(arm::stop);
    }

    /** Move arm to the DEPLOYED angle. */
    public static Command down(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmDown")
            .onExecute(() -> arm.setPosition(DEPLOYED_ANGLE_DEGREES))
            .isFinished(() -> arm.isAtPosition(DEPLOYED_ANGLE_DEGREES, 5.0))
            .onEnd(arm::stop);
    }

    /** Move arm to an arbitrary angle. */
    public static Command moveTo(IntakeArmSubsystem arm, double degrees) {
        return new CommandBuilder(arm)
            .named("IntakeArmMoveTo(" + degrees + ")")
            .onExecute(() -> arm.setPosition(degrees))
            .isFinished(() -> arm.isAtPosition(degrees, 5.0))
            .onEnd(arm::stop);
    }

    /** Manual control (Elastic UI / SmartDashboard). */
    public static Command manual(IntakeArmSubsystem arm, Supplier<Double> angleSupplier) {
        return new CommandBuilder(arm)
            .named("IntakeArmManual")
            .onExecute(() -> arm.setPosition(angleSupplier.get()))
            .onEnd(arm::stop);
    }

    /** Immediately stop the arm. */
    public static Command stop(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmStop")
            .onInitialize(arm::stop)
            .isFinished(true);
    }
}
