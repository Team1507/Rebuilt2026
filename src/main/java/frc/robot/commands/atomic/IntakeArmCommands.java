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

    /** Move arm to the RETRACTED angle. */
    public static Command up(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmUp")
            .onExecute(() -> arm.setAngle(RETRACTED_ANGLE_DEGREES))
            .isFinished(() -> arm.isAtPosition(RETRACTED_ANGLE_DEGREES, 2))
            .timeout(1.0)
            .onEnd((interrupted, timedOut) -> {
                if (timedOut) {
                    // Mechanical fault → back off
                    arm.setAngle(DEPLOYED_ANGLE_DEGREES);
                }
                else {
                    // Driver release → do nothing
                    arm.stop();
                }
            });
    }

    /** Move arm to the DEPLOYED angle. */
    public static Command down(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmDown")
            .onExecute(() -> arm.setAngle(DEPLOYED_ANGLE_DEGREES))
            .isFinished(() -> arm.isAtPosition(DEPLOYED_ANGLE_DEGREES, 2.0))
            .timeout(1.0)
            .onEnd((interrupted, timedOut) -> {
                if (timedOut) {
                    // Mechanical fault → back off
                    arm.setAngle(RETRACTED_ANGLE_DEGREES);
                }
                else {
                    // Driver release → do nothing
                    arm.stop();
                }
            });
    }

    /** Move arm to an arbitrary angle. */
    public static Command moveTo(IntakeArmSubsystem arm, double degrees) {
        return new CommandBuilder(arm)
            .named("IntakeArmMoveTo(" + degrees + ")")
            .onExecute(() -> arm.setAngle(degrees))
            .isFinished(() -> arm.isAtPosition(degrees, 5.0))
            .onEnd((interrupted, timedOut) -> arm.stop());
    }

    /** Manual control (Elastic UI / SmartDashboard). */
    public static Command manualAngle(IntakeArmSubsystem arm, Supplier<Double> angleSupplier) {
        return new CommandBuilder(arm)
            .named("IntakeArmManualAngle")
            .onExecute(() -> arm.setAngle(angleSupplier.get()))
            .onEnd((interrupted, timedOut) -> arm.stop());
    }

    /** Manual control (Joystick)). */
    public static Command manualPower(IntakeArmSubsystem arm, Supplier<Double> angleSupplier) {
        return new CommandBuilder(arm)
            .named("IntakeArmManualPower")
            .onExecute(() -> arm.runPower(angleSupplier.get()))
            .onEnd((interrupted, timedOut) -> arm.stop());
    }

    /** Immediately stop the arm. */
    public static Command stop(IntakeArmSubsystem arm) {
        return new CommandBuilder(arm)
            .named("IntakeArmStop")
            .onInitialize(arm::stop)
            .isFinished(true);
    }
    
}
