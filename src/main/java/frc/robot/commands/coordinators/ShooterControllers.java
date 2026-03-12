//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.coordinators;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.core.util.CommandBuilder;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.subsystems.ShooterSubsystem;

import static frc.robot.Constants.kIntake.INTAKE_ROLLER_DUTY_IDLE;
import static frc.robot.Constants.kShooter.TARGET_TOLERANCE;

/**
 * Coordinates BOTH shooters, BOTH feeders, and the shared agitator.
 *
 * This is the ONLY place where cross-subsystem shooting logic lives.
 * Subsystem-specific commands (ShooterCommands, FeederCommands, AgitatorCommands)
 * remain simple and atomic.
 */
public final class ShooterControllers {

    private ShooterControllers() {}

    private static boolean ready(ShooterSubsystem shooter) {
        return shooter.getShooterRPM() >= shooter.getTargetRPM() - TARGET_TOLERANCE;
    }

    private static boolean bothShootersReady(
        ShooterSubsystem blu,
        ShooterSubsystem yel
    ) {
        return ready(blu) && ready(yel);
    }

    private static boolean oneShootersReady(
        ShooterSubsystem blu,
        ShooterSubsystem yel
    ) {
        return ready(blu) || ready(yel);
    }

    // -------------------------------------------------------------------------
    // MODEL-BASED SHOOTING (competition)
    // -------------------------------------------------------------------------
    public static Command shootModelBased(
        CoordinatorRecord record,
        double agitatorDuty,
        double intakeRoller
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(),
            record.BLUfeeder(), record.YELfeeder(),
            record.agitator(),
            record.roller()
        )
        .named("ShootModelBased")
        .onExecute(() -> {
            // 1. Update shooter targets from model
            record.BLUshooter().updateShooterFromModel();
            record.YELshooter().updateShooterFromModel();

            // 2. Feeders spin continuously at a stable RPM
            double bluFeederRPM = record.BLUshooter().getTargetRPM() * 0.75;
            double yelFeederRPM = record.YELshooter().getTargetRPM() * 0.75;

            
            record.roller().run(intakeRoller);

            // 3. Latch feeding once shooters are ready
            if (!record.agitator().getInputs().feedingEnabled &&
                bothShootersReady(record.BLUshooter(), record.YELshooter())) {
                record.agitator().setFeeding();
            }

            // 4. Agitator runs once feeding is enabled
            if (record.agitator().getInputs().feedingEnabled) {
                record.agitator().run(agitatorDuty);
                record.BLUfeeder().runRPM(bluFeederRPM);
                record.YELfeeder().runRPM(yelFeederRPM);
            }
        })
        .onEnd(interrupted -> {
            // Stop Blue side
            record.BLUshooter().stop(); record.BLUfeeder().stop();
            // Stop Yellow Side
            record.YELshooter().stop(); record.YELfeeder().stop();
            // Stop Agitator
            record.agitator().stop();
            // Resetting feedingEnabled for the agitator
            record.agitator().resetFeeding();

            record.roller().run(INTAKE_ROLLER_DUTY_IDLE);
        });
    }

    // -------------------------------------------------------------------------
    // FIXED-RPM SHOOTING (ML training + pit testing)
    // -------------------------------------------------------------------------
    public static Command shootFixedRPM(
        CoordinatorRecord record,
        double shooterRPM,
        double agitatorDuty,
        double intakeRoller
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(),
            record.BLUfeeder(), record.YELfeeder(),
            record.agitator(),
            record.roller()
        )
        .named("ShootFixedRPM")
        .onInitialize(() -> {
            record.BLUshooter().setTargetRPM(shooterRPM);
            record.YELshooter().setTargetRPM(shooterRPM);
            record.roller().run(intakeRoller);
        })
        .onExecute(() -> {
            // Feeders always spin once command starts
            double bluFeederRPM = record.BLUshooter().getTargetRPM() * 0.75;
            double yelFeederRPM = record.YELshooter().getTargetRPM() * 0.75;

            // Latch feeding once shooters are ready
            if (!record.agitator().getInputs().feedingEnabled &&
                oneShootersReady(record.BLUshooter(), record.YELshooter())) {
                record.agitator().setFeeding();
            }

            // Agitator runs once feeding is enabled
            if (record.agitator().getInputs().feedingEnabled) {
                record.agitator().run(agitatorDuty);
                record.BLUfeeder().runRPM(bluFeederRPM);
                record.YELfeeder().runRPM(yelFeederRPM);
            }
        })
        .onEnd(interrupted -> {
            // Stop Blue side
            record.BLUshooter().stop(); record.BLUfeeder().stop();
            // Stop Yellow Side
            record.YELshooter().stop(); record.YELfeeder().stop();
            // Stop Agitator
            record.agitator().stop();
            // Resetting feedingEnabled for the agitator
            record.agitator().resetFeeding();

            record.roller().run(INTAKE_ROLLER_DUTY_IDLE);
        });
    }
}
