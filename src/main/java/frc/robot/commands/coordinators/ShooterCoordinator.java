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

import static frc.robot.Constants.kShooter.TARGET_TOLERANCE;

/**
 * Coordinates BOTH shooters, BOTH feeders, and the shared agitator.
 *
 * This is the ONLY place where cross-subsystem shooting logic lives.
 * Subsystem-specific commands (ShooterCommands, FeederCommands, AgitatorCommands)
 * remain simple and atomic.
 */
public final class ShooterCoordinator {

    private ShooterCoordinator() {}

    private static boolean ready(ShooterSubsystem shooter) {
        return shooter.getShooterRPM() >= shooter.getTargetRPM() - TARGET_TOLERANCE;
    }

    private static boolean bothShootersReady(
        ShooterSubsystem blu,
        ShooterSubsystem yel
    ) {
        return ready(blu) && ready(yel);
    }

    // -------------------------------------------------------------------------
    // MODEL-BASED SHOOTING (competition)
    // -------------------------------------------------------------------------
    public static Command shootModelBased(
        CoordinatorRecord record,
        double agitatorDuty
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(),
            record.BLUfeeder(), record.YELfeeder(),
            record.agitator()
        )
        .named("ShootModelBased")
        .onExecute(new Runnable() {
            private boolean feedingEnabled = false;

            @Override
            public void run() {
                // 1. Update shooter targets from model
                record.BLUshooter().updateShooterFromModel();
                record.YELshooter().updateShooterFromModel();

                // 2. Feeders spin continuously at a stable RPM
                double bluFeederRPM = record.BLUshooter().getTargetRPM() * 0.75;
                double yelFeederRPM = record.YELshooter().getTargetRPM() * 0.75;

                record.BLUfeeder().runRPM(bluFeederRPM);
                record.YELfeeder().runRPM(yelFeederRPM);

                // 3. Latch feeding once shooters are ready
                if (!feedingEnabled &&
                    bothShootersReady(record.BLUshooter(), record.YELshooter())) {
                    feedingEnabled = true;
                }

                // 4. Agitator runs once feeding is enabled
                if (feedingEnabled) {
                    record.agitator().run(agitatorDuty);
                }
            }
        })
        .onEnd(interrupted -> {
            record.BLUfeeder().stop();
            record.YELfeeder().stop();
            record.BLUshooter().stop();
            record.YELshooter().stop();
            record.agitator().stop();
        });
    }

    // -------------------------------------------------------------------------
    // FIXED-RPM SHOOTING (ML training + pit testing)
    // -------------------------------------------------------------------------
    public static Command shootFixedRPM(
        CoordinatorRecord record,
        double shooterRPM,
        double agitatorDuty
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(),
            record.BLUfeeder(), record.YELfeeder(),
            record.agitator()
        )
        .named("ShootFixedRPM")
        .onInitialize(() -> {
            record.BLUshooter().setTargetRPM(shooterRPM);
            record.YELshooter().setTargetRPM(shooterRPM);
        })
        .onExecute(new Runnable() {
            private boolean feedingEnabled = false;

            @Override
            public void run() {
                // Feeders always spin once command starts
                record.BLUfeeder().runRPM(
                    record.BLUshooter().getTargetRPM() * 0.75
                );
                record.YELfeeder().runRPM(
                    record.YELshooter().getTargetRPM() * 0.75
                );

                // Latch feeding once both shooters are ready
                if (!feedingEnabled && bothShootersReady(record.BLUshooter(), record.YELshooter())) {
                    feedingEnabled = true;
                    record.agitator().run(agitatorDuty);
                }
            }
        })
        .onEnd(interrupted -> {
            // Stop Blue side
            record.BLUshooter().stop(); record.BLUfeeder().stop();
            // Stop Yellow Side
            record.YELshooter().stop(); record.YELfeeder().stop();
            // Stop Agitator
            record.agitator().stop();
        });
    }
}
