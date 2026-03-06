//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.coordinators;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.lib.core.util.CommandBuilder;
import frc.robot.framework.CoordinatorRecord;

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

    private static boolean shooterReady = false;

    private static boolean ready(ShooterSubsystem shooter) {
        return shooter.getShooterRPM() >= shooter.getTargetRPM() - TARGET_TOLERANCE;
    }

    private static boolean ready(ShooterSubsystem shooter, double targetRPM) {
        return shooter.getShooterRPM() >= targetRPM - TARGET_TOLERANCE;
    }

    // -------------------------------------------------------------------------
    // MODEL-BASED SHOOTING (competition)
    // -------------------------------------------------------------------------
    public static Command shootModelBased(
        CoordinatorRecord record, double agitatorDuty
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(), 
            record.BLUfeeder(), record.YELfeeder(),
            record.agitator()
        )
        .named("ShootModelBased")
        .onInitialize(() -> { shooterReady = false; })
        .onExecute(() -> {

            record.BLUshooter().updateShooterFromModel();
            record.YELshooter().updateShooterFromModel();

            if (ready(record.BLUshooter()) && ready(record.YELshooter())) {
                shooterReady = true;
                record.BLUfeeder().runRPM(record.BLUshooter().getShooterRPM() * 0.75);
                record.YELfeeder().runRPM(record.YELshooter().getShooterRPM() * 0.75);
            }

            if(shooterReady) {
                record.agitator().run(agitatorDuty);
            }
        })
        .onEnd(interrupted -> {
            record.BLUfeeder().stop();
            record.YELfeeder().stop();
            record.BLUshooter().stop();
            record.YELshooter().stop();
            shooterReady = false;
        });
    }

    // -------------------------------------------------------------------------
    // FIXED-RPM SHOOTING (ML training + pit testing)
    // -------------------------------------------------------------------------
    public static Command shootFixedRPM(
        CoordinatorRecord record,
        double shooterRPM
    ) {
        return new CommandBuilder(
            record.BLUshooter(), record.YELshooter(), 
            record.BLUfeeder(), record.YELfeeder()
        )
        .named("ShootFixedRPM")
        .onInitialize(() -> {
            record.BLUshooter().setTargetRPM(shooterRPM);
            record.YELshooter().setTargetRPM(shooterRPM);
        })
        .onExecute(() -> {
            if (ready(record.BLUshooter(), shooterRPM) && ready(record.YELshooter(), shooterRPM)) {
                record.BLUfeeder().runRPM(shooterRPM * 0.75);
                record.YELfeeder().runRPM(shooterRPM * 0.75);
            }
        })
        .onEnd(interrupted -> {
            record.BLUfeeder().stop();
            record.YELfeeder().stop();
            record.BLUshooter().stop();
            record.YELshooter().stop();
        });
    }
}
