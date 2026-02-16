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

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;

import frc.robot.Constants.kAgitator;
import frc.robot.Constants.kFeeder;
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

    // -------------------------------------------------------------------------
    // MODEL-BASED SHOOTING (competition)
    // -------------------------------------------------------------------------
    public static Command shootModelBased(
        ShooterSubsystem shooterBLU,
        ShooterSubsystem shooterYEL,
        FeederSubsystem feederBLU,
        FeederSubsystem feederYEL,
        AgitatorSubsystem agitator
    ) {
        return new CommandBuilder(
            shooterBLU, shooterYEL, feederBLU, feederYEL, agitator
        )
        .named("ShootModelBased")
        .onExecute(() -> {

            shooterBLU.updateShooterFromModel();
            shooterYEL.updateShooterFromModel();

            if (ready(shooterBLU) && ready(shooterYEL)) {
                feederBLU.runRPM(kFeeder.FEED_RPM);
                feederYEL.runRPM(kFeeder.FEED_RPM);
                agitator.run(kAgitator.AGITATE_TO_SHOOTER_DUTY);
            }
        })
        .onEnd(interrupted -> {
            feederBLU.stop();
            feederYEL.stop();
            agitator.stop();
            shooterBLU.stop();
            shooterYEL.stop();
        });
    }

    // -------------------------------------------------------------------------
    // FIXED-RPM SHOOTING (ML training + pit testing)
    // -------------------------------------------------------------------------
    public static Command shootFixedRPM(
        ShooterSubsystem shooterBLU,
        ShooterSubsystem shooterYEL,
        FeederSubsystem feederBLU,
        FeederSubsystem feederYEL,
        AgitatorSubsystem agitator,
        double shooterRPMBLU,
        double shooterRPMYEL
    ) {
        return new CommandBuilder(
            shooterBLU, shooterYEL, feederBLU, feederYEL, agitator
        )
        .named("ShootFixedRPM")
        .onInitialize(() -> {
            shooterBLU.setTargetRPM(shooterRPMBLU);
            shooterYEL.setTargetRPM(shooterRPMYEL);
        })
        .onExecute(() -> {

            boolean bluReady = shooterBLU.getShooterRPM() >= shooterRPMBLU - TARGET_TOLERANCE;
            boolean yelReady = shooterYEL.getShooterRPM() >= shooterRPMYEL - TARGET_TOLERANCE;

            if (bluReady && yelReady) {
                feederBLU.runRPM(kFeeder.FEED_RPM);
                feederYEL.runRPM(kFeeder.FEED_RPM);
                agitator.run(kAgitator.AGITATE_TO_SHOOTER_DUTY);
            }
        })
        .onEnd(interrupted -> {
            feederBLU.stop();
            feederYEL.stop();
            agitator.stop();
            shooterBLU.stop();
            shooterYEL.stop();
        });
    }
}
