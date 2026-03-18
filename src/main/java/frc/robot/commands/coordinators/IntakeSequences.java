//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.coordinators;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

// Subsystems
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;

// Commands
import frc.robot.commands.atomic.HopperCommands;
import frc.robot.commands.atomic.IntakeArmCommands;
import frc.robot.commands.atomic.IntakeRollerCommands;

public final class IntakeSequences {

    private IntakeSequences() {}

    public static Command deployAndRun( 
        HopperSubsystem hopper,
        IntakeArmSubsystem arm,
        IntakeRollerSubsystem roller
    ) {
        return HopperCommands.extend(hopper)
            //.alongWith(IntakeRollerCommands.stop(roller)
            .alongWith(
                Commands.waitUntil(hopper::isHopperSafeForIntake)
                    .andThen(
                        IntakeArmCommands.down(arm)
                            .alongWith(IntakeRollerCommands.intake(roller))
                    )
            );
    }

    public static Command stow(
        IntakeArmSubsystem arm,
        IntakeRollerSubsystem roller
    ) {
        return IntakeArmCommands.up(arm)
            .alongWith(IntakeRollerCommands.stop(roller));
    }
}



