//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.framework;

import frc.robot.localization.QuestNavController;
import frc.robot.subsystems.*;

public record SubsystemsRecord(
    SwerveSubsystem swerve,
    AgitatorSubsystem agitator,
    FeederSubsystem BLUfeeder,
    FeederSubsystem YELfeeder,
    HopperSubsystem hopper,
    IntakeArmSubsystem intakeArm,
    IntakeRollerSubsystem intakeRoller,
    ShooterSubsystem BLUshooter,
    ShooterSubsystem YELshooter,
    QuestNavController questNavController
) {}
