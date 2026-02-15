//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.utilities;

import frc.robot.localization.PhotonVision.PVManager;
import frc.robot.localization.quest.QuestNavSubsystem;
import frc.robot.subsystems.*;

public record SubsystemsRecord(
    CommandSwerveDrivetrain drivetrain,
    AgitatorSubsystem agitator,
    ClimberSubsystem climber,
    FeederSubsystem BLUfeeder,
    FeederSubsystem YELfeeder,
    HopperSubsystem hopper,
    IntakeArmSubsystem intakeArm,
    IntakeRollerSubsystem intakeRoller,
    ShooterSubsystem BLUshooter,
    ShooterSubsystem YELshooter,
    PVManager pvManager,
    QuestNavSubsystem questNav
) {}
