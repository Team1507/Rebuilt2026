package frc.robot.utilities;

import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public record SubsystemsRecord(
     CommandSwerveDrivetrain drivetrain,
     AgitatorSubsystem agitator,
     ClimberSubsystem climber,
     FeederSubsystem BLUfeeder,
     FeederSubsystem YELfeeder,
     HopperSubsystem hopper,
     IntakeArmSubsystem intakeArm,
     IntakeSubsystem intakeRoller,
     ShooterSubsystem shooter
) {
    
}
