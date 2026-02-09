package frc.robot.utilities;

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
     ShooterSubsystem shooter
) {}
