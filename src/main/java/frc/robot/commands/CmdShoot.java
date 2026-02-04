// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Shooter.SHOOTER_TOLERANCE;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdShoot extends Command {

  public final FeederSubsystem feederSubsystem;
  public final AgitatorSubsystem agitatorSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final double shooterTargetRPM;
  public final double feederTargetRPM;
  public final double agitatorTargetRPM;
  public boolean shooterReachedTarget = false;

  /** Creates a new CmdShoot. */
  public CmdShoot(double shooterRPM, double feederRPM, double agitatorRPM, AgitatorSubsystem agitatorSubsystem, FeederSubsystem feederSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.agitatorSubsystem = agitatorSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterTargetRPM = shooterRPM;
    this.feederTargetRPM = feederRPM;
    this.agitatorTargetRPM = agitatorRPM;

    addRequirements(agitatorSubsystem);
    addRequirements(feederSubsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterReachedTarget = false;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTargetRPM(shooterTargetRPM);

    //agitator runs once actual RPM is within a specific tolerance for desired RPM
    if(shooterSubsystem.getShooterRPM() >= shooterTargetRPM - SHOOTER_TOLERANCE && !shooterReachedTarget) {

      shooterReachedTarget = true;
    }

    if(shooterReachedTarget){
      feederSubsystem.setVelocityRPM(feederTargetRPM);
      agitatorSubsystem.setVelocityRPM(agitatorTargetRPM);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setVelocityRPM(0.0);
    agitatorSubsystem.setVelocityRPM(0.0);
    shooterSubsystem.setTargetRPM(2000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
