// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdSetShooterRPM extends Command {

  private final ShooterSubsystem shooterSubsystem;
  private final double shooterRPM;
  /** Creates a new CmdSetShooterRPM. */
  public CmdSetShooterRPM(ShooterSubsystem shooterSubsystem, double shooterRPM) {
    // Use addRequirements() here to declare subsystem dependencies.

    // maps the inputs to the local variables
    this.shooterSubsystem = shooterSubsystem;
    this.shooterRPM = shooterRPM;

    //can't call this system once it is running
    addRequirements(shooterSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set shooter RPM
    shooterSubsystem.setTargetRPM(shooterRPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
