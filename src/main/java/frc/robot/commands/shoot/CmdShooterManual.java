// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdShooterManual extends Command {
  public final ShooterSubsystem shooterSubsystem;
  public final boolean manualMode;
  public final double targetRPM;
  /** Creates a new CmdShooterManual. */
  public CmdShooterManual(ShooterSubsystem shooterSubsystem, boolean manualMode, double targetRPM) {
    this.shooterSubsystem = shooterSubsystem;
    this.manualMode = manualMode;
    this.targetRPM = targetRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTargetRPM(targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !manualMode;
  }
}
