// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feed;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdFeederManual extends Command {
  public final FeederSubsystem feederSubsystem;
  public final boolean manualMode;
  public final double targetRPM;  
  /** Creates a new CmdFeederManual. */
  public CmdFeederManual(FeederSubsystem feederSubsystem, boolean manualMode, double targetRPM) {
    this.feederSubsystem = feederSubsystem;
    this.manualMode = manualMode;
    this.targetRPM = targetRPM;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feederSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.run(targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !manualMode;
  }
}
