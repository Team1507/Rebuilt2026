// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeArmManual extends Command {
  public final IntakeArmSubsystem intakeArmSubsystem;
  public final boolean manualMode;
  public final double targetAngle;
  /** Creates a new CmdIntakeArmManual. */
  public CmdIntakeArmManual(IntakeArmSubsystem intakeArmSubsystem, boolean manualMode, double targetAngle) {
    this.intakeArmSubsystem = intakeArmSubsystem;
    this.manualMode = manualMode;
    this.targetAngle = targetAngle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeArmSubsystem.setPosition(targetAngle);
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
