// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import static frc.robot.Constants.Intake.INTAKE_ARM_RETRACTED_ANGLE_DEGREES;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.IntakeArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeArmUp extends Command {

public final IntakeArmSubsystem intakeArmSubsystem;

  /** Creates a new CmdIntakeArmUp. */
  public CmdIntakeArmUp(IntakeArmSubsystem intakeArmSubsystem ) {
    this.intakeArmSubsystem = intakeArmSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeArmSubsystem.setPosition(INTAKE_ARM_RETRACTED_ANGLE_DEGREES);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
