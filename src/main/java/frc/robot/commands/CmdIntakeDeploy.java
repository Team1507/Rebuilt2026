// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Intake.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeDeploy extends Command {
  /** Creates a new CmdIntakeDeploy. */
  private final IntakeArmSubsystem intakeArmSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  public boolean deploy = false;

  public CmdIntakeDeploy(IntakeArmSubsystem intakeArmSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeArmSubsystem = intakeArmSubsystem;
    this.intakeSubsystem = intakeSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeArmSubsystem.setPosition(INTAKE_ARM_DEPLOYED_ANGLE_DEGREES);
    intakeSubsystem.run(.35);
    deploy = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.run(0);
    intakeArmSubsystem.setPosition(INTAKE_ARM_RETRACTED_ANGLE_DEGREES);
    deploy = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
