// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kIntake;
import frc.robot.subsystems.IntakeRollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeRollerOuttake extends Command {
  public final IntakeRollerSubsystem intakeRollerSubsystem;
  /** Creates a new CmdIntakeRollerOuttake. */
  public CmdIntakeRollerOuttake(IntakeRollerSubsystem intakeRollerSubsystem) {
    this.intakeRollerSubsystem = intakeRollerSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeRollerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeRollerSubsystem.run(kIntake.OUTTAKE_ROLLER_DUTY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
