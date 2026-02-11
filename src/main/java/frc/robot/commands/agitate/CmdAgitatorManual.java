// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.agitate;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AgitatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdAgitatorManual extends Command {
  public final AgitatorSubsystem agitatorSubsystem;
  public final double dutyCycle;
  public final boolean manualMode;
  /** Creates a new CmdAgitatorManual. */
  public CmdAgitatorManual(AgitatorSubsystem agitatorSubsystem, double dutyCycle, boolean manualMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.agitatorSubsystem = agitatorSubsystem;
    this.dutyCycle = dutyCycle;
    this.manualMode = manualMode;

    addRequirements(agitatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    agitatorSubsystem.run(dutyCycle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    agitatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !manualMode;
  }
}
