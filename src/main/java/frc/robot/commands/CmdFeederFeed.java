// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Feeder;
import frc.robot.subsystems.FeederSubsystem;  

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdFeederFeed extends Command {
 
  public final FeederSubsystem feederSubsystem;
  public final double targetRPM;
  public boolean FeederFeeding = true;
  public CmdFeederFeed(double RPM, FeederSubsystem feederSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.feederSubsystem = feederSubsystem;
    this.targetRPM = RPM;

    addRequirements(feederSubsystem);
    
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    feederSubsystem.setVelocityRPM(targetRPM);
    SmartDashboard.putNumber("Feeder RPM", feederSubsystem.getVelocityRPM());
    SmartDashboard.putBoolean("FeederFeeding", FeederFeeding);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederSubsystem.setVelocityRPM(0);
    SmartDashboard.putBoolean("FeederFeeding", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
