//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.agitate;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.AgitatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdAgitatorAgitate extends Command {

  public final AgitatorSubsystem agitatorSubsystem;
  public final double targetRPM;
  public boolean FeederFeeding = true;
  public CmdAgitatorAgitate(double RPM, AgitatorSubsystem agitatorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.agitatorSubsystem = agitatorSubsystem;
    this.targetRPM = RPM;

    addRequirements(agitatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    agitatorSubsystem.setVelocityRPM(targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    agitatorSubsystem.setVelocityRPM(targetRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    agitatorSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
