//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.shoot;

import edu.wpi.first.wpilibj2.command.Command;

// Subsystems
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// Constants
import static frc.robot.Constants.kShooter.TARGET_TOLERANCE;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdShoot extends Command {

  public final FeederSubsystem feederBLUsystem;
  public final FeederSubsystem feederYELsystem;
  public final AgitatorSubsystem agitatorSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final double shooterTargetRPM;
  public final double feederTargetRPM;
  public final double agitatorTargetRPM;
  public boolean shooterReachedTarget = false;

  /** Creates a new CmdShoot. */
  public CmdShoot(double shooterRPM, double feederRPM, double agitatorRPM, AgitatorSubsystem agitatorSubsystem, FeederSubsystem feederBLUsystem, FeederSubsystem feederYELsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.agitatorSubsystem = agitatorSubsystem;
    this.feederBLUsystem = feederBLUsystem;
    this.feederYELsystem = feederYELsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.shooterTargetRPM = shooterRPM;
    this.feederTargetRPM = feederRPM;
    this.agitatorTargetRPM = agitatorRPM;

    addRequirements(agitatorSubsystem);
    addRequirements(feederBLUsystem);
    addRequirements(feederYELsystem);
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterReachedTarget = false;
  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setTargetRPM(shooterTargetRPM);

    //agitator runs once actual RPM is within a specific tolerance for desired RPM
    if(shooterSubsystem.getShooterRPM() >= shooterTargetRPM - TARGET_TOLERANCE && !shooterReachedTarget) {

      shooterReachedTarget = true;
    }

    if(shooterReachedTarget){
      feederBLUsystem.setVelocityRPM(feederTargetRPM);
      feederYELsystem.setVelocityRPM(feederTargetRPM);
      agitatorSubsystem.setVelocityRPM(agitatorTargetRPM);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feederBLUsystem.setVelocityRPM(0.0);
    feederYELsystem.setVelocityRPM(0.0);
    agitatorSubsystem.setVelocityRPM(0.0);
    shooterSubsystem.setTargetRPM(2000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
