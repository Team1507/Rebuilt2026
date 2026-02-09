//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.climb;

// WPI Lib
import edu.wpi.first.wpilibj2.command.Command;

// Subsystems
import frc.robot.subsystems.ClimberSubsystem;

// Constants
import frc.robot.Constants.kClimber;

public class CmdClimberRobotUp extends Command {
    private final ClimberSubsystem climberSubsystem;

    /** Creates a new CmdClimberClimb. */
    public CmdClimberRobotUp(ClimberSubsystem climberSubsystem ) {
        this.climberSubsystem = climberSubsystem;

        addRequirements(climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climberSubsystem.setPosition(kClimber.ROBOT_UP);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
