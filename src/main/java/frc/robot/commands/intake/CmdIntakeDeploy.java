//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;
import frc.robot.commands.intake.CmdIntakeDeploy;
import static frc.robot.Constants.kIntake.kArm.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeDeploy extends Command {
    private final IntakeArmSubsystem intakeArmSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;

    public boolean deploy = false;

    /** Creates a new CmdIntakeDeploy. */
    public CmdIntakeDeploy(IntakeArmSubsystem intakeArmSubsystem, IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeArmSubsystem = intakeArmSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;

        addRequirements(intakeArmSubsystem, intakeRollerSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeArmSubsystem.setPosition(DEPLOYED_ANGLE_DEGREES);
        intakeRollerSubsystem.run(.35);
        deploy = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeRollerSubsystem.stop();
        intakeArmSubsystem.setPosition(RETRACTED_ANGLE_DEGREES);
        deploy = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}