//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// Robot Constants
import static frc.robot.Constants.kMoveThroughPose.*;

public class CmdMoveThroughPose extends Command {

    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d targetPose;
    private final double maxSpeed;
    private final double passRadius;

    private final PIDController xController = new PIDController(X_KP, X_KI, X_KD);
    private final PIDController yController = new PIDController(Y_KP, Y_KI, Y_KD);

    private double lastX, lastY, lastCheckTime;

    public CmdMoveThroughPose(
        CommandSwerveDrivetrain drivetrain,
        Pose2d targetPose,
        double maxSpeed,
        double passRadius
    ) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.passRadius = passRadius;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        lastX = drivetrain.getState().Pose.getX();
        lastY = drivetrain.getState().Pose.getY();
        lastCheckTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        xController.reset();
        yController.reset();
    }

    @Override
    public void execute() {
        Pose2d current = drivetrain.getState().Pose;

        double xSpeed = xController.calculate(current.getX(), targetPose.getX());
        double ySpeed = yController.calculate(current.getY(), targetPose.getY());

        // Cap speeds but DO NOT slow down near target
        xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), maxSpeed), xSpeed);
        ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), maxSpeed), ySpeed);

        // No deadband. No angle control. No slowdown.

        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, 0.0, current.getRotation()
        );

        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotRelative)
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(
            new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds())
        );
    }

    @Override
    public boolean isFinished() {
        Pose2d current = drivetrain.getState().Pose;

        // Finish when within radius
        if (current.getTranslation().getDistance(targetPose.getTranslation()) < passRadius) {
            return true;
        }

        // Optional: stall detection (copy from moveTo)
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double dx = Math.abs(current.getX() - lastX);
        double dy = Math.abs(current.getY() - lastY);

        if (dx < 0.01 && dy < 0.01 && (now - lastCheckTime) > 0.5) {
            return true;
        }

        if (dx > 0.01 || dy > 0.01) {
            lastX = current.getX();
            lastY = current.getY();
            lastCheckTime = now;
        }

        return false;
    }
}
