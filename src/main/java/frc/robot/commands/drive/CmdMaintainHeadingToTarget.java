// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.function.Supplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdMaintainHeadingToTarget extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final Supplier<Pose2d> targetPoseSupplier;
  private final Supplier<Double> xSupplier;
  private final Supplier<Double> ySupplier;
  private final PIDController headingController = new PIDController(6.0, 0.0, 0.1); // Tune these values
  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
  /** Creates a new CmdMaintainHeadingToTarget. */
  public CmdMaintainHeadingToTarget(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> targetPoseSupplier, Supplier<Double> xSupplier, Supplier<Double> ySupplier) {
    this.drivetrain = drivetrain;
    this.targetPoseSupplier = targetPoseSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;
    Pose2d targetPose = targetPoseSupplier.get();
    Rotation2d headingToTarget = computeHeadingToTarget(currentPose, targetPose);
    double RotRate = headingController.calculate(drivetrain.getHeading().getRadians(), headingToTarget.getRadians());
    drivetrain.setControl(driveRequest.withVelocityX(xSupplier.get()).withVelocityY(ySupplier.get()).withRotationalRate(RotRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  private Rotation2d computeHeadingToTarget(Pose2d robot, Pose2d target) {
    double deltaX = target.getX() - robot.getX();
    double deltaY = target.getY() - robot.getY();
    return new Rotation2d(Math.atan2(deltaY, deltaX));
  }
}

