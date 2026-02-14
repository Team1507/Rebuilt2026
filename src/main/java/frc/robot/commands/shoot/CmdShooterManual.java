// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shoot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdShooterManual extends Command {
    public final ShooterSubsystem shooterSubsystem;
    public final Supplier<Double> rpmSupplier;

    /** Creates a new CmdShooterManual. */
    public CmdShooterManual(ShooterSubsystem shooterSubsystem, Supplier<Double> rpmSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.rpmSupplier = rpmSupplier;

        addRequirements(shooterSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSubsystem.setTargetRPM(rpmSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
