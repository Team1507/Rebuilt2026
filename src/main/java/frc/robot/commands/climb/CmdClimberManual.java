// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climb;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdClimberManual extends Command {
    public final ClimberSubsystem climberSubsystem;
    public final Supplier<Double> positionSupplier;

    /** Creates a new CmdClimberManual. */
    public CmdClimberManual(ClimberSubsystem climberSubsystem, Supplier<Double> targetPosition) {
        this.climberSubsystem = climberSubsystem;
        this.positionSupplier = targetPosition;

        addRequirements(climberSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climberSubsystem.setPosition(positionSupplier.get());
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
