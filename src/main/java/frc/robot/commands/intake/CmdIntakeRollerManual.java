// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeRollerManual extends Command {
    public final IntakeRollerSubsystem intakeRollerSubsystem;
    public final Supplier<Double> dcSupplier;
    /** Creates a new CmdIntakeRollerManual. */
    public CmdIntakeRollerManual(IntakeRollerSubsystem intakeRollerSubsystem, Supplier<Double> dutyCycle) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.dcSupplier = dutyCycle;

        addRequirements(intakeRollerSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeRollerSubsystem.run(dcSupplier.get());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intakeRollerSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
