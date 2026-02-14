// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdIntakeArmManual extends Command {
    public final IntakeArmSubsystem intakeArmSubsystem;
    public final Supplier<Double> angleSupplier;
    
    /** Creates a new CmdIntakeArmManual. */
    public CmdIntakeArmManual(IntakeArmSubsystem intakeArmSubsystem, Supplier<Double> targetAngle) {
        this.intakeArmSubsystem = intakeArmSubsystem;
        this.angleSupplier = targetAngle;

        addRequirements(intakeArmSubsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeArmSubsystem.setPosition(angleSupplier.get());
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
