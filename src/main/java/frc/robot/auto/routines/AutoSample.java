package frc.robot.auto.routines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.auto.AutoSequence;
import frc.robot.navigation.Nodes;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoSample {

    public static Command build(CommandSwerveDrivetrain drivetrain) {

        return new AutoSequence(drivetrain)
            .moveTo(Nodes.Start.CENTER)
            .waitSeconds(2.0)
            .build();
    }
}