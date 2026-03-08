//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;


public class AutoSubwayRight {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, Consumer<Pose2d> resetQuestPose, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, resetQuestPose, MaxSpeed, MaxAngularRate)

            .resetPose(Nodes.Start.RIGHT)
            .startTimer()
            .driveTo(Nodes.Start.RIGHT)
            .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.2)
            .intakeDeploy()
            .driveTo(Nodes.Midfield.LEFT_RIGHT_SUBWAY)
           
            .driveTo(Nodes.Midfield.RIGHT_OVER_BUMP)
             .intakeRetract()
            .driveTo(Nodes.Start.RIGHT)
            .shoot()
            .build();
    }
}