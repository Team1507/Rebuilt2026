//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;
import frc.robot.Constants.kShooter;


public class AutoSubwayOutpost {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

            .startTimer()
            .resetPose(Nodes.Start.RIGHT)
            // .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)
            // .intakeHigh()
            // .parallel(
            //     seq -> seq.withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY, 0.1),
            //     seq -> seq.intakeDeploy())
            // .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY, 0.2)
            // .intakeHigh()
            // .parallel(
            //     seq -> seq.intakeRetract(),
            //     seq -> seq.moveThrough(Nodes.Midfield.RIGHT_BEFORE_BUMP, 0.5))
            //     .moveThrough(Nodes.Midfield.RIGHT_OVER_BUMP, 0.2)  
            // .driveTo(Nodes.Start.RIGHT)
            // .waitSeconds(0.5)
            // .pointToShoot()
            // .shootUntil(9)

             .driveTo(Nodes.Outpost.RIGHT_APPROACH_POINT_QUEST)
            .intakeDeploy()
            // .driveTo(Nodes.Outpost.RIGHT_APPROACH_POINT)
            // .waitSeconds(1.5)
            // .parallel(  
            //   seq -> seq.intakeRetract(),
            //   seq -> seq.driveTo(Nodes.Start.RIGHT))
            // .shootRPMUntil(19.5, kShooter.kRPM.BUMP_RAYMOND)
            
            //.moveThrough(Nodes.Outpost.APPROACH_RIGHT, 0.2)
            // .intakeDeploy()
            // .moveTo(Nodes.Outpost.RIGHT_APPROACH_POINT_QUEST)
            // .waitSeconds(1.5)
            // .moveTo(Nodes.Start.RIGHT)
            // .waitSeconds(0.5)
            // .pointToShoot()
            // .shootUntil(20)
            .build();
    }
}