//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoSubway18InchRed {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)

            .startTimer()
            .resetPose(Nodes.Start.Red.RIGHT)
            .moveThrough(Nodes.Midfield.Red.RIGHT_OVER_BUMP, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY_RED, 0.1),
                seq -> seq.intakeDeploy())
            .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY_RED, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.Red.RIGHT_BEFORE_BUMP, 0.5))
                .moveThrough(Nodes.Midfield.Red.RIGHT_OVER_BUMP, 0.2)  
            .driveTo(Nodes.Start.Red.RIGHT)
            .waitSeconds(0.5)
            .headingToTarget(Nodes.Hub.RED_CENTER)
            .shootUntil(10)
            .changeHeading(Nodes.Midfield.Red.RIGHT_TURN)
            .moveThrough(Nodes.Midfield.Red.RIGHT_OVER_BUMP2, 0.2)
            .moveThrough(Nodes.Midfield.RIGHT_RIGHT_SUBWAY_RED,0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed *0.5).moveThrough(Nodes.Midfield.MIDDLE_18INCH_SUBWAY_RED, 0.1),
                seq -> seq.intakeDeploy())            .moveThrough(Nodes.Midfield.LEFT_RIGHT_SUBWAY_RED, 0.2)
                .moveThrough(Nodes.Midfield.Red.RIGHT_BEFORE_BUMP, 0.2)
                .moveThrough(Nodes.Midfield.Red.RIGHT_OVER_BUMP, 0.2)
                .driveTo(Nodes.Start.Red.RIGHT)
            .waitSeconds(0.5)
            .headingToTarget(Nodes.Hub.RED_CENTER)
             .waitSeconds(0.5)
            .shootUntil(19.99)
            .build();
    }
}
