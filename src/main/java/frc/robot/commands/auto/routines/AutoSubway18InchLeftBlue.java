//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS
//                         Programmed by Andrew :)
package frc.robot.commands.auto.routines;




import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.kShooter;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.CoordinatorRecord;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;
import frc.robot.localization.nodes.Nodes.Start.Red;


public class AutoSubway18InchLeftBlue {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, double MaxSpeed, double MaxAngularRate) {


        return new AutoSequence(record, coordinator, MaxSpeed, MaxAngularRate)
            .startTimer()
            .resetPose(Nodes.Start.Blue.LEFT)
            .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY, 0.1),
                seq -> seq.intakeDeploy())
            .withSpeed(MaxSpeed * 0.5).moveThrough(Nodes.Midfield.RIGHT_LEFT_SUBWAY, 0.5)
            .intakeHigh()
            .parallel(
                seq -> seq.intakeRetract(),
                seq -> seq.moveThrough(Nodes.Midfield.LEFT_BEFORE_BUMP, 0.5))  
            .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
            .moveThrough(Nodes.Start.Blue.LEFT, 0.2)
            .withSpeed( MaxSpeed * 0.5).driveTo(Nodes.Start.Blue.SHOOTING_SPOT_LEFT)
            .headingToTarget(Nodes.Hub.CENTER)
            .shootUntil(11.0)
             .headingToTarget(Nodes.Midfield.LEFT_TURN)
             .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP2, 0.2)
            .moveThrough(Nodes.Midfield.LEFT_LEFT_SUBWAY,0.2)
            .intakeHigh()
            .parallel(
                seq -> seq.withSpeed(MaxSpeed *0.5).moveThrough(Nodes.Midfield.MIDDLE_18INCH_SUBWAY_LEFT, 0.1),
                seq -> seq.intakeDeploy())
                .moveThrough(Nodes.Midfield.LEFT_BEFORE_BUMP, 0.2)
                .moveThrough(Nodes.Midfield.LEFT_OVER_BUMP, 0.2)
                .driveTo(Nodes.Start.Red.LEFT)
            .waitSeconds(0.5)
            .pointToShoot()
            .shootUntil(19.99)
            .build();
    }
}
