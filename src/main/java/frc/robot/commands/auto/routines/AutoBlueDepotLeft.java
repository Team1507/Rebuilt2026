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


public class AutoBlueDepotLeft {
    public static Command build(SubsystemsRecord record, CoordinatorRecord coordinator, Consumer<Pose2d> resetQuestPose, double MaxSpeed, double MaxAngularRate) {

        return new AutoSequence(record, coordinator, resetQuestPose, MaxSpeed, MaxAngularRate)
            .moveTo(Nodes.Start.START_DEPOT_LEFT)
            .intakeDeploy()
            .moveTo(Nodes.Depot.INTAKE_DEPOT)
            .moveTo(Nodes.AllianceZoneBlue.BACK_LEFT)
            .intakeRetract()
            .shoot()
            .moveTo(Nodes.Tower.APPROACH_LEFT)
            .build();
    }
}