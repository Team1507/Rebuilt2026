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
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;


public class AutoBlueDepotLeft {

    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)
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