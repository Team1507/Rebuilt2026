package frc.robot.commands.auto.routines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.auto.AutoSequence;
import frc.robot.framework.SubsystemsRecord;
import frc.robot.localization.nodes.Nodes;

public class AutoMoveLog {
    public static Command build(SubsystemsRecord record, double maxSpeed, double maxAngularSpeed) {

        return new AutoSequence(record, maxSpeed, maxAngularSpeed)

            .startTimer()

            // First Log
            .logMoveTo(Nodes.AllianceZoneBlue.BACK_RIGHT)

            .pointToTarget()
            .shootUntil(5.0)

            // Second Log
            .logMoveTo(Nodes.Tower.APPROACH_RIGHT)

            // Third Log
            .logMoveTo(Nodes.Tower.CLIMB)

            // Analyze All Logs
            .analyzeAllLogs()

            .build();
    }
    
}
