//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.core.logging;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Telemetry {

    /* ---------------- Shooter Telemetry ---------------- */

    private final Map<String, Boolean> registeredShooterSources = new HashMap<>();

    public void registerShooterSource(String name) {
        registeredShooterSources.put(name, true);
    }

    public void logShooter(
        String name,
        double rpm,
        double targetRpm,
        double voltage,
        double statorCurrent,
        double supplyCurrent,
        double closedLoopError,
        Pose2d shooterPose,
        double distanceToTarget
    ) {
        if (!registeredShooterSources.containsKey(name)) return;

        // --- SANITIZE VALUES ---
        if (Double.isNaN(rpm)) rpm = 0.0;
        if (Double.isNaN(targetRpm)) targetRpm = 0.0;
        if (Double.isNaN(voltage)) voltage = 0.0;
        if (Double.isNaN(statorCurrent)) statorCurrent = 0.0;
        if (Double.isNaN(supplyCurrent)) supplyCurrent = 0.0;
        if (Double.isNaN(closedLoopError)) closedLoopError = 0.0;
        if (Double.isNaN(distanceToTarget)) distanceToTarget = 0.0;

        if (shooterPose == null) shooterPose = new Pose2d();

        String base = "Shooter/" + name;

        Logger.recordOutput(base + "/RPM", rpm);
        Logger.recordOutput(base + "/TargetRPM", targetRpm);
        Logger.recordOutput(base + "/Voltage", voltage);
        Logger.recordOutput(base + "/StatorCurrent", statorCurrent);
        Logger.recordOutput(base + "/SupplyCurrent", supplyCurrent);
        Logger.recordOutput(base + "/ClosedLoopError", closedLoopError);
        //Logger.recordOutput(base + "/Pose", shooterPose);
        Logger.recordOutput(base + "/DistanceToTarget", distanceToTarget);
    }


    /* ---------------- Drivetrain Telemetry ---------------- */

    public void logDriveState(SwerveDriveState state) {
        Logger.recordOutput("Drive/Pose", state.Pose);
        Logger.recordOutput("Drive/Speeds", state.Speeds);
        Logger.recordOutput("Drive/ModuleStates", state.ModuleStates);
        Logger.recordOutput("Drive/ModuleTargets", state.ModuleTargets);
        Logger.recordOutput("Drive/ModulePositions", state.ModulePositions);
        Logger.recordOutput("Drive/Timestamp", state.Timestamp);
        Logger.recordOutput("Drive/OdometryPeriod", state.OdometryPeriod);
    }

    /* ---------------- Optional: Field Overlay Logging ---------------- */

    public void logPolygon(String name, java.util.List<Translation2d> polygon) {
        Pose2d[] pts = new Pose2d[polygon.size()];
        for (int i = 0; i < polygon.size(); i++) {
            pts[i] = new Pose2d(polygon.get(i), new edu.wpi.first.math.geometry.Rotation2d());
        }
        Logger.recordOutput("Field/" + name, pts);
    }
}
