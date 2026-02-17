//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.logging;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

public class Telemetry {

    /* ---------------- Vision Pose Sources ---------------- */

    private final Map<String, Boolean> registeredVisionSources = new HashMap<>();

    public void registerVisionPoseSource(String sourceKey) {
        registeredVisionSources.put(sourceKey, true);
    }

    public void logVisionPose(String sourceKey, Pose2d pose) {
        if (registeredVisionSources.containsKey(sourceKey)) {
            Logger.recordOutput("Vision/" + sourceKey + "/Pose", pose);
        }
    }

    /* ---------------- Vision Cloud Logging ---------------- */

    public void logVisionAccepted(Pose3d[] poses) {
        Logger.recordOutput("Vision/AcceptedPoses", poses);
    }

    public void logVisionRejected(Pose3d[] poses) {
        Logger.recordOutput("Vision/RejectedPoses", poses);
    }

    public void logVisionTagPoses(Pose3d[] poses) {
        Logger.recordOutput("Vision/TagPoses", poses);
    }

    public void logVisionFusedCount(int count) {
        Logger.recordOutput("Vision/FusedObservationCount", count);
    }

    public void logVisionStartupSeed(Pose2d seed) {
        Logger.recordOutput("Vision/StartupPoseSeeded", seed);
    }

    /* ---------------- Photon Diagnostics ---------------- */

    private static class PhotonDiagKeys {
        final String hasTarget;
        final String tagCount;
        final String bestAmbiguity;
        final String bestTagId;
        final String poseAccepted;

        PhotonDiagKeys(String cam) {
            hasTarget = "Vision/" + cam + "/HasTarget";
            tagCount = "Vision/" + cam + "/TagCount";
            bestAmbiguity = "Vision/" + cam + "/BestAmbiguity";
            bestTagId = "Vision/" + cam + "/BestTagID";
            poseAccepted = "Vision/" + cam + "/PoseAccepted";
        }
    }

    private final Map<String, PhotonDiagKeys> photonDiagKeys = new HashMap<>();

    public void logPhotonCameraDiagnostics(
        String cameraName,
        boolean hasTarget,
        int tagCount,
        double bestAmbiguity,
        int bestTagId,
        boolean poseAccepted
    ) {
        PhotonDiagKeys keys = photonDiagKeys.computeIfAbsent(
            cameraName,
            PhotonDiagKeys::new
        );

        Logger.recordOutput(keys.hasTarget, hasTarget);
        Logger.recordOutput(keys.tagCount, tagCount);
        Logger.recordOutput(keys.bestAmbiguity, bestAmbiguity);
        Logger.recordOutput(keys.bestTagId, bestTagId);
        Logger.recordOutput(keys.poseAccepted, poseAccepted);
    }

    /* ---------------- PVManager Fused Output ---------------- */

    public void logPVFusedPose(Pose2d pose) {
        //Logger.recordOutput("Vision/PV/FusedPose", pose);
    }

    public void logPVFusedStdDevs(double xyStd, double angStd) {
        Logger.recordOutput("Vision/PV/FusedXYStd", xyStd);
        Logger.recordOutput("Vision/PV/FusedAngStd", angStd);
    }

    /* ---------------- LocalizationManager Logging ---------------- */

    public void logLocalizationFusedPose(Pose2d pose) {
        //Logger.recordOutput("Localization/FusedPose2d", pose);
    }

    public void logLocalizationTranslationSource(String source) {
        Logger.recordOutput("Localization/TranslationSource", source);
    }

    public void logLocalizationHeadingSource(String source) {
        Logger.recordOutput("Localization/HeadingSource", source);
    }

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


    /* ---------------- QuestNav Telemetry ---------------- */

    public void logQuestNavRawPose(Pose3d pose) {
        Logger.recordOutput("QuestNav/RawPose", pose);
    }

    public void logQuestNavCorrectedPose(Pose2d pose) {
        Logger.recordOutput("QuestNav/CorrectedPose", pose);
    }

    public void logQuestNavTracking(boolean tracking) {
        Logger.recordOutput("QuestNav/Tracking", tracking);
    }

    public void logQuestNavBattery(int percent) {
        Logger.recordOutput("QuestNav/BatteryPercent", percent);
    }

    public void logQuestNavAppTimestamp(double timestamp) {
        Logger.recordOutput("QuestNav/AppTimestamp", timestamp);
    }

    public void logQuestNavFrameTimestamp(double timestamp) {
        Logger.recordOutput("QuestNav/FrameTimestamp", timestamp);
    }

    public void logQuestNavLatency(double latencyMs) {
        Logger.recordOutput("QuestNav/LatencyMs", latencyMs);
    }

    public void logQuestNavConnected(boolean connected) {
        Logger.recordOutput("QuestNav/Connected", connected);
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
