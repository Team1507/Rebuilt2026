//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.logging.Telemetry;
import frc.lib.core.math.geometry.VisionMeasurement;
import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionInputs;

/**
 * Single-camera PhotonVision manager for Bluecam only.
 *
 * Handles:
 *  - Raw PV input
 *  - Per-frame processing
 *  - Strategy selection
 *  - Fused pose output
 *  - Debug info
 */
public class PVManager extends SubsystemBase {

    private final PhotonVisionIO io;
    private final PhotonVisionInputs inputs = new PhotonVisionInputs();
    private final Telemetry telemetry;

    private final PVPerCameraProcessor processor;

    private boolean seeded = false;

    // Fused PV output
    private Optional<Pose2d> fusedPose = Optional.empty();
    private double fusedTimestamp = 0.0;
    private double fusedXyStd = 0.0;
    private double fusedAngStd = 0.0;
    private double fusedAvgDistance = 999.0;

    // 10 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.10;

    private PVPerCameraProcessor.TagMode tagMode = PVPerCameraProcessor.TagMode.HUB;

    // Debug info (single camera)
    private final PVDebugInfo debugInfo = new PVDebugInfo();

    public PVManager(PhotonVisionIO io, Telemetry telemetry) {
        this.io = io;
        this.telemetry = telemetry;

        // Single processor for Bluecam
        this.processor = new PVPerCameraProcessor(
            "Bluecam",
            io.getEstimator(0)
        );
        this.processor.setTagMode(tagMode);

        telemetry.registerVisionPoseSource("Bluecam");
        telemetry.registerVisionPoseSource("PhotonVision-Fused");
    }

    @Override
    public void periodic() {

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        io.updateInputs(inputs, seeded);

        // Reset fused output
        fusedPose = Optional.empty();
        fusedTimestamp = 0.0;
        fusedXyStd = 0.0;
        fusedAngStd = 0.0;
        fusedAvgDistance = 999.0;

        boolean enabled = DriverStation.isEnabled();

        // Debug state
        debugInfo.tagMode = tagMode.toString();
        debugInfo.seeded = seeded;

        // Pull camera input
        var cam = inputs.camera;
        var dbg = debugInfo.camera;

        dbg.name = cam.name;
        dbg.connected = cam.connected;
        dbg.hasTarget = cam.hasTarget;
        dbg.timestamp = cam.timestamp;

        if (cam.rawResult != null && cam.rawResult.hasTargets()) {
            dbg.fiducialId = cam.rawResult.getBestTarget().fiducialId;
        } else {
            dbg.fiducialId = -1;
        }

        // Debug path
        var debugOpt = processor.processDebug(cam.rawResult, enabled);
        if (debugOpt.isPresent()) {
            var d = debugOpt.get();
            dbg.strategyUsed = d.strategyUsed;
            dbg.accepted = d.accepted;
            dbg.rejectionReason = d.rejectionReason;
            dbg.tagCount = d.tagCount;
            dbg.avgDistance = d.avgDistance;
            dbg.ambiguity = d.ambiguity;
            dbg.xyStd = d.xyStd;
            dbg.angStd = d.angStd;
        } else {
            dbg.strategyUsed = "NONE";
            dbg.accepted = false;
            dbg.rejectionReason = "NO_RESULT";
            dbg.tagCount = 0;
            dbg.avgDistance = 0.0;
            dbg.ambiguity = 0.0;
            dbg.xyStd = 0.0;
            dbg.angStd = 0.0;
        }

        if (!dbg.accepted) {
            debugInfo.fusedValid = false;
            return;
        }

        // Final measurement
        var resultOpt = processor.process(cam.rawResult, enabled);
        if (resultOpt.isEmpty()) {
            debugInfo.fusedValid = false;
            return;
        }

        var res = resultOpt.get();
        VisionMeasurement m = res.measurement;

        fusedPose = Optional.of(m.pose());
        fusedTimestamp = m.timestamp();
        fusedAvgDistance = res.avgDistance;

        if (m.stdDevs() != null) {
            fusedXyStd = m.stdDevs().get(0, 0);
            fusedAngStd = m.stdDevs().get(2, 0);
        } else {
            fusedXyStd = 0.5;
            fusedAngStd = 0.5;
        }

        debugInfo.fusedValid = true;
        debugInfo.fusedPose = m.pose();
        debugInfo.fusedTimestamp = fusedTimestamp;
        debugInfo.fusedXyStd = fusedXyStd;
        debugInfo.fusedAngStd = fusedAngStd;

        telemetry.logVisionPose("PhotonVision-Fused", m.pose());
    }

    // ============================
    // Public API
    // ============================

    public void setTagMode(PVPerCameraProcessor.TagMode newMode) {
        this.tagMode = newMode;
        processor.setTagMode(newMode);
    }

    public void setSeeded(boolean seeded) {
        this.seeded = seeded;
    }

    public Optional<Pose2d> getFusedPose() {
        return fusedPose;
    }

    public double getFusedTimestamp() {
        return fusedTimestamp;
    }

    public double getFusedXyStd() {
        return fusedXyStd;
    }

    public double getFusedAngStd() {
        return fusedAngStd;
    }

    public double getFusedAvgDistance() {
        return fusedAvgDistance;
    }

    public boolean hasGoodVision() {
        return fusedPose.isPresent();
    }

    public PVDebugInfo getDebugInfo() {
        return debugInfo;
    }
}