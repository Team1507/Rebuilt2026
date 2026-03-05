//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionInputs;
import frc.lib.logging.Telemetry;
import frc.lib.math.geometry.VisionMeasurement;

public class PVManager extends SubsystemBase {

    private final PhotonVisionIO io;
    private final PhotonVisionInputs inputs = new PhotonVisionInputs();
    private final Telemetry telemetry;

    private final String[] cameraNames;
    private final PVPerCameraProcessor[] processors;

    private boolean seeded = false;

    // Fused PV output
    private Optional<Pose2d> fusedPose = Optional.empty();
    private double fusedTimestamp = 0.0;
    private double fusedXyStd = 0.0;
    private double fusedAngStd = 0.0;
    private double fusedAvgDistance = 999.0;   // NEW: distance gate for LocalizationManager

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.10;

    private PVPerCameraProcessor.TagMode tagMode = PVPerCameraProcessor.TagMode.HUB;

    // Debug info
    private final PVDebugInfo debugInfo = new PVDebugInfo();

    public PVManager(PhotonVisionIO io, Telemetry telemetry) {
        this.io = io;
        this.telemetry = telemetry;

        inputs.cameras = new PhotonVisionInputs.CameraInputs[io.getCameraCount()];
        for (int i = 0; i < inputs.cameras.length; i++) {
            inputs.cameras[i] = new PhotonVisionInputs.CameraInputs();
        }

        this.cameraNames = io.getCameraNames();

        processors = new PVPerCameraProcessor[io.getCameraCount()];
        debugInfo.cameras = new PVDebugInfo.CameraDebug[io.getCameraCount()];

        for (int i = 0; i < processors.length; i++) {
            processors[i] = new PVPerCameraProcessor(
                cameraNames[i],
                io.getEstimator(i)
            );
            processors[i].setTagMode(tagMode);

            debugInfo.cameras[i] = new PVDebugInfo.CameraDebug();
            debugInfo.cameras[i].name = cameraNames[i];
        }

        for (String name : cameraNames) {
            telemetry.registerVisionPoseSource(name);
        }
    }

    @Override
    public void periodic() {

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        io.updateInputs(inputs, seeded);

        // Reset fused output
        fusedPose = Optional.empty();
        fusedTimestamp = 0.0;
        fusedXyStd = 0.0;
        fusedAngStd = 0.0;
        fusedAvgDistance = 999.0;

        double bestScore = Double.NEGATIVE_INFINITY;
        PVPerCameraProcessor.VisionResult bestResult = null;

        boolean enabled = DriverStation.isEnabled();

        // Update debug tagMode
        debugInfo.tagMode = tagMode.toString();
        debugInfo.seeded = seeded;

        // Process each camera
        for (int i = 0; i < inputs.cameras.length; i++) {
            var cam = inputs.cameras[i];
            var dbg = debugInfo.cameras[i];

            dbg.connected = cam.connected;
            dbg.hasTarget = cam.hasTarget;
            dbg.timestamp = cam.timestamp;

            if (cam.rawResult != null && cam.rawResult.hasTargets()) {
                dbg.fiducialId = cam.rawResult.getBestTarget().fiducialId;
            } else {
                dbg.fiducialId = -1;
            }

            // 2. Field bounds (allow slight negative due to transforms)
            double x = pose3d.getX();
            double y = pose3d.getY();
            double fieldLen = kVision.APRILTAG_LAYOUT.getFieldLength();
            double fieldWid = kVision.APRILTAG_LAYOUT.getFieldWidth();

            if (x < -1.0 || x > fieldLen + 1.0 ||
                y < -1.0 || y > fieldWid + 1.0) {
                reject = true;
            }

            // 3. Distance sanity check (allow up to 6–7m)
            if (cam.avgDistance > 7.0) {  // was too strict before
                reject = true;
            }

            // 4. Ambiguity check (only reject VERY ambiguous single-tag solves)
            if (cam.tagCount == 1 && cam.ambiguity > 0.35) {  // relaxed from your version
                reject = true;
            }

            if (!dbg.accepted) continue;

            // Final measurement
            var resultOpt = processors[i].process(cam.rawResult, enabled);
            if (resultOpt.isEmpty()) continue;

            var res = resultOpt.get();
            VisionMeasurement m = res.measurement;

            // Score
            double score =
                res.tagCount * 200.0 -
                res.avgDistance * 20.0 -
                res.ambiguity * 80.0;

            if (score > bestScore) {
                bestScore = score;
                bestResult = res;
            }
        }

        // Save fused result
        if (bestResult != null) {
            VisionMeasurement m = bestResult.measurement;

            fusedPose = Optional.of(m.pose());
            fusedTimestamp = m.timestamp();
            fusedAvgDistance = bestResult.avgDistance;   // NEW: store avgDistance

            if (m.stdDevs() != null) {
                fusedXyStd = m.stdDevs().get(0, 0);
                fusedAngStd = m.stdDevs().get(2, 0);
            } else {
                fusedXyStd = 0.5;
                fusedAngStd = 0.5;
            }

            telemetry.logVisionPose("PhotonVision-Fused", m.pose());
        }

        // Update debug fused info
        debugInfo.fusedValid = fusedPose.isPresent();
        debugInfo.fusedPose = fusedPose.orElse(new Pose2d());
        debugInfo.fusedTimestamp = fusedTimestamp;
        debugInfo.fusedXyStd = fusedXyStd;
        debugInfo.fusedAngStd = fusedAngStd;
    }

    public void setTagMode(PVPerCameraProcessor.TagMode newMode) {
        this.tagMode = newMode;
        for (var p : processors) p.setTagMode(newMode);
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

    // NEW: used by LocalizationManager to gate PV usage
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
