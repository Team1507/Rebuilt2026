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
import edu.wpi.first.math.geometry.Pose3d;

import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionInputs;

// Logging
import frc.lib.logging.Telemetry;

// Constants
import frc.robot.Constants.kVision;

public class PVManager extends SubsystemBase {

    private final PhotonVisionIO io;
    private final PhotonVisionInputs inputs = new PhotonVisionInputs();
    private final Telemetry telemetry;

    private final String[] cameraNames;

    private boolean seeded = false;

    // Fused PV output
    private Optional<Pose2d> fusedPose = Optional.empty();
    private double fusedTimestamp = 0.0;
    private double fusedXyStd = 0.0;
    private double fusedAngStd = 0.0;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.10;

    public PVManager(PhotonVisionIO io, Telemetry telemetry) {
        this.io = io;
        this.telemetry = telemetry;

        // Allocate camera input slots
        inputs.cameras = new PhotonVisionInputs.CameraInputs[io.getCameraCount()];
        for (int i = 0; i < inputs.cameras.length; i++) {
            inputs.cameras[i] = new PhotonVisionInputs.CameraInputs();
        }

        this.cameraNames = io.getCameraNames();

        // Register each camera as a vision pose source
        for (String name : cameraNames) {
            telemetry.registerVisionPoseSource(name);
        }
    }

    @Override
    public void periodic() {

        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        // Update raw inputs (fast)
        io.updateInputs(inputs, seeded);

        // Reset fused output
        fusedPose = Optional.empty();
        fusedTimestamp = 0.0;
        fusedXyStd = 0.0;
        fusedAngStd = 0.0;

        double bestScore = Double.NEGATIVE_INFINITY;
        Pose2d bestPose = null;
        double bestTimestamp = 0.0;
        double bestXyStd = 0.0;
        double bestAngStd = 0.0;

        // Process each camera
        for (int i = 0; i < inputs.cameras.length; i++) {
            var cam = inputs.cameras[i];

            if (!cam.pose3d.isPresent()) continue;

            Pose3d pose3d = cam.pose3d.get();
            Pose2d pose2d = cam.pose2d.get();

            // Reject bad observations (robust version)
            boolean reject = false;

            // 1. Z-height sanity check (camera height + noise)
            if (Math.abs(pose3d.getZ()) > 1.0) {  // was too strict before
                reject = true;
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

            // 5. Require at least 1 tag (should never happen, but safe)
            if (cam.tagCount < 1) {
                reject = true;
            }

            if (reject) continue;

            // Compute score for fusion
            double score =
                cam.tagCount * 200.0 -
                cam.avgDistance * 20.0 -
                cam.ambiguity * 80.0;

            if (score > bestScore) {
                bestScore = score;
                bestPose = pose2d;
                bestTimestamp = cam.timestamp;

                if (cam.stdDevs != null) {
                    bestXyStd = cam.stdDevs.get(0, 0);
                    bestAngStd = cam.stdDevs.get(2, 0);
                } else {
                    bestXyStd = 0.5;   // reasonable default
                    bestAngStd = 0.5;
                }

            }
        }

        // Save fused result
        if (bestPose != null) {
            fusedPose = Optional.of(bestPose);
            fusedTimestamp = bestTimestamp;
            fusedXyStd = bestXyStd;
            fusedAngStd = bestAngStd;

            // Log only the fused pose (fast)
            telemetry.logVisionPose("PhotonVision-Fused", bestPose);
        }
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

    public boolean hasGoodVision() {
        return fusedPose.isPresent();
    }
}
