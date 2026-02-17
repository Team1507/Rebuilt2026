//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import frc.lib.io.photonvision.PhotonVisionIO;
import frc.lib.io.photonvision.PhotonVisionInputs;
import frc.robot.Constants.kVision;
import frc.lib.logging.Telemetry;

import java.util.Optional;

public class PVManager extends SubsystemBase {

    private final PhotonVisionIO io;
    private final PhotonVisionInputs inputs = new PhotonVisionInputs();
    private final Telemetry telemetry;

    private final String[] cameraNames;

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
        io.updateInputs(inputs);

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

            // Reject bad observations
            boolean reject =
                Math.abs(pose3d.getZ()) > kVision.maxZError ||
                pose3d.getX() < 0.0 ||
                pose3d.getX() > kVision.APRILTAG_LAYOUT.getFieldLength() ||
                pose3d.getY() < 0.0 ||
                pose3d.getY() > kVision.APRILTAG_LAYOUT.getFieldWidth() ||
                cam.avgDistance > kVision.maxTagDistance ||
                (cam.tagCount == 1 && cam.ambiguity > kVision.maxAmbiguity);

            if (reject) continue;

            // Compute score for fusion
            double score =
                cam.tagCount * 100.0 -
                cam.avgDistance * 10.0 -
                cam.ambiguity * 50.0;

            if (score > bestScore) {
                bestScore = score;
                bestPose = pose2d;
                bestTimestamp = cam.timestamp;

                if (cam.stdDevs != null) {
                    bestXyStd = cam.stdDevs.get(0, 0);
                    bestAngStd = cam.stdDevs.get(2, 0);
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
