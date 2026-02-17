//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import frc.lib.hardware.PhotonVisionHardware;
import frc.robot.Constants.kVision;

/**
 * Real PhotonVision implementation with explicit camera names.
 *
 * This class wraps PhotonCamera + PhotonPoseEstimator and produces
 * raw pose estimates, timestamps, and stddevs for PVManager.
 */
public class PhotonVisionIOReal implements PhotonVisionIO {

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;
    private final String[] cameraNames;

    /**
     * Creates a PhotonVision IO layer
     */
    public PhotonVisionIOReal() {
        int count = PhotonVisionHardware.CAMERAS.length;

        cameras = new PhotonCamera[count];
        estimators = new PhotonPoseEstimator[count];
        cameraNames = new String[count];

        for (int i = 0; i < count; i++) {
            var cfg = PhotonVisionHardware.CAMERAS[i];

            // Explicit camera name (BLU/YEL)
            cameraNames[i] = cfg.name;

            cameras[i] = new PhotonCamera(cfg.name);
            estimators[i] = new PhotonPoseEstimator(
                kVision.APRILTAG_LAYOUT,
                cfg.robotToCamera
            );
        }
    }

    @Override
    public int getCameraCount() {
        return cameras.length;
    }

    /** Allows PVManager to use human-readable names like "Photon-BLU" */
    public String[] getCameraNames() {
        return cameraNames;
    }

    @Override
    public void updateInputs(PhotonVisionInputs inputs) {

        // Ensure array is sized correctly
        if (inputs.cameras.length != cameras.length) {
            inputs.cameras = new PhotonVisionInputs.CameraInputs[cameras.length];
            for (int i = 0; i < cameras.length; i++) {
                inputs.cameras[i] = new PhotonVisionInputs.CameraInputs();
                inputs.cameras[i].name = cameraNames[i];
            }
        }

        inputs.anyCameraConnected = false;
        inputs.totalTags = 0;

        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera cam = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];
            PhotonVisionInputs.CameraInputs out = inputs.cameras[i];

            out.connected = cam.isConnected();
            if (out.connected) inputs.anyCameraConnected = true;

            var results = cam.getAllUnreadResults();
            if (results.isEmpty()) {
                out.hasTarget = false;
                out.pose2d = Optional.empty();
                out.pose3d = Optional.empty();
                continue;
            }

            PhotonPipelineResult result = results.get(results.size() - 1);
            out.hasTarget = result.hasTargets();

            if (out.hasTarget) {
                out.yaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                out.pitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
            }

            Optional<EstimatedRobotPose> estimate = estimator.estimatePnpDistanceTrigSolvePose(result);

            if (estimate.isEmpty()) {
                out.pose2d = Optional.empty();
                out.pose3d = Optional.empty();
                continue;
            }

            EstimatedRobotPose est = estimate.get();
            Pose3d pose3d = est.estimatedPose;

            out.pose3d = Optional.of(pose3d);
            out.pose2d = Optional.of(pose3d.toPose2d());
            out.timestamp = est.timestampSeconds;

            out.tagCount = est.targetsUsed.size();
            inputs.totalTags += out.tagCount;

            out.avgDistance = est.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(0.0);

            out.ambiguity = est.targetsUsed.stream()
                .mapToDouble(t -> t.poseAmbiguity)
                .average().orElse(0.0);

            // Compute stddevs
            double xyStd = kVision.constrainedPnpXyStdBase * out.avgDistance;
            double angStd = kVision.constrainedPnpAngStdBase * out.avgDistance;

            if (out.tagCount > 1) {
                double discount = Math.sqrt(out.tagCount);
                xyStd /= discount;
                angStd /= discount;
            }

            out.stdDevs = VecBuilder.fill(xyStd, xyStd, angStd);
        }
    }
}
