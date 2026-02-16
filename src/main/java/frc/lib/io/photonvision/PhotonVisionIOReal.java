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
 * Real PhotonVision implementation.
 *
 * This class wraps PhotonCamera + PhotonPoseEstimator and produces
 * raw pose estimates, timestamps, and stddevs for PVManager.
 */
public class PhotonVisionIOReal implements PhotonVisionIO {

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;

    public PhotonVisionIOReal() {
        int count = PhotonVisionHardware.CAMERAS.length;

        cameras = new PhotonCamera[count];
        estimators = new PhotonPoseEstimator[count];

        for (int i = 0; i < count; i++) {
            var cfg = PhotonVisionHardware.CAMERAS[i];

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

    @Override
    public void updateInputs(CameraInputs[] inputs) {
        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera cam = cameras[i];
            PhotonPoseEstimator estimator = estimators[i];
            CameraInputs out = inputs[i];

            var results = cam.getAllUnreadResults();
            if (results.isEmpty()) {
                out.hasTarget = false;
                continue;
            }

            PhotonPipelineResult result = results.get(results.size() - 1);
            out.hasTarget = result.hasTargets();

            if (out.hasTarget) {
                out.latestYaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                out.latestPitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
            }

            Optional<EstimatedRobotPose> estimate = estimator.update(result);
            if (estimate.isEmpty()) {
                out.latestPose3d = Optional.empty();
                out.latestPose2d = Optional.empty();
                continue;
            }

            EstimatedRobotPose est = estimate.get();
            Pose3d pose3d = est.estimatedPose;

            out.latestPose3d = Optional.of(pose3d);
            out.latestPose2d = Optional.of(pose3d.toPose2d());
            out.timestampSeconds = est.timestampSeconds;

            out.tagCount = est.targetsUsed.size();
            out.avgDistance = est.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(0.0);

            // Compute stddevs based on your constants
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
