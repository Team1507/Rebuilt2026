package frc.robot.localization.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.lib.core.math.FieldInfo;
import frc.lib.core.math.geometry.VisionMeasurement;
import frc.lib.core.util.Alliance;

public final class Vision {

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public enum TagMode { ALL_TAGS, HUB, TOWER }

    private static final double Z_TOLERANCE = 0.5;

    // Std-dev weights for MultiTag, multiplied by avgDistance^2
    private static final double MULTITAG_XY  = 0.1;
    private static final double MULTITAG_ANG = 0.25;

    private final Camera[] cameras;
    private TagMode tagMode = TagMode.HUB;

    public Vision(CameraConfig[] configs) {
        cameras = new Camera[configs.length];
        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new Camera(configs[i]);
        }
    }

    public void addHeadingData(double timestampSeconds, Rotation2d heading) {
        for (var cam : cameras) cam.estimator.addHeadingData(timestampSeconds, heading);
    }

    public void resetHeadingData(double timestampSeconds, Rotation2d heading) {
        for (var cam : cameras) cam.estimator.resetHeadingData(timestampSeconds, heading);
    }

    public void setTagMode(TagMode mode) {
        this.tagMode = mode;
    }

    /**
     * Returns all accepted measurements from all cameras since the last call.
     * Feed every result into the drivetrain's Kalman filter.
     */
    public VisionMeasurement[] getUnreadResults() {
        List<VisionMeasurement> out = new ArrayList<>();
        for (var cam : cameras) cam.processResults(out);
        return out.toArray(VisionMeasurement[]::new);
    }

    private boolean useTag(int id) {
        boolean noAlliance = DriverStation.getAlliance().isEmpty();
        switch (tagMode) {
            case ALL_TAGS:
                return id >= 1 && id <= 32;
            case HUB:
                boolean blueHub = (id >= 18 && id <= 21) || (id >= 24 && id <= 27);
                boolean redHub  = (id >= 2  && id <= 5)  || (id >= 8  && id <= 11);
                return noAlliance ? (blueHub || redHub) : Alliance.isBlue() ? blueHub : redHub;
            case TOWER:
                boolean blueTower = id == 31 || id == 32;
                boolean redTower  = id == 15 || id == 16;
                return noAlliance ? (blueTower || redTower) : Alliance.isBlue() ? blueTower : redTower;
        }
        return false;
    }

    private boolean allMultiTagIdsValid(List<Short> ids) {
        for (short id : ids) {
            if (!useTag(id)) return false;
        }
        return true;
    }

    private class Camera {
        final PhotonCamera camera;
        final PhotonPoseEstimator estimator;

        Camera(CameraConfig cfg) {
            camera = new PhotonCamera(cfg.name());
            estimator = new PhotonPoseEstimator(FieldInfo.aprilTags(), cfg.robotToCamera());
        }

        void processResults(List<VisionMeasurement> out) {
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                if (!result.hasTargets()) continue;

                // Only accept coprocessor MultiTag solve — requires ≥2 valid tags
                var multi = result.getMultiTagResult();
                if (multi.isEmpty()
                    || multi.get().fiducialIDsUsed.isEmpty()
                    || !allMultiTagIdsValid(multi.get().fiducialIDsUsed)) continue;

                Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                Pose3d pose3d = estimate.get().estimatedPose;
                if (Math.abs(pose3d.getZ()) > Z_TOLERANCE) continue;

                double avgDist = 0.0;
                for (var t : estimate.get().targetsUsed) {
                    avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
                }
                avgDist /= estimate.get().targetsUsed.size();

                double xyStd  = MULTITAG_XY  * avgDist * avgDist;
                double angStd = MULTITAG_ANG * avgDist * avgDist;

                out.add(new VisionMeasurement(
                    pose3d.toPose2d(),
                    estimate.get().timestampSeconds,
                    VecBuilder.fill(xyStd, xyStd, angStd)
                ));
            }
        }
    }
}
