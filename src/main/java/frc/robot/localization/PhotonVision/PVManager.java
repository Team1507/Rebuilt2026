//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization.PhotonVision;

import static frc.robot.Constants.kVision.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import frc.robot.Constants.kQuest;
import frc.robot.utilities.Telemetry;

public class PVManager extends SubsystemBase {

    public static record CameraConfig(String name, Transform3d robotToCamera) {}

    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private final Telemetry telemetry;
    private final VisionConsumer consumer;
    private final Supplier<Rotation2d> headingSupplier;
    private final Consumer<Pose2d> poseSeeder;
    private final Consumer<Pose3d> questNavPoseSeeder;
    private final Camera[] cameras;

    private boolean startupPoseSeeded = false;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05;

    public PVManager(
        Telemetry telemetry,
        VisionConsumer consumer,
        Supplier<Rotation2d> headingSupplier,
        Consumer<Pose2d> poseSeeder,
        Consumer<Pose3d> questNavPoseSeeder,
        CameraConfig... configs
    ) {
        this.telemetry = telemetry;
        this.consumer = consumer;
        this.headingSupplier = headingSupplier;
        this.poseSeeder = poseSeeder;
        this.questNavPoseSeeder = questNavPoseSeeder;

        this.cameras = new Camera[configs.length];

        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new Camera(configs[i]);
                telemetry.registerVisionPoseSource(configs[i].name());
        }
    }

    public void resetHeadingData() {
        double now = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();
        for (var cam : cameras) {
            cam.estimator.resetHeadingData(now, heading);
        }
    }

    public Rotation2d getTargetX(int cameraIndex) {
        return cameras[cameraIndex].latestTargetYaw;
    }

    public Rotation2d getTargetY(int cameraIndex) {
        return cameras[cameraIndex].latestTargetPitch;
    }

    public boolean hasTarget(int cameraIndex) {
        return cameras[cameraIndex].hasTarget;
    }

    @Override
    public void periodic() {

        // 20 Hz throttle
        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PROCESS_PERIOD) return;
        lastProcessTime = now;

        Rotation2d heading = headingSupplier.get();
        for (var cam : cameras) cam.estimator.addHeadingData(now, heading);

        List<Pose3d> accepted = new LinkedList<>();
        List<Pose3d> rejected = new LinkedList<>();
        List<Pose3d> tagPoses = new LinkedList<>();

        Pose2d latestVisionPose = null;
        Pose2d seedPose = null;
        double seedScore = Double.NEGATIVE_INFINITY;

        int fusedCount = 0;

        for (int i = 0; i < cameras.length; i++) {
            Camera cam = cameras[i];

            boolean usingTrig = startupPoseSeeded
                ? cam.disabledDebounce.calculate(DriverStation.isEnabled())
                : false;

            var results = cam.photonCamera.getAllUnreadResults();
            if (results.isEmpty()) continue;

            PhotonPipelineResult result = results.get(results.size() - 1);

            cam.hasTarget = result.hasTargets();
            if (cam.hasTarget) {
                cam.latestTargetYaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                cam.latestTargetPitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
            }

            Optional<EstimatedRobotPose> estimate =
                usingTrig
                    ? cam.estimator.estimatePnpDistanceTrigSolvePose(result)
                    : cam.estimator.estimateCoprocMultiTagPose(result);

            if (estimate.isEmpty()) continue;
            var est = estimate.get();
            if (est.targetsUsed.isEmpty()) continue;

            Pose3d robotPose3d = est.estimatedPose;
            int tagCount = est.targetsUsed.size();

            double avgDistance = est.targetsUsed.stream()
                .mapToDouble(t -> t.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(0.0);

            boolean reject =
                Math.abs(robotPose3d.getZ()) > maxZError ||
                robotPose3d.getX() < 0.0 ||
                robotPose3d.getX() > APRILTAG_LAYOUT.getFieldLength() ||
                robotPose3d.getY() < 0.0 ||
                robotPose3d.getY() > APRILTAG_LAYOUT.getFieldWidth() ||
                avgDistance > maxTagDistance ||
                (tagCount == 1 && est.targetsUsed.get(0).getPoseAmbiguity() > maxAmbiguity);

            if (reject) {
                rejected.add(robotPose3d);
                continue;
            }

            accepted.add(robotPose3d);
            latestVisionPose = robotPose3d.toPose2d();

            for (var target : est.targetsUsed) {
                APRILTAG_LAYOUT.getTagPose(target.getFiducialId())
                    .ifPresent(tagPoses::add);
            }

            if (!startupPoseSeeded) {
                double score = tagCount * 100.0 - avgDistance * 10.0;
                if (score > seedScore) {
                    seedScore = score;
                    seedPose = robotPose3d.toPose2d();
                }
            } else {
                double xyStd =
                    (usingTrig ? trigXyStdBase : constrainedPnpXyStdBase) * avgDistance;
                double angStd =
                    (usingTrig ? 1e5 : constrainedPnpAngStdBase) * avgDistance;

                if (tagCount > 1) {
                    double discount = Math.sqrt(tagCount);
                    xyStd /= discount;
                    angStd /= discount;
                }

                Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, angStd);
                consumer.accept(robotPose3d.toPose2d(), est.timestampSeconds, stdDevs);
                fusedCount++;
            }

            telemetry.logVisionPose(cam.name, robotPose3d.toPose2d());
        }

        if (!startupPoseSeeded && seedPose != null) {
            poseSeeder.accept(seedPose);
            questNavPoseSeeder.accept(new Pose3d(seedPose).transformBy(kQuest.ROBOT_TO_QUEST));
            resetHeadingData();
            startupPoseSeeded = true;
            telemetry.logVisionStartupSeed(seedPose);
        }

        telemetry.logVisionAccepted(accepted.toArray(new Pose3d[0]));
        telemetry.logVisionRejected(rejected.toArray(new Pose3d[0]));
        telemetry.logVisionTagPoses(tagPoses.toArray(new Pose3d[0]));
        telemetry.logVisionFusedCount(fusedCount);
    }

    public boolean isVisionSeeded(){
        return startupPoseSeeded;
    }

    private class Camera {
        final String name;
        final PhotonCamera photonCamera;
        final PhotonPoseEstimator estimator;
        final Debouncer disabledDebounce;
        final Alert disconnectedAlert;

        Rotation2d latestTargetYaw = Rotation2d.kZero;
        Rotation2d latestTargetPitch = Rotation2d.kZero;
        boolean hasTarget = false;

        Camera(CameraConfig config) {
            this.name = config.name();
            this.photonCamera = new PhotonCamera(config.name());
            this.estimator = new PhotonPoseEstimator(APRILTAG_LAYOUT, config.robotToCamera());
            this.disabledDebounce = new Debouncer(5.0, DebounceType.kFalling);
            this.disconnectedAlert = new Alert(
                "Vision camera " + config.name() + " is disconnected.",
                AlertType.kWarning);
        }
    }
}
