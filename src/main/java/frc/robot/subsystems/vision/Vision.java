// Team 1507 Warlocks

package frc.robot.subsystems.vision;

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
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kQuest;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Vision extends SubsystemBase {

    public static record CameraConfig(String name, Transform3d robotToCamera) {}

    @FunctionalInterface
    public static interface VisionConsumer {
        void accept(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs);
    }

    private final VisionConsumer consumer;
    private final Supplier<Rotation2d> headingSupplier;
    private final Consumer<Pose2d> poseSeeder;
    private final Consumer<Pose3d> questNavPoseSeeder;
    private final Camera[] cameras;

    private boolean startupPoseSeeded = false;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PROCESS_PERIOD = 0.05;

    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private final NetworkTable visionPoseTable = ntInst.getTable("VisionPose");

    private final DoubleArrayPublisher visionFieldPub =
        visionPoseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher visionFieldTypePub =
        visionPoseTable.getStringTopic(".type").publish();

    private final StructPublisher<Pose2d> visionPosePub =
        visionPoseTable.getStructTopic("Pose", Pose2d.struct).publish();

    @SuppressWarnings("unchecked")
    private StructPublisher<Pose2d>[] cameraPosePubs = new StructPublisher[0];

    public Vision(
        VisionConsumer consumer,
        Supplier<Rotation2d> headingSupplier,
        Consumer<Pose2d> poseSeeder,
        Consumer<Pose3d> questNavPoseSeeder,
        CameraConfig... configs
    ) {
        this.consumer = consumer;
        this.headingSupplier = headingSupplier;
        this.poseSeeder = poseSeeder;
        this.questNavPoseSeeder = questNavPoseSeeder;

        this.cameras = new Camera[configs.length];

        @SuppressWarnings("unchecked")
        StructPublisher<Pose2d>[] pubs = new StructPublisher[configs.length];
        for (int i = 0; i < configs.length; i++) {
            cameras[i] = new Camera(configs[i]);
            pubs[i] = visionPoseTable
                .getStructTopic(configs[i].name() + "/Pose", Pose2d.struct)
                .publish();
        }
        this.cameraPosePubs = pubs;
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

        for (var cam : cameras) {
            cam.estimator.addHeadingData(now, heading);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allAccepted = new LinkedList<>();
        List<Pose3d> allRejected = new LinkedList<>();

        Pose2d seedPose = null;
        double seedScore = Double.NEGATIVE_INFINITY;

        Pose2d latestVisionPose = null;

        int totalFused = 0;

        for (int i = 0; i < cameras.length; i++) {
            Camera cam = cameras[i];

            boolean usingTrig;
            if (!startupPoseSeeded) {
                usingTrig = false;
            } else {
                usingTrig = cam.disabledDebounce.calculate(DriverStation.isEnabled());
            }

            boolean connected = cam.photonCamera.isConnected();
            cam.disconnectedAlert.set(!connected);

            cam.hasTarget = false;

            var allResults = cam.photonCamera.getAllUnreadResults();

            // Only process the latest result
            if (allResults.isEmpty()) {
                continue;
            }

            PhotonPipelineResult result = allResults.get(allResults.size() - 1);

            if (result.hasTargets()) {
                cam.hasTarget = true;
                cam.latestTargetYaw =
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                cam.latestTargetPitch =
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch());
            }

            Optional<EstimatedRobotPose> estimate = Optional.empty();

            if (usingTrig) {
                estimate = cam.estimator.estimatePnpDistanceTrigSolvePose(result);
                if (estimate.isEmpty()) {
                    estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                }
            } else {
                estimate = cam.estimator.estimateCoprocMultiTagPose(result);
                if (estimate.isEmpty()) {
                    estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                }
            }

            if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) {
                continue;
            }

            var est = estimate.get();
            Pose3d robotPose3d = est.estimatedPose;
            int tagCount = est.targetsUsed.size();

            double totalDist = 0.0;
            for (var target : est.targetsUsed) {
                totalDist += target.getBestCameraToTarget().getTranslation().getNorm();
            }
            double avgDistance = totalDist / tagCount;

            var primaryTarget = est.targetsUsed.get(0);

            boolean reject =
                Math.abs(robotPose3d.getZ()) > maxZError
                    || robotPose3d.getX() < 0.0
                    || robotPose3d.getX() > APRILTAG_LAYOUT.getFieldLength()
                    || robotPose3d.getY() < 0.0
                    || robotPose3d.getY() > APRILTAG_LAYOUT.getFieldWidth();

            if (avgDistance > maxTagDistance) reject = true;

            if (tagCount == 1 && primaryTarget.getPoseAmbiguity() > maxAmbiguity) {
                reject = true;
            }

            if (reject) {
                allRejected.add(robotPose3d);
            } else {
                allAccepted.add(robotPose3d);

                for (var target : est.targetsUsed) {
                    APRILTAG_LAYOUT.getTagPose(target.getFiducialId())
                        .ifPresent(allTagPoses::add);
                }

                if (i < cameraPosePubs.length) {
                    cameraPosePubs[i].set(robotPose3d.toPose2d());
                }
                latestVisionPose = robotPose3d.toPose2d();

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

                    if (i < cameraStdDevFactors.length) {
                        xyStd *= cameraStdDevFactors[i];
                        angStd *= cameraStdDevFactors[i];
                    }

                    Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, angStd);
                    consumer.accept(robotPose3d.toPose2d(), est.timestampSeconds, stdDevs);
                    totalFused++;
                }
            }

            String camName = cam.name;
        }

        if (!startupPoseSeeded && seedPose != null) {
            poseSeeder.accept(seedPose);
            questNavPoseSeeder.accept(new Pose3d(seedPose).transformBy(kQuest.ROBOT_TO_QUEST));
            resetHeadingData();
            startupPoseSeeded = true;

            Logger.recordOutput("Vision/StartupPoseSeeded", seedPose);
        }

        if (latestVisionPose != null) {
            visionPosePub.set(latestVisionPose);
            visionFieldTypePub.set("Field2d");
            visionFieldPub.set(new double[] {
                latestVisionPose.getX(),
                latestVisionPose.getY(),
                latestVisionPose.getRotation().getDegrees()
            });
        }

        Logger.recordOutput("Vision/FusedObservationCount", totalFused);
        Logger.recordOutput("Vision/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/AcceptedPoses", allAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput("Vision/RejectedPoses", allRejected.toArray(new Pose3d[0]));
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

            this.estimator = new PhotonPoseEstimator(
                APRILTAG_LAYOUT,
                config.robotToCamera());

            this.disabledDebounce = new Debouncer(5.0, DebounceType.kFalling);

            this.disconnectedAlert = new Alert(
                "Vision camera " + config.name() + " is disconnected.",
                AlertType.kWarning);
        }
    }
}
