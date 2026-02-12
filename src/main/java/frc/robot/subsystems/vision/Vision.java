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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kAgitator;
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

// Uses the official v2026 PhotonPoseEstimator API. When the robot is enabled we use
// estimatePnpDistanceTrigSolvePose for fast XY (it leans on the gyro for heading).
// When disabled or at startup we use estimateCoprocMultiTagPose for a full 3D pose
// from the coprocessor, with estimateLowestAmbiguityPose as a fallback when we only
// see one tag. All valid observations from both cameras get fused into the drivetrain
// each cycle. We scale std devs by average tag distance so closer tags are trusted more,
// and multi-tag observations get a sqrt(tagCount) discount. At startup we prefer
// multi-tag or lowest-ambiguity so the initial pose has its own heading instead of
// just echoing the gyro. Vision pose is published to the VisionPose NetworkTable so
// you can see it on the field in AdvantageScope.
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

    private final NetworkTableInstance ntInst = NetworkTableInstance.getDefault();
    private final NetworkTable visionPoseTable = ntInst.getTable("VisionPose");

    // Field2d-compatible so it shows up in AdvantageScope and Shuffleboard
    private final DoubleArrayPublisher visionFieldPub =
        visionPoseTable.getDoubleArrayTopic("robotPose").publish();
    private final StringPublisher visionFieldTypePub =
        visionPoseTable.getStringTopic(".type").publish();

    // Exact Pose2d for consumers that want the full struct
    private final StructPublisher<Pose2d> visionPosePub =
        visionPoseTable.getStructTopic("Pose", Pose2d.struct).publish();

    // One publisher per camera for debugging
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

    // Call after resetting pose or gyro. Clears the heading buffer and seeds it with current heading.
    public void resetHeadingData() {
        double now = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();
        for (var cam : cameras) {
            cam.estimator.resetHeadingData(now, heading);
        }
    }

    // Yaw to best target on this camera, used for aiming
    public Rotation2d getTargetX(int cameraIndex) {
        return cameras[cameraIndex].latestTargetYaw;
    }

    // Pitch to best target on this camera
    public Rotation2d getTargetY(int cameraIndex) {
        return cameras[cameraIndex].latestTargetPitch;
    }

    // True if this camera currently sees at least one target
    public boolean hasTarget(int cameraIndex) {
        return cameras[cameraIndex].hasTarget;
    }

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();

        // Must feed heading every frame for trig solve and constrained PnP
        for (var cam : cameras) {
            cam.estimator.addHeadingData(now, heading);
        }

        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allAccepted = new LinkedList<>();
        List<Pose3d> allRejected = new LinkedList<>();

        // Best candidate we've seen so far for startup seeding
        Pose2d seedPose = null;
        double seedScore = Double.NEGATIVE_INFINITY;

        // Most recent accepted pose, for publishing
        Pose2d latestVisionPose = null;

        int totalFused = 0;

        for (int i = 0; i < cameras.length; i++) {
            Camera cam = cameras[i];

            // Before we've seeded pose we want multi-tag or lowest-ambiguity so we get
            // a real heading instead of echoing the gyro. After that, when enabled we use
            // trig solve for fast XY. When disabled we stick with multi-tag for full pose.
            boolean usingTrig;
            if (!startupPoseSeeded) {
                usingTrig = false;
            } else {
                // Wait 5 seconds after disable before switching strategies so we don't flicker
                usingTrig = cam.disabledDebounce.calculate(DriverStation.isEnabled());
            }

            boolean connected = cam.photonCamera.isConnected();
            cam.disconnectedAlert.set(!connected);

            int acceptedCount = 0;
            int rejectedCount = 0;
            int resultCount = 0;
            int targetResultCount = 0;
            cam.hasTarget = false;

            var allResults = cam.photonCamera.getAllUnreadResults();
            resultCount = allResults.size();

            for (PhotonPipelineResult result : allResults) {

                // Grab yaw and pitch for aiming
                if (result.hasTargets()) {
                    cam.hasTarget = true;
                    targetResultCount++;
                    cam.latestTargetYaw =
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                    cam.latestTargetPitch =
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch());
                }

                // Pose estimation via the v2026 PhotonPoseEstimator methods
                Optional<EstimatedRobotPose> estimate = Optional.empty();

                if (usingTrig) {
                    // Trig solve uses gyro for heading and gives us good XY fast
                    estimate = cam.estimator.estimatePnpDistanceTrigSolvePose(result);
                    if (estimate.isEmpty()) {
                        estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                    }
                } else {
                    // Multi-tag runs on the coprocessor and gives us full 3D pose with its own heading
                    estimate = cam.estimator.estimateCoprocMultiTagPose(result);
                    if (estimate.isEmpty()) {
                        estimate = cam.estimator.estimateLowestAmbiguityPose(result);
                    }
                }

                if (estimate.isEmpty() || estimate.get().targetsUsed.isEmpty()) continue;

                var est = estimate.get();
                Pose3d robotPose3d = est.estimatedPose;
                int tagCount = est.targetsUsed.size();

                // Average distance to all tags we used
                double totalDist = 0.0;
                for (var target : est.targetsUsed) {
                    totalDist +=
                        target.getBestCameraToTarget().getTranslation().getNorm();
                }
                double avgDistance = totalDist / tagCount;

                var primaryTarget = est.targetsUsed.get(0);

                // Reject obviously bogus poses
                boolean reject =
                    Math.abs(robotPose3d.getZ()) > maxZError
                        || robotPose3d.getX() < 0.0
                        || robotPose3d.getX() > APRILTAG_LAYOUT.getFieldLength()
                        || robotPose3d.getY() < 0.0
                        || robotPose3d.getY() > APRILTAG_LAYOUT.getFieldWidth();

                // Tag too far away to trust
                if (avgDistance > maxTagDistance) {
                    reject = true;
                }

                // Single-tag with high ambiguity tends to be wrong
                if (tagCount == 1 && primaryTarget.getPoseAmbiguity() > maxAmbiguity) {
                    reject = true;
                }

                if (reject) {
                    rejectedCount++;
                    allRejected.add(robotPose3d);
                    continue;
                }

                acceptedCount++;
                allAccepted.add(robotPose3d);

                // Track which tags contributed
                for (var target : est.targetsUsed) {
                    APRILTAG_LAYOUT.getTagPose(target.getFiducialId())
                        .ifPresent(allTagPoses::add);
                }

                // Send this camera's pose to NT so we can debug
                if (i < cameraPosePubs.length) {
                    cameraPosePubs[i].set(robotPose3d.toPose2d());
                }
                latestVisionPose = robotPose3d.toPose2d();

                // Before first seed we just pick the best candidate; don't fuse yet
                if (!startupPoseSeeded) {
                    double score = tagCount * 100.0 - avgDistance * 10.0;
                    if (score > seedScore) {
                        seedScore = score;
                        seedPose = robotPose3d.toPose2d();
                    }
                    continue;
                }

                // Fuse into the drivetrain estimator. Std devs scale with distance so
                // close tags are trusted more. Trig solve is great for XY but doesn't
                // give heading so we use a huge angular std dev there.
                double xyStd =
                    (usingTrig ? trigXyStdBase : constrainedPnpXyStdBase) * avgDistance;
                double angStd =
                    (usingTrig ? 1e5 : constrainedPnpAngStdBase) * avgDistance;

                // Multi-tag is more reliable so we tighten the std devs
                if (tagCount > 1) {
                    double discount = Math.sqrt(tagCount);
                    xyStd /= discount;
                    angStd /= discount;
                }

                // Apply per-camera trust factor
                if (i < cameraStdDevFactors.length) {
                    xyStd *= cameraStdDevFactors[i];
                    angStd *= cameraStdDevFactors[i];
                }

                Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStd, xyStd, angStd);
                consumer.accept(robotPose3d.toPose2d(), est.timestampSeconds, stdDevs);
                questNavPoseSeeder.accept(robotPose3d.transformBy(kQuest.ROBOT_TO_QUEST));
                totalFused++;
            }

            // Dashboard values for debugging. Connected false usually means camera name doesn't
            // match the PhotonVision UI. ResultCount 0 means no frames at all (name or NT). 
            // TargetResultCount 0 means frames came through but no AprilTags were seen. 
            // If AcceptedCount is 0 but RejectedCount is high, poses are being thrown out.
            String camName = cam.name;
            SmartDashboard.putBoolean("Vision/" + camName + "/Connected", connected);
            SmartDashboard.putBoolean("Vision/" + camName + "/HasTarget", cam.hasTarget);
            SmartDashboard.putNumber("Vision/" + camName + "/ResultCount", resultCount);
            SmartDashboard.putNumber("Vision/" + camName + "/TargetResultCount", targetResultCount);
            SmartDashboard.putNumber(
                "Vision/" + camName + "/AcceptedCount", acceptedCount);
            SmartDashboard.putNumber(
                "Vision/" + camName + "/RejectedCount", rejectedCount);
            SmartDashboard.putBoolean("Vision/" + camName + "/UsingTrig", usingTrig);
        }

        // On first valid pose we seed the drivetrain. We use multi-tag or lowest-ambiguity
        // so we get a real heading instead of echoing the gyro. Then reset the heading
        // buffer since the gyro was just reset.
        if (!startupPoseSeeded && seedPose != null) {
            poseSeeder.accept(seedPose);
            resetHeadingData();
            startupPoseSeeded = true;

            Logger.recordOutput("Vision/StartupPoseSeeded", seedPose);
        }

        // Publish overall vision pose so we can see it on the field in AdvantageScope
        // or Shuffleboard next to the drivetrain pose.
        if (latestVisionPose != null) {
            visionPosePub.set(latestVisionPose);
            visionFieldTypePub.set("Field2d");
            visionFieldPub.set(new double[] {
                latestVisionPose.getX(),
                latestVisionPose.getY(),
                latestVisionPose.getRotation().getDegrees()
            });
        }

        // AdvantageKit logging
        Logger.recordOutput("Vision/FusedObservationCount", totalFused);
        Logger.recordOutput("Vision/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/AcceptedPoses", allAccepted.toArray(new Pose3d[0]));
        Logger.recordOutput(
            "Vision/RejectedPoses", allRejected.toArray(new Pose3d[0]));
    }

    // One PhotonCamera plus its pose estimator
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

            // v2026 PhotonPoseEstimator. We pick strategy per-result with the estimate*Pose methods.
            this.estimator = new PhotonPoseEstimator(
                APRILTAG_LAYOUT,
                config.robotToCamera());

            // Wait 5 seconds after disable before we switch away from trig
            this.disabledDebounce = new Debouncer(5.0, DebounceType.kFalling);

            this.disconnectedAlert = new Alert(
                "Vision camera " + config.name() + " is disconnected.",
                AlertType.kWarning);
        }
    }
}
