package frc.robot.subsystems;

// PhotonVision Imports
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// WPI libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.Timer;

// Vision base class
import frc.robot.subsystems.lib.Vision1507;

// Robot Utilities
import frc.robot.utilities.Telemetry;

// Robot Constants
import static frc.robot.Constants.Vision.*;

/**
 * VisionSystem implementation for PhotonVision.
 *
 * Responsibilities:
 *  - Read AprilTag detections from PhotonVision
 *  - Run PhotonPoseEstimator
 *  - Accept or invalidate poses via VisionSystem helpers
 *  - Publish diagnostics safely
 */
public class PhotonVisionSubsystem extends Vision1507 {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d cameraToRobot;

    /**
     * Constructor for the PhotonVisionSubsystem.
     *
     * @param drivetrain The drivetrain whose pose may be reset by this system.
     * @param logger     Telemetry logger for publishing vision poses.
     * @param cameraName The name of the camera as configured in PhotonVision UI.
     */
    public PhotonVisionSubsystem(
        CommandSwerveDrivetrain drivetrain,
        Telemetry logger,
        String cameraName,
        Transform3d cameraToRobot
    ) {
        super(drivetrain, logger);

        this.camera = new PhotonCamera(cameraName);
        this.cameraToRobot = cameraToRobot;

        this.poseEstimator = new PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY,
            cameraToRobot
        );
    }

    /**
     * Updates the vision pose.
     * Called automatically by VisionSystem.periodic().
     */
    @Override
    protected void update() {

        PhotonPipelineResult result = camera.getLatestResult();

        // ------------------------------------------------------------
        // Basic diagnostics
        // ------------------------------------------------------------
        SmartDashboard.putBoolean(camera.getName() + "/HasTarget", result.hasTargets());
        SmartDashboard.putNumber(camera.getName() + "/TagCount", result.getTargets().size());

        // ------------------------------------------------------------
        // Per‑target debug info (safe even if empty)
        // ------------------------------------------------------------
        for (var t : result.getTargets()) {
            SmartDashboard.putNumber(camera.getName() + "/Tag " + t.getFiducialId() + "/Yaw", t.getYaw());
            SmartDashboard.putNumber(camera.getName() + "/Tag " + t.getFiducialId() + "/Pitch", t.getPitch());
            SmartDashboard.putNumber(camera.getName() + "/Tag " + t.getFiducialId() + "/Area", t.getArea());
        }

        // ------------------------------------------------------------
        // Ambiguity (guarded)
        // ------------------------------------------------------------
        if (result.hasTargets() && result.getBestTarget() != null) {
            SmartDashboard.putNumber(
                camera.getName() + "/BestAmbiguity",
                result.getBestTarget().getPoseAmbiguity()
            );
        } else {
            SmartDashboard.putNumber(camera.getName() + "/BestAmbiguity", -1);
        }

        // If no targets at all, invalidate and bail 
        if (!result.hasTargets()) {
            invalidatePose(); 
            return; 
        }

        // ------------------------------------------------------------
        // Quality filters
        // ------------------------------------------------------------

        if (result.getBestTarget() != null && result.getBestTarget().getPoseAmbiguity() > 0.25) {
            invalidatePose();
            return;
        }

        if (result.getTargets().size() == 1) {
            double dist = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            if (dist > 2.5) {
                invalidatePose();
                return;
            }
        }

        if (Math.abs(result.getBestTarget().getYaw()) > 25) {
            invalidatePose();
            return;
        }

        // ------------------------------------------------------------
        // Pose estimation
        // ------------------------------------------------------------
        var estimated = poseEstimator.update(result);
        SmartDashboard.putBoolean(camera.getName() + "/PoseEstimated", estimated.isPresent());

        if (estimated.isEmpty()) {
            invalidatePose();
            return;
        }

        EstimatedRobotPose pose = estimated.get();

        // ------------------------------------------------------------
        // Accept pose
        // ------------------------------------------------------------
        acceptPose(
            pose.estimatedPose.toPose2d(),
            pose.timestampSeconds
        );

        Pose2d robotPose = drivetrain.getState().Pose;

        Translation3d t3 = cameraToRobot.getTranslation();
        Rotation3d r3 = cameraToRobot.getRotation();

        Translation2d t2 = new Translation2d(t3.getX(), t3.getY());
        Rotation2d r2 = r3.toRotation2d();

        Transform2d cameraOffset = new Transform2d(t2, r2);

        Pose2d cameraPose = robotPose.transformBy(cameraOffset);
        SmartDashboard.putNumber(camera.getName() + "/Pose/X",cameraPose.getX());
        SmartDashboard.putNumber(camera.getName() + "/Pose/Y",cameraPose.getY());
        SmartDashboard.putNumber(camera.getName() + "/Pose/ROT",cameraPose.getRotation().getDegrees());

        SmartDashboard.putNumber(camera.getName() + "/PV Timerstamp", getLastPoseTimestamp());
        SmartDashboard.putNumber(camera.getName() + "/RIO Timestamp", Timer.getFPGATimestamp());
        SmartDashboard.putNumber(camera.getName() + "/Delta", Timer.getFPGATimestamp() -getLastPoseTimestamp());
    }

    @Override
    protected void addVisionMeasurementToDrivetrain() {
        getLatestPose().ifPresent(pose -> {

            PhotonPipelineResult result = camera.getLatestResult();
            Matrix<N3, N1> dynamicStdDev = computeStdDevs(result);

            drivetrain.addVisionMeasurement(
                pose,
                getLastPoseTimestamp(),
                dynamicStdDev
            );
        });
    }

    private Matrix<N3, N1> computeStdDevs(PhotonPipelineResult result) {

        // If no targets, return very large uncertainty
        if (!result.hasTargets()) {
            return VecBuilder.fill(0.50, 0.50, 0.80); // basically ignore
        }

        // -----------------------------
        // 1. Ambiguity
        // -----------------------------
        double ambiguity = 1.0;
        if (result.getBestTarget() != null) {
            ambiguity = result.getBestTarget().getPoseAmbiguity();
        }
        double ambScale = 1.0 + 3.0 * ambiguity;

        // -----------------------------
        // 2. Tag count
        // -----------------------------
        int tagCount = result.getTargets().size();
        double tagScale = 1.0 / Math.sqrt(tagCount);

        // -----------------------------
        // 3. Distance scaling
        // -----------------------------
        double avgDist = 0.0;
        for (var t : result.getTargets()) {
            avgDist += t.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= tagCount;

        double distScale = 1.0 + 0.25 * avgDist;

        // -----------------------------
        // 4. Combine scales
        // -----------------------------
        double finalScale = ambScale * tagScale * distScale;

        // -----------------------------
        // 5. Apply to baseline
        // -----------------------------
        double BASE_XY = 0.06;   // 6 cm
        double BASE_ROT = 0.12;  // ~7 degrees

        double xyStd = BASE_XY * finalScale;
        double rotStd = BASE_ROT * finalScale;

        return VecBuilder.fill(xyStd, xyStd, rotStd);
    }

}
