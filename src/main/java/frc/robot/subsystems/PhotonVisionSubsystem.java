package frc.robot.subsystems;

// PhotonVision Imports
import org.photonvision.PhotonCamera;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

// WPI libraries
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.Matrix;

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
    private final Matrix<N3, N1> std_dev;

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
        Transform3d cameraToRobot,
        Matrix<N3, N1> std_dev
    ) {
        super(drivetrain, logger);

        this.camera = new PhotonCamera(cameraName);

        this.poseEstimator = new PhotonPoseEstimator(
            APRILTAG_LAYOUT,
            PoseStrategy.LOWEST_AMBIGUITY,
            cameraToRobot
        );

        this.std_dev = std_dev;
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
    }

    public void addVisionMeasurementToDrivetrain() {
        getLatestPose().ifPresent(pose -> {
            drivetrain.addVisionMeasurement(
                pose,
                getLastPoseTimestamp(),
                std_dev
            );
        });
    }
}