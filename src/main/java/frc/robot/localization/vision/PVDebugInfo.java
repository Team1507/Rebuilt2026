package frc.robot.localization.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Structured debug information for PhotonVision.
 *
 * PVManager fills this object every cycle, and DashboardManager
 * reads it to publish detailed telemetry.
 */
public final class PVDebugInfo {

    /** Debug information for the single Bluecam camera */
    public static final class CameraDebug {
        public String name = "Bluecam";

        // Raw camera state
        public boolean connected = false;
        public boolean hasTarget = false;
        public int fiducialId = -1;
        public double timestamp = 0.0;

        // Processor-level metrics
        public int tagCount = 0;
        public double avgDistance = 0.0;
        public double ambiguity = 0.0;

        // Strategy + acceptance
        public String strategyUsed = "NONE";
        public boolean accepted = false;
        public String rejectionReason = "";

        // Computed standard deviations
        public double xyStd = 0.0;
        public double angStd = 0.0;
    }

    /** Single-camera debug packet (Bluecam only) */
    public CameraDebug camera = new CameraDebug();

    /** Fused PV output */
    public boolean fusedValid = false;
    public Pose2d fusedPose = new Pose2d();
    public double fusedTimestamp = 0.0;
    public double fusedXyStd = 0.0;
    public double fusedAngStd = 0.0;

    /** PVManager state */
    public boolean seeded = false;
    public String tagMode = "HUB";
}
