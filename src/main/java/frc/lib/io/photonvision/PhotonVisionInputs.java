//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;

/**
 * IO container for all PhotonVision camera data.
 *
 * This matches the AdvantageKit IO pattern:
 *  - Per-camera raw observations
 *  - A fused PV pose (computed by PVManager)
 *  - Metadata for connectivity
 */
public class PhotonVisionInputs {

    /** Per-camera inputs */
    public static class CameraInputs {
        public String name = "";

        public boolean connected = false;

        public boolean hasTarget = false;
        public Rotation2d yaw = Rotation2d.kZero;
        public Rotation2d pitch = Rotation2d.kZero;

        public Optional<Pose3d> pose3d = Optional.empty(); // unused by new pipeline
        public Optional<Pose2d> pose2d = Optional.empty(); // unused by new pipeline

        public double timestamp = 0.0;

        /** Last raw PhotonVision pipeline result for this camera. */
        public PhotonPipelineResult rawResult = null;
    }

    /** Array of per-camera inputs */
    public CameraInputs[] cameras = new CameraInputs[0];

    /** Fused PV pose (computed by PVManager) */
    public Optional<Pose2d> fusedPose = Optional.empty();
    public double fusedTimestamp = 0.0;

    /** Metadata */
    public int totalTags = 0;
    public boolean anyCameraConnected = false;
}
