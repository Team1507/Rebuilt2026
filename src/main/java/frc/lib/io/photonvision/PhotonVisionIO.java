//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * IO abstraction for PhotonVision.
 *
 * PVManager will depend on this interface instead of directly
 * touching PhotonCamera or PhotonPoseEstimator.
 */
public interface PhotonVisionIO {

    /** Container for per-camera inputs. */
    public static class CameraInputs {
        public boolean hasTarget = false;
        public Rotation2d latestYaw = Rotation2d.kZero;
        public Rotation2d latestPitch = Rotation2d.kZero;

        public Optional<Pose3d> latestPose3d = Optional.empty();
        public Optional<Pose2d> latestPose2d = Optional.empty();

        public double timestampSeconds = 0.0;
        public Matrix<N3, N1> stdDevs = null;

        public int tagCount = 0;
        public double avgDistance = 0.0;
    }

    /** Update all camera inputs. */
    void updateInputs(CameraInputs[] inputs);

    /** Number of cameras in the system. */
    int getCameraCount();
}
