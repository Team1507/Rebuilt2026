//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.photonvision;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import frc.lib.hardware.PhotonVisionHardware;
import frc.robot.localization.vision.PhotonPoseEstimator;
import frc.lib.math.FieldInfo;

/**
 * Real PhotonVision implementation with explicit camera names.
 *
 * This class wraps PhotonCamera + custom PhotonPoseEstimator and produces
 * raw observations for PVManager + PVPerCameraProcessor.
 */
public class PhotonVisionIOReal implements PhotonVisionIO {

    private final PhotonCamera[] cameras;
    private final PhotonPoseEstimator[] estimators;
    private final String[] cameraNames;

    private final Supplier<Rotation2d> headingSupplier;

    /**
     * Creates a PhotonVision IO layer.
     */
    public PhotonVisionIOReal(Supplier<Rotation2d> headingSupplier) {
        this.headingSupplier = headingSupplier;

        int count = PhotonVisionHardware.CAMERAS.length;

        cameras = new PhotonCamera[count];
        estimators = new PhotonPoseEstimator[count];
        cameraNames = new String[count];

        for (int i = 0; i < count; i++) {
            var cfg = PhotonVisionHardware.CAMERAS[i];

            cameraNames[i] = cfg.name;
            cameras[i] = new PhotonCamera(cfg.name);

            // 340-style custom estimator: (AprilTag layout, robotToCamera)
            estimators[i] = new PhotonPoseEstimator(
                FieldInfo.aprilTags(),
                cfg.robotToCamera
            );
        }
    }

    @Override
    public int getCameraCount() {
        return cameras.length;
    }

    @Override
    public String[] getCameraNames() {
        return cameraNames;
    }

    @Override
    public PhotonPoseEstimator getEstimator(int index) {
        return estimators[index];
    }

    @Override
    public void updateInputs(PhotonVisionInputs inputs, boolean seeded) {

        double now = Timer.getFPGATimestamp();
        Rotation2d heading = headingSupplier.get();

        // Feed heading data into each estimator
        for (int i = 0; i < cameras.length; i++) {
            estimators[i].addHeadingData(now, heading);
        }

        // Ensure array is sized correctly
        if (inputs.cameras.length != cameras.length) {
            inputs.cameras = new PhotonVisionInputs.CameraInputs[cameras.length];
            for (int i = 0; i < cameras.length; i++) {
                inputs.cameras[i] = new PhotonVisionInputs.CameraInputs();
                inputs.cameras[i].name = cameraNames[i];
            }
        }

        inputs.anyCameraConnected = false;
        inputs.totalTags = 0; // will be updated by processors if needed

        for (int i = 0; i < cameras.length; i++) {
            PhotonCamera cam = cameras[i];
            PhotonVisionInputs.CameraInputs out = inputs.cameras[i];

            out.connected = cam.isConnected();
            if (out.connected) inputs.anyCameraConnected = true;

            var results = cam.getAllUnreadResults();
            if (results.isEmpty()) {
                out.hasTarget = false;
                out.rawResult = null;
                continue;
            }

            PhotonPipelineResult result = results.get(results.size() - 1);
            out.rawResult = result;
            out.hasTarget = result.hasTargets();
            out.timestamp = now; // or result.getTimestampSeconds() if you prefer

            if (out.hasTarget) {
                out.yaw = Rotation2d.fromDegrees(result.getBestTarget().getYaw());
                out.pitch = Rotation2d.fromDegrees(result.getBestTarget().getPitch());
            }
        }
    }
}
