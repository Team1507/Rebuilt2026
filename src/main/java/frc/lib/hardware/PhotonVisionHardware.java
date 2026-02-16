//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants.kVision;

/**
 * Hardware definitions for PhotonVision cameras.
 *
 * This contains ONLY physical configuration:
 *  - camera names
 *  - robot-to-camera transforms
 *  - number of cameras
 *
 * No PhotonVision classes or logic belong here.
 */
public final class PhotonVisionHardware {

    public static final class CameraConfig {
        public final String name;
        public final Transform3d robotToCamera;

        public CameraConfig(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    // Example: two-camera system (BLU + YEL)
    public static final CameraConfig[] CAMERAS = new CameraConfig[] {
        new CameraConfig("Bluecam", kVision.BLU.ROBOT_TO_CAMERA),
        new CameraConfig("Yellowcam", kVision.YEL.ROBOT_TO_CAMERA)
    };

    private PhotonVisionHardware() {}
}
