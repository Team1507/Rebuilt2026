//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Simple simulation implementation of SwerveIO.
 *
 * <p>This does not attempt to model real physics yet — it simply
 * integrates chassis speeds into a pose estimate. You can expand
 * this later with a full physics model if desired.
 */
public class SwerveIOSim implements SwerveIO {

    private Pose2d pose = new Pose2d();
    private ChassisSpeeds speeds = new ChassisSpeeds();
    private SwerveModuleState[] moduleStates = new SwerveModuleState[0];
    private Rotation2d heading = new Rotation2d();

    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.pose = pose;
        inputs.speeds = speeds;
        inputs.moduleStates = moduleStates;
    }

    @Override
    public void drive(ChassisSpeeds speeds) {
        this.speeds = speeds;
        // Simple integration placeholder
        pose = new Pose2d(
            pose.getX() + speeds.vxMetersPerSecond * 0.02,
            pose.getY() + speeds.vyMetersPerSecond * 0.02,
            pose.getRotation().plus(
                edu.wpi.first.math.geometry.Rotation2d.fromRadians(
                    speeds.omegaRadiansPerSecond * 0.02
                )
            )
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {}

    @Override
    public void setModuleStates(SwerveModuleState[] states) {
        this.moduleStates = states;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public void resetPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> visionMeasurementStdDevs) {}

    @Override
    public void alignGyro(Rotation2d heading) {
        this.heading = heading;
    }

    @Override
    public Rotation2d getHeading() {
        return heading;
    }

    @Override
    public void stop() {
        speeds = new ChassisSpeeds();
    }
}
