//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.localization;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.core.logging.Telemetry;
import frc.robot.localization.vision.PVManager;
import frc.robot.localization.vision.QuestNavManager;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * LocalizationManager:
 *
 *  - QuestNav is primary during ENABLED (translation + heading)
 *  - PhotonVision only corrects when avgDistance < 2m and stable
 *  - PhotonVision fully controls pose when DISABLED
 *  - Gyro is fallback only
 *  - Startup seeding uses PV stability
 */
public class LocalizationManager extends SubsystemBase {

    private final SwerveSubsystem swerve;
    private final PVManager pv;
    private final QuestNavManager quest;
    private final Telemetry telemetry;

    private boolean startupSeeded = false;
    private int pvStableCount = 0;

    // 20 Hz throttle
    private double lastProcessTime = 0.0;
    private static final double PERIOD = 0.05;

    public LocalizationManager(
        SwerveSubsystem swerve,
        PVManager pv,
        QuestNavManager quest,
        Telemetry telemetry
    ) {
        this.swerve = swerve;
        this.pv = pv;
        this.quest = quest;
        this.telemetry = telemetry;

        telemetry.registerVisionPoseSource("Localization-Fused");
    }

    private boolean pvClose() {
        return pv.hasGoodVision() && pv.getFusedAvgDistance() < 2.0;
    }

    @Override
    public void periodic() {

        double now = Timer.getFPGATimestamp();
        if (now - lastProcessTime < PERIOD) return;
        lastProcessTime = now;

        // ============================
        // 1. Read all sources
        // ============================

        var pvPoseOpt = pv.getFusedPose();
        boolean pvGood = pv.hasGoodVision();
        boolean pvClose = pvClose();

        boolean questGood = quest.isTracking();
        Pose2d questPose = quest.getPose2d();
        double questTs = quest.getTimestamp();

        Pose2d odomPose = swerve.getPose();

        boolean enabled = DriverStation.isEnabled();

        // ============================
        // 2. Startup seeding (PV only)
        // ============================

        if (!startupSeeded) {

            boolean pvStable =
                pvGood &&
                pv.getFusedXyStd() < 1.0 &&
                pv.getFusedAngStd() < 0.7;

            if (pvStable) pvStableCount++;
            else pvStableCount = 0;

            if (pvStableCount >= 5) {
                Pose2d seed = pvPoseOpt.get();

                // Seed drivetrain
                swerve.seedPoseFromVision(seed);

                // Seed QuestNav
                quest.seedPose(new Pose3d(seed));

                startupSeeded = true;
                pv.setSeeded(true);

                telemetry.logVisionStartupSeed(seed);
            }
        }

        // ============================
        // 3. Choose translation source
        // ============================

        Pose2d translationSource;

        if (!enabled) {
            // Disabled → PV owns translation
            if (pvGood) {
                translationSource = pvPoseOpt.get();
                telemetry.logLocalizationTranslationSource("PhotonVision (Disabled)");
            } else if (questGood) {
                translationSource = questPose;
                telemetry.logLocalizationTranslationSource("QuestNav (Disabled-Fallback)");
            } else {
                translationSource = odomPose;
                telemetry.logLocalizationTranslationSource("Odometry (Disabled-Fallback)");
            }
        }
        else {
            // Enabled → QuestNav owns translation
            if (questGood) {
                translationSource = questPose;
                telemetry.logLocalizationTranslationSource("QuestNav");
            }
            else if (pvClose) {
                translationSource = pvPoseOpt.get();
                telemetry.logLocalizationTranslationSource("PhotonVision (Close-Range Fallback)");
            }
            else {
                translationSource = odomPose;
                telemetry.logLocalizationTranslationSource("Odometry (Fallback)");
            }
        }

        // ============================
        // 4. Choose heading source
        // ============================

        Rotation2d heading;

        if (!enabled) {
            // Disabled → PV heading
            if (pvGood) {
                heading = pvPoseOpt.get().getRotation();
                telemetry.logLocalizationHeadingSource("PhotonVision (Disabled)");
            } else if (questGood) {
                heading = questPose.getRotation();
                telemetry.logLocalizationHeadingSource("QuestNav (Disabled-Fallback)");
            } else {
                heading = swerve.getHeading();
                telemetry.logLocalizationHeadingSource("Gyro (Disabled-Fallback)");
            }
        }
        else {
            // Enabled → QuestNav heading
            if (questGood) {
                heading = questPose.getRotation();
                telemetry.logLocalizationHeadingSource("QuestNav");
            } else {
                heading = swerve.getHeading();
                telemetry.logLocalizationHeadingSource("Gyro (Fallback)");
            }
        }

        // ============================
        // 5. Build fused pose
        // ============================

        Pose2d fusedPose = new Pose2d(
            translationSource.getX(),
            translationSource.getY(),
            heading
        );

        telemetry.logLocalizationFusedPose(fusedPose);

        // ============================
        // 6. Feed fused pose into drivetrain
        // ============================

        if (!enabled) {
            // Disabled → PV fully controls pose
            if (pvGood) {
                swerve.seedPoseFromVision(pvPoseOpt.get());
            }
            return;
        }

        // Enabled → QuestNav always contributes
        if (questGood) {
            swerve.addVisionMeasurement(
                questPose,
                questTs,
                frc.robot.Constants.kQuest.STD_DEVS
            );
        }

        // PV only contributes when close and stable
        if (pvClose) {
            swerve.addVisionMeasurement(
                pvPoseOpt.get(),
                pv.getFusedTimestamp(),
                VecBuilder.fill(
                    pv.getFusedXyStd(),
                    pv.getFusedXyStd(),
                    pv.getFusedAngStd()
                )
            );
        }
    }

    // ============================
    // Public API
    // ============================

    public Pose2d getFusedPose() {
        return swerve.getPose();
    }

    public boolean isStartupSeeded() {
        return startupSeeded;
    }

    public void resetVisionSeed() {
        startupSeeded = false;
        pv.setSeeded(false);
    }

    public void resetHeadingToZero() {
        // 1. Get the current fused pose (this is what the robot "thinks" it is)
        Pose2d current = getFusedPose();

        // 2. Build a new pose with the same X/Y but heading = 0
        Pose2d zeroed = new Pose2d(
            current.getX(),
            current.getY(),
            new Rotation2d(0.0)
        );

        // 3. Reset the drivetrain pose estimator to this new pose
        swerve.seedPoseFromVision(zeroed);
    }
}
