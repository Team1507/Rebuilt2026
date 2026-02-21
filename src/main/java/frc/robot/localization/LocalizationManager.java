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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.localization.vision.PVManager;
import frc.robot.localization.vision.QuestNavManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.lib.logging.Telemetry;

/**
 * LocalizationManager:
 *
 *  - Fuses PhotonVision + QuestNav + Gyro + Odometry
 *  - Chooses best translation source (PV or QuestNav)
 *  - Chooses best heading source (QuestNav or Gyro)
 *  - Seeds SwerveSubsystem and QuestNav on startup
 *  - Feeds final fused pose into SwerveSubsystem
 *
 * This is the single source of truth for robot pose.
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

        boolean questGood = quest.isTracking();
        Pose2d questPose = quest.getPose2d();
        double questTs = quest.getTimestamp();

        Pose2d odomPose = swerve.getPose();

        // ============================
        // 2. Startup seeding (safe)
        // ============================

        if (!startupSeeded) {

            boolean pvStable = pvGood && pv.getFusedXyStd() < 1.0 && pv.getFusedAngStd() < 0.7;

            if (pvStable) {
                pvStableCount++;
            } else {
                pvStableCount = 0;
            }

            if (pvStableCount >= 5) {
                Pose2d seed = pvPoseOpt.get();

                // 1. Seed drivetrain pose
                swerve.seedPoseFromVision(seed);

                // 2. Seed QuestNav
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

        if (pvGood) {
            translationSource = pvPoseOpt.get();
            telemetry.logLocalizationTranslationSource("PhotonVision");
        } else if (questGood) {
            translationSource = questPose;
            telemetry.logLocalizationTranslationSource("QuestNav");
        } else {
            translationSource = odomPose;
            telemetry.logLocalizationTranslationSource("Odometry");
        }

        // ============================
        // 4. Choose heading source
        // ============================

        edu.wpi.first.math.geometry.Rotation2d heading;

        if (questGood) {
            heading = questPose.getRotation();
            telemetry.logLocalizationHeadingSource("QuestNav");
        } else {
            heading = swerve.getHeading();
            telemetry.logLocalizationHeadingSource("Gyro");
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

        if (pvGood) {
            swerve.addVisionMeasurement(
                fusedPose,
                pv.getFusedTimestamp(),
                VecBuilder.fill(
                    pv.getFusedXyStd(),
                    pv.getFusedXyStd(),
                    pv.getFusedAngStd()
                )
            );
        } else if (questGood) {
            swerve.addVisionMeasurement(
                fusedPose,
                questTs,
                frc.robot.Constants.kQuest.STD_DEVS
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
}
