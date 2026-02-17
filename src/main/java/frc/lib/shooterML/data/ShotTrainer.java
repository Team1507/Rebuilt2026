package frc.lib.shooterML.data;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShotTrainer collects unlabeled shot data for offline model training.
 *
 * <p>This version is IO‑friendly and does not depend on TalonFX hardware.
 * All shooter telemetry is pulled from ShooterSubsystem, which abstracts
 * real vs simulated hardware.
 */
public class ShotTrainer {

    private final ShooterSubsystem shooter;
    private final Supplier<Pose2d> poseSupplier;
    private Translation2d targetPose;

    private int nextShotId = 0;

    private final List<ShotRecord> pendingShots = new ArrayList<>();

    private final NetworkTable rootTable =
        NetworkTableInstance.getDefault()
            .getTable("Shooter")
            .getSubTable("UnlabeledShots");

    public ShotTrainer(
        ShooterSubsystem shooter,
        Supplier<Pose2d> poseSupplier,
        Translation2d targetPose
    ) {
        this.shooter = shooter;
        this.poseSupplier = poseSupplier;
        this.targetPose = targetPose;
    }

    public void notifyShotFired(double timestampSeconds) {
        recordShot(timestampSeconds);
    }

    private void recordShot(double timestampSeconds) {

        Pose2d pose = poseSupplier.get();
        double distance = pose.getTranslation().getDistance(targetPose);

        ShotRecord record = new ShotRecord(
            shooter.getShooterRPM(),
            shooter.getShooterVoltage(),
            shooter.getStatorCurrent(),
            shooter.getSupplyCurrent(),
            shooter.getClosedLoopError(),
            pose,
            distance
        );

        record.id = nextShotId++;
        pendingShots.add(record);

        publishShot(record);
    }

    private void publishShot(ShotRecord r) {

        NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));

        t.getEntry("id").setDouble(r.id);
        t.getEntry("distance").setDouble(r.distanceToTarget);
        t.getEntry("rpm").setDouble(r.shooterRPM);
        t.getEntry("stator").setDouble(r.statorCurrent);
        t.getEntry("supply").setDouble(r.supplyCurrent);
        t.getEntry("error").setDouble(r.closedLoopError);
    }

    public void update() {
        checkForLabels();
    }

    private void checkForLabels() {

        Iterator<ShotRecord> it = pendingShots.iterator();

        while (it.hasNext()) {
            ShotRecord r = it.next();

            NetworkTable t = rootTable.getSubTable(Integer.toString(r.id));
            String label = t.getEntry("label").getString("unlabeled");

            if (label == null || label.isBlank() ||
                label.equals("unlabeled") || label.equals("processed")) {
                continue;
            }

            ShotLabel parsed = ShotLabelParser.parse(label);
            r.setLabel(parsed.made());

            CSVWriter.writeSingleRecord(r);

            t.getEntry("label").setString("processed");

            for (String key : t.getKeys()) {
                t.getEntry(key).unpublish();
            }

            rootTable.getEntry(Integer.toString(r.id)).unpublish();

            it.remove();
        }
    }
}
