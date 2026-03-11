// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.DashBoardStuff;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.atomic.AgitatorCommands;
import frc.robot.commands.atomic.ClimberCommands;
import frc.robot.commands.atomic.FeederCommands;
import frc.robot.commands.atomic.HopperCommands;
import frc.robot.commands.atomic.IntakeArmCommands;
import frc.robot.commands.atomic.IntakeRollerCommands;
import frc.robot.commands.atomic.ShooterCommands;
import frc.robot.framework.SubsystemsRecord;

/** Add your docs here. */
public class DashboardManagerManual {
    private final SubsystemsRecord subsystems;

    /* ---------------- NT Root ---------------- */
    private final NetworkTable nt =
        NetworkTableInstance.getDefault().getTable("Manual Mode");

    private final DoubleSubscriber subAgitatorDuty =
        nt.getDoubleTopic("Agitator/Target DC").subscribe(0.0);
    private final DoubleSubscriber subClimberPos =
        nt.getDoubleTopic("Climber/Target Position").subscribe(0.0);
    private final DoubleSubscriber subBLUFeederRPM =
        nt.getDoubleTopic("Feeder/BLU/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subYELFeederRPM =
        nt.getDoubleTopic("Feeder/YEL/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subHopperAngle =
        nt.getDoubleTopic("Hopper/Target Angle").subscribe(0.0);
    private final DoubleSubscriber subIntakeArmAngle =
        nt.getDoubleTopic("Intake/Arm/Target Angle").subscribe(0.0);
    private final DoubleSubscriber subIntakeRollerDuty =
        nt.getDoubleTopic("Intake/Roller/Target DC").subscribe(0.0);
    private final DoubleSubscriber subBLUShooterRPM =
        nt.getDoubleTopic("Shooter/BLU/Target RPM").subscribe(0.0);
    private final DoubleSubscriber subYELShooterRPM =
        nt.getDoubleTopic("Shooter/YEL/Target RPM").subscribe(0.0);

    /* ---------------- RUN Button Subscribers ---------------- */
    private final BooleanSubscriber subAgitatorRun =
        nt.getBooleanTopic("Agitator/Run").subscribe(false);
    private final BooleanSubscriber subClimberRun =
        nt.getBooleanTopic("Climber/Run").subscribe(false);
    private final BooleanSubscriber subBLUFeederRun =
        nt.getBooleanTopic("Feeder/BLU/Run").subscribe(false);
    private final BooleanSubscriber subYELFeederRun =
        nt.getBooleanTopic("Feeder/YEL/Run").subscribe(false);
    private final BooleanSubscriber subHopperRun =
        nt.getBooleanTopic("Hopper/Run").subscribe(false);
    private final BooleanSubscriber subIntakeArmRun =
        nt.getBooleanTopic("Intake/Arm/Run").subscribe(false);
    private final BooleanSubscriber subIntakeRollerRun =
        nt.getBooleanTopic("Intake/Roller/Run").subscribe(false);
    private final BooleanSubscriber subBLUShooterRun =
        nt.getBooleanTopic("Shooter/BLU/Run").subscribe(false);
    private final BooleanSubscriber subYELShooterRun =
        nt.getBooleanTopic("Shooter/YEL/Run").subscribe(false);

     /* ---------------- Rising-edge tracking ---------------- */
    private boolean prevAgitator, prevClimber, prevBLUFeeder, prevYELFeeder;
    private boolean prevHopper, prevIntakeArm, prevIntakeRoller;
    private boolean prevBLUShooter, prevYELShooter;

    public DashboardManagerManual(
        SubsystemsRecord subsystems
    ) {
        this.subsystems = subsystems;
    }
    public void updateInputs() {
          /* ---------------- Manual RUN Buttons ---------------- */
        boolean agitator = subAgitatorRun.get();
        boolean climberRun = subClimberRun.get();
        boolean bluFeeder = subBLUFeederRun.get();
        boolean yelFeeder = subYELFeederRun.get();
        boolean hopperRun = subHopperRun.get();
        boolean intakeArm = subIntakeArmRun.get();
        boolean intakeRoller = subIntakeRollerRun.get();
        boolean bluShooterRun = subBLUShooterRun.get();
        boolean yelShooterRun = subYELShooterRun.get();

         /* Rising edges */
        if (agitator && !prevAgitator)
            AgitatorCommands.manual(subsystems.agitator(), () -> subAgitatorDuty.get()).schedule();
        if (climberRun && !prevClimber)
            ClimberCommands.manual(subsystems.climber(), () -> subClimberPos.get()).schedule();
        if (bluFeeder && !prevBLUFeeder)
            FeederCommands.manual(subsystems.BLUfeeder(), () -> subBLUFeederRPM.get()).schedule();
        if (yelFeeder && !prevYELFeeder)
            FeederCommands.manual(subsystems.YELfeeder(), () -> subYELFeederRPM.get()).schedule();
        if (hopperRun && !prevHopper)
            HopperCommands.manualPosition(subsystems.hopper(), () -> subHopperAngle.get()).schedule();
        if (intakeArm && !prevIntakeArm)
            IntakeArmCommands.manualAngle(subsystems.intakeArm(), () -> subIntakeArmAngle.get()).schedule();
        if (intakeRoller && !prevIntakeRoller)
            IntakeRollerCommands.manual(subsystems.intakeRoller(), () -> subIntakeRollerDuty.get()).schedule();
        if (bluShooterRun && !prevBLUShooter)
            ShooterCommands.manual(subsystems.BLUshooter(), () -> subBLUShooterRPM.get()).schedule();
        if (yelShooterRun && !prevYELShooter)
            ShooterCommands.manual(subsystems.YELshooter(), () -> subYELShooterRPM.get()).schedule();

        /* Falling edges */
        if (!agitator && prevAgitator) subsystems.agitator().getCurrentCommand().cancel();
        if (!climberRun && prevClimber) subsystems.climber().getCurrentCommand().cancel();
        if (!bluFeeder && prevBLUFeeder) subsystems.BLUfeeder().getCurrentCommand().cancel();
        if (!yelFeeder && prevYELFeeder) subsystems.YELfeeder().getCurrentCommand().cancel();
        if (!hopperRun && prevHopper) subsystems.hopper().getCurrentCommand().cancel();
        if (!intakeArm && prevIntakeArm) subsystems.intakeArm().getCurrentCommand().cancel();
        if (!intakeRoller && prevIntakeRoller) subsystems.intakeRoller().getCurrentCommand().cancel();
        if (!bluShooterRun && prevBLUShooter) subsystems.BLUshooter().getCurrentCommand().cancel();
        if (!yelShooterRun && prevYELShooter) subsystems.YELshooter().getCurrentCommand().cancel();

         /* Update previous */
        prevAgitator = agitator;
        prevClimber = climberRun;
        prevBLUFeeder = bluFeeder;
        prevYELFeeder = yelFeeder;
        prevHopper = hopperRun;
        prevIntakeArm = intakeArm;
        prevIntakeRoller = intakeRoller;
        prevBLUShooter = bluShooterRun;
        prevYELShooter = yelShooterRun;
    

    }
}
