// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

// Mechanics
import frc.robot.mechanics.GearRatio;

// Constants
import frc.robot.Constants.Intake.Gains;

/**
 * Intake Arm Subsystem
 */
public class IntakeArmSubsystem extends Subsystems1507 {
    
    private final TalonFX intakeArmMotor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

    private GearRatio ratio = GearRatio.gearBox(1, 2);

    /** Creates a new IntakeSubsystem. */
    public IntakeArmSubsystem(TalonFX motor) {
        this.intakeArmMotor = motor;
        configureMotor();
    }

    private void configureMotor() {
            
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = Gains.Arm.KP;
        cfg.Slot0.kI = Gains.Arm.KI;
        cfg.Slot0.kD = Gains.Arm.KD;

        cfg.Slot0.kV = Gains.Arm.KV;
        cfg.Slot0.kS = Gains.Arm.KS;
        cfg.Slot0.kA = Gains.Arm.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        intakeArmMotor.getConfigurator().apply(cfg);
    }

    public void setPosition(double degrees){
        double outputRot = degrees / 360.0;
        double motorRot = ratio.toMotor(outputRot);

        intakeArmMotor.setControl(positionRequest.withPosition(motorRot));
    }

    public double getPositionDegrees() {
        double motorRot = intakeArmMotor.getPosition().getValueAsDouble();
        double outputRot = ratio.toOutput(motorRot);
        return outputRot * 360.0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Arm Angle", getPositionDegrees());
    }

}
