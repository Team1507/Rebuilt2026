// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

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
    
    private final TalonFXS intakeLeftArmMotor;
    private final TalonFXS intakeRightArmMotor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

    private GearRatio ratio = GearRatio.gearBox(1, 2);

    /** Creates a new IntakeSubsystem. */
    public IntakeArmSubsystem(TalonFXS motorLeft,TalonFXS motorRight) {
        this.intakeLeftArmMotor = motorLeft;
        configureMotor();
        this.intakeRightArmMotor = motorRight;
        configureMotor();
    }

    private void configureMotor() {
            
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();
        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfg.Slot0.kP = Gains.Arm.KP;
        cfg.Slot0.kI = Gains.Arm.KI;
        cfg.Slot0.kD = Gains.Arm.KD;

        cfg.Slot0.kV = Gains.Arm.KV;
        cfg.Slot0.kS = Gains.Arm.KS;
        cfg.Slot0.kA = Gains.Arm.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        intakeLeftArmMotor.getConfigurator().apply(cfg);
        intakeRightArmMotor.getConfigurator().apply(cfg);
    }

    public void setPosition(double degrees){
        double leftOutputRot = degrees / 360.0;
        double rightOutputRot = -degrees / 360.0;
        double leftMotorRot = ratio.toMotor(leftOutputRot);
        double rightMotorRot = -ratio.toMotor(rightOutputRot);

        intakeLeftArmMotor.setControl(positionRequest.withPosition(leftMotorRot));
        intakeRightArmMotor.setControl(positionRequest.withPosition(rightMotorRot));
    }

    public double getPositionDegrees() {
        double leftMotorRot = intakeLeftArmMotor.getPosition().getValueAsDouble();
        //double rightMotorRot = -intakeRightArmMotor.getPosition().getValueAsDouble();
        double leftOutputRot = ratio.toOutput(leftMotorRot);
        //double rightOutputRot = -ratio.toOutput(rightMotorRot);

        return leftOutputRot;
        //cant figure out how to return both positions.
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Arm Angle", getPositionDegrees());
    }

}
