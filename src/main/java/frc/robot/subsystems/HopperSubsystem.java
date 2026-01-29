// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import frc.robot.Constants.Intake.Gains;
// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

public class HopperSubsystem extends Subsystems1507 {

  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

  private final TalonFXS hopperMotor = new TalonFXS(29);
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {
    
    //declare dependencies
    configureMotor();
    
  }
  private void configureMotor() {
            
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        cfg.Slot0.kP = Gains.Arm.KP;
        cfg.Slot0.kI = Gains.Arm.KI;
        cfg.Slot0.kD = Gains.Arm.KD;

        cfg.Slot0.kV = Gains.Arm.KV;
        cfg.Slot0.kS = Gains.Arm.KS;
        cfg.Slot0.kA = Gains.Arm.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        hopperMotor.getConfigurator().apply(config);
    }

    public void setPosition(double degrees){
        double outputRot = degrees / 360.0;
        //double motorRot = ratio.toMotor(outputRot);

        hopperMotor.setControl(positionRequest.withPosition(outputRot));
    }

    public double getPositionDegrees() {
        double motorRot = hopperMotor.getPosition().getValueAsDouble();
        //double outputRot = ratio.toOutput(motorRot);
        return motorRot * 360.0;
    }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
