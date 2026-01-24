// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.Constants.Intake.Gains;
import frc.robot.subsystems.lib.Subsystems1507;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeArmSubsystem extends Subsystems1507 {
 private final TalonFX intakeArmMotor;
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);
  /** Creates a new IntakeSubsystem. */
  public IntakeArmSubsystem(TalonFX motor) {
    this.intakeArmMotor = motor;
    configureMotor();
  }

  private void configureMotor() {
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        

        cfg.Slot0.kP = Gains.KP;
        cfg.Slot0.kI = Gains.KI;
        cfg.Slot0.kD = Gains.KD;

        cfg.Slot0.kV = Gains.KV;
        cfg.Slot0.kS = Gains.KS;
        cfg.Slot0.kA = Gains.KA;

         // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                      .withPeakReverseVoltage(Volts.of(-8));
    }
    public void setPosition(double degrees){
      intakeArmMotor.setControl(positionRequest.withPosition(degrees/360.0));
    }

    public double getPosition(){
      return  intakeArmMotor.getPosition().getValueAsDouble();
    }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Arm Position", getPosition());
  }

}
