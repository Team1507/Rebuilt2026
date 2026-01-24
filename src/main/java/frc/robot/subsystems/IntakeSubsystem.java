// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.Gains;
import frc.robot.mechanics.GearRatio;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.subsystems.lib.Subsystems1507;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Intake.Gains;
import com.ctre.phoenix6.controls.DutyCycleOut;
public class IntakeSubsystem extends Subsystems1507 {
  private final TalonFX intakeMotor;
  private final GearRatio ratio; 
  private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
  private double lastDutyCycle = 0.0;
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem(TalonFX intakeMotor, GearRatio ratio) {
    this.ratio = ratio;
    this.intakeMotor = intakeMotor;
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

  public void run(double dutyCycle) {
    lastDutyCycle = dutyCycle;
    intakeMotor.setControl(dutyRequest.withOutput(dutyCycle));

  } 

  public void setpower(double power) {
    intakeMotor.set(power);

  }                              

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}