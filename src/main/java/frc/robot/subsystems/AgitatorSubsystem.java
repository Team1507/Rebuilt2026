// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import frc.robot.Constants.Agitator.Gains;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

public class AgitatorSubsystem extends Subsystems1507 {

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  
  private final TalonFXS agitatorMotor;
  /** Creates a new HopperSubsystem. */
  public AgitatorSubsystem(TalonFXS motor) {

    //declare dependencies
    this.agitatorMotor = motor;
    configureMotor();
    
  }
  private void configureMotor() {
            
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();
        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfg.Slot0.kP = Gains.KP;
        cfg.Slot0.kI = Gains.KI;
        cfg.Slot0.kD = Gains.KD;

        cfg.Slot0.kV = Gains.KV;
        cfg.Slot0.kS = Gains.KS;
        cfg.Slot0.kA = Gains.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        agitatorMotor.getConfigurator().apply(cfg);
    }

  public void setVelocityRPM(double rpm) {
    double agitatorRPS = rpm / 60.0;
    agitatorMotor.setControl(velocityRequest.withVelocity(agitatorRPS));
  }
  private double getTargetRPM(){
    double agitatorRPS = agitatorMotor.getVelocity().getValueAsDouble();
    double agitatorRPM = agitatorRPS * 60.0;
    return agitatorRPM;
  }
  
}