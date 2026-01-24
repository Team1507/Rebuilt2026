// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

// Constants
import frc.robot.Constants.Intake.Gains;

public class IntakeSubsystem extends Subsystems1507 {
    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(TalonFX intakeMotor) {
        this.intakeMotor = intakeMotor;
        configureMotor();
    }

    private void configureMotor() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = Gains.Roller.KP;
        cfg.Slot0.kI = Gains.Roller.KI;
        cfg.Slot0.kD = Gains.Roller.KD;

        cfg.Slot0.kV = Gains.Roller.KV;
        cfg.Slot0.kS = Gains.Roller.KS;
        cfg.Slot0.kA = Gains.Roller.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));
        
        intakeMotor.getConfigurator().apply(cfg);
    }

    public void run(double dutyCycle) {
        SmartDashboard.putNumber("Intake/Roller Target Duty Cycle", dutyCycle);
        intakeMotor.setControl(dutyRequest.withOutput(dutyCycle));
    }

    public void setpower(double power) {
        intakeMotor.set(power);
    }    

    public double getDutyCycle(){
        return intakeMotor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake/Roller Duty Cycle", getDutyCycle());
    }
}