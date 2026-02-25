//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.framework.base;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MotorConfig;
import frc.lib.util.MotorConfig.ControlMode;
import frc.lib.util.MotorConfig.GravityType;;

public abstract class Subsystems1507 extends SubsystemBase {
   
    protected void configureFXMotor(MotorConfig config, TalonFX motor) {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        if (config.mode() != ControlMode.DUTY_CYCLE) {
            cfg.Slot0.kP = config.kP();
            cfg.Slot0.kI = config.kI();
            cfg.Slot0.kD = config.kD();

            cfg.Slot0.kV = config.kV();
            cfg.Slot0.kS = config.kS();
            cfg.Slot0.kA = config.kA();
        }

        cfg.Voltage.withPeakForwardVoltage(Volts.of(config.peakForwardVoltage()))
                .withPeakReverseVoltage(Volts.of(config.peakReverseVoltage()));

        safeApply(motor, cfg);
    }

    protected void configureFXSMotor(MotorConfig config, TalonFXS motor) {
        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.ExternalFeedback.RotorToSensorRatio = 1.0;
        cfg.ExternalFeedback.SensorToMechanismRatio = 1.0;

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        if (config.mode() != ControlMode.DUTY_CYCLE) {
            cfg.Slot0.kP = config.kP();
            cfg.Slot0.kI = config.kI();
            cfg.Slot0.kD = config.kD();

            cfg.Slot0.kV = config.kV();
            cfg.Slot0.kS = config.kS();
            cfg.Slot0.kA = config.kA();
        }

        if (config.gravityType() == GravityType.COSINE) {
            cfg.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
            cfg.Slot0.kG = config.kG();
        }

        if (config.motorInverted()) {
            cfg.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        } else {
            cfg.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        cfg.Voltage.withPeakForwardVoltage(Volts.of(config.peakForwardVoltage()))
                .withPeakReverseVoltage(Volts.of(config.peakReverseVoltage()));

        safeApply(motor, cfg);
    }

    // -------------------------
    // Safe Apply Overloads
    // -------------------------

    protected void safeApply(TalonFX motor, TalonFXConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FX " + motor.getDeviceID() +
                            ": " + status.toString());
        }
    }

    protected void safeApply(TalonFXS motor, TalonFXSConfiguration cfg) {
        var status = motor.getConfigurator().apply(cfg);
        if (!status.isOK()) {
            System.out.println("[WARN] Failed to apply config to FXS " + motor.getDeviceID() +
                            ": " + status.toString());
        }
    }

}