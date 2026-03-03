//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.framework.base;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.MotorConfig;
import frc.lib.util.MotorConfig.ControlMode;
import frc.lib.util.MotorConfig.GravityType;

public abstract class Subsystems1507 extends SubsystemBase {

    // ============================================================
    // FX (Kraken) CONFIGURATION
    // ============================================================
    protected void configureFXMotor(TalonFX motor, MotorConfig... configs) {
        if (configs[0].slotNumber() != 0) {
            throw new IllegalArgumentException("First MotorConfig must be slot 0");
        }

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        cfg.MotorOutput.Inverted = configs[0].motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.withPeakForwardVoltage(Volts.of(configs[0].peakForwardVoltage()))
                .withPeakReverseVoltage(Volts.of(configs[0].peakReverseVoltage()));

        safeApply(motor, cfg);
    }

    // ============================================================
    // FXS (Minion) CONFIGURATION
    // ============================================================
    protected void configureFXSMotor(TalonFXS motor, MotorConfig... configs) {
        if (configs[0].slotNumber() != 0) {
            throw new IllegalArgumentException("First MotorConfig must be slot 0");
        }

        TalonFXSConfiguration cfg = new TalonFXSConfiguration();

        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        for (MotorConfig config : configs) {
            applySlot(cfg, config);
        }

        cfg.MotorOutput.Inverted = configs[0].motorInverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        cfg.Voltage.withPeakForwardVoltage(Volts.of(configs[0].peakForwardVoltage()))
                .withPeakReverseVoltage(Volts.of(configs[0].peakReverseVoltage()));

        safeApply(motor, cfg);
    }

    // ============================================================
    // APPLY CONFIG TO THE CORRECT SLOT (0, 1, OR 2)
    // ============================================================
    private void applySlot(TalonFXConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    private void applySlot(TalonFXSConfiguration cfg, MotorConfig config) {
        switch (config.slotNumber()) {
            case 1 -> applyToSlot(cfg.Slot1, config);
            case 2 -> applyToSlot(cfg.Slot2, config);
            default -> applyToSlot(cfg.Slot0, config);
        }
    }

    // ============================================================
    // APPLY PID / FF / GRAVITY TO EACH SLOT TYPE
    // (Duplicated because CTRE made them unrelated classes)
    // ============================================================

    private void applyToSlot(Slot0Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    private void applyToSlot(Slot1Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    private void applyToSlot(Slot2Configs slot, MotorConfig config) {
        if (config.mode() != ControlMode.DUTY_CYCLE) {
            slot.kP = config.kP();
            slot.kI = config.kI();
            slot.kD = config.kD();

            slot.kV = config.kV();
            slot.kS = config.kS();
            slot.kA = config.kA();
        }

        if (config.gravityType() != GravityType.NONE) {
            switch (config.gravityType()) {
                case COSINE -> slot.GravityType = GravityTypeValue.Arm_Cosine;
                case CONSTANT -> slot.GravityType = GravityTypeValue.Elevator_Static;
                default -> {}
            }
            slot.kG = config.kG();
        }
    }

    // ============================================================
    // SAFE APPLY
    // ============================================================
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
