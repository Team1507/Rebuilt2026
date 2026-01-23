package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Feeder.Gains;
import static edu.wpi.first.units.Units.Hertz;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.Constants.Feeder.Gains;
import frc.robot.Constants.Shooter;
// Mechanics
import frc.robot.mechanics.GearRatio;

//lib
import frc.robot.subsystems.lib.Subsystems1507;


public class FeederSubsystem extends Subsystems1507 {

    private final TalonFXS feedermotor;
    private final GearRatio ratio; 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

    public FeederSubsystem(TalonFXS feedermotor, GearRatio ratio) {
        this.feedermotor = feedermotor;
        this.ratio = ratio;
        configureMotor();
    }

   // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------

    /**
     * Applies PID and feedforward gains from {@link Shooter.Gains}
     * to the TalonFX Slot0 configuration.
     */
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
    }

    public void setVelocityRPM(double rpm) {
        double motorRPM = ratio.toMotor(rpm);
        double motorRPS = motorRPM / 60.0;
        feedermotor.setControl(velocityRequest.withVelocity(motorRPS));
    }
    public double getVelocityRPM() {
        double motorRPS = feedermotor.getVelocity().getValueAsDouble();
        double motorRPM = motorRPS * 60.0;
        return ratio.toOutput(motorRPM);
    }
    public void setFeederVoltage(double voltage){
        feedermotor.setVoltage(voltage);
    }

     /** Run motor in open-loop (percent output) */
    public void runMotor(double power) {
        feedermotor.set(power);
    }

    /** Stop motor */
    public void stopMotor() {
        feedermotor.set(0);
    }
}
