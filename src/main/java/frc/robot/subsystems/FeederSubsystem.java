package frc.robot.subsystems;
import static frc.robot.Constants.Feeder.Gains;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.VelocityVoltage;

// Mechanics
import frc.robot.mechanics.GearRatio;

//lib
import frc.robot.subsystems.lib.Subsystems1507;


public class FeederSubsystem extends Subsystems1507 {
    private final TalonFX feedermotor;
    private final GearRatio ratio; 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    public FeederSubsystem(TalonFX feedermotor, GearRatio ratio) {
        this.feedermotor = feedermotor;
        this.ratio = ratio;
        configurePID();
    }

   // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------

    /**
     * Applies PID and feedforward gains from {@link Shooter.Gains}
     * to the TalonFX Slot0 configuration.
     */
    private void configurePID() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = Gains.KP;
        cfg.Slot0.kI = Gains.KI;
        cfg.Slot0.kD = Gains.KD;

        cfg.Slot0.kV = Gains.KV;
        cfg.Slot0.kS = Gains.KS;
        cfg.Slot0.kA = Gains.KA;

        feedermotor.getConfigurator().apply(cfg);
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
    
}
