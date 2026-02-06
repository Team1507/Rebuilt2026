//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// CTRE Libraries
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import static edu.wpi.first.units.Units.Volts;

import frc.robot.Constants.kFeeder.BLU;
import frc.robot.Constants.kFeeder.YEL;
import frc.robot.Constants.kShooter;

// Mechanics
import frc.robot.mechanics.GearRatio;

//lib
import frc.robot.subsystems.lib.Subsystems1507;


public class FeederSubsystem extends Subsystems1507 {

    private final TalonFX feedermotor;
    private final GearRatio ratio; 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);


    public FeederSubsystem(TalonFX feedermotor, GearRatio ratio, String color) {
        this.feedermotor = feedermotor;
        this.ratio = ratio;
        if(color == "BLU")
        {
            configureBlueMotor();
        }

        if(color == "YEL")
        {
            configureYellowMotor();
        }
        
    }

   // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------

    /**
     * Applies PID and feedforward gains from {@link kShooter.kGains}
     * to the TalonFX Slot0 configuration.
     */
    private void configureBlueMotor() {
        
         TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = BLU.kGains.KP;
        cfg.Slot0.kI = BLU.kGains.KI;
        cfg.Slot0.kD = BLU.kGains.KD;

        cfg.Slot0.kV = BLU.kGains.KV;
        cfg.Slot0.kS = BLU.kGains.KS;
        cfg.Slot0.kA = BLU.kGains.KA;

         // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                      .withPeakReverseVoltage(Volts.of(-8));
    }

    private void configureYellowMotor() {
        
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = YEL.kGains.KP;
        cfg.Slot0.kI = YEL.kGains.KI;
        cfg.Slot0.kD = YEL.kGains.KD;

        cfg.Slot0.kV = YEL.kGains.KV;
        cfg.Slot0.kS = YEL.kGains.KS;
        cfg.Slot0.kA = YEL.kGains.KA;

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
