//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import frc.robot.Constants.kHopper.kGains;
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
            

        TalonFXSConfiguration cfg = new TalonFXSConfiguration();
        cfg.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        cfg.Slot0.kP = kGains.KP;
        cfg.Slot0.kI = kGains.KI;
        cfg.Slot0.kD = kGains.KD;

        cfg.Slot0.kV = kGains.KV;
        cfg.Slot0.kS = kGains.KS;
        cfg.Slot0.kA = kGains.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        hopperMotor.getConfigurator().apply(cfg);
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
