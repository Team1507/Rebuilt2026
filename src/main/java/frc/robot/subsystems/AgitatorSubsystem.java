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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;
import frc.robot.utilities.MotorConfig;

public class AgitatorSubsystem extends Subsystems1507 {

  private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
  
  private final TalonFXS agitatorMotor;
  /** Creates a new HopperSubsystem. */
  public AgitatorSubsystem(MotorConfig motor) {
    this.agitatorMotor = new TalonFXS(motor.CAN_ID());

    configureFXSMotor(motor, agitatorMotor);
    
  }
 

  public void setVelocityRPM(double rpm) {
    double agitatorRPS = rpm / 60.0;
    agitatorMotor.setControl(velocityRequest.withVelocity(agitatorRPS));
  }
  private double getTargetRPM() {
    double agitatorRPS = agitatorMotor.getVelocity().getValueAsDouble();
    double agitatorRPM = agitatorRPS * 60.0;
    return agitatorRPM;
  }

   /** Stop motor */
    public void stopMotor() {
        agitatorMotor.set(0);
  }
}