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
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;



// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;
import frc.robot.utilities.MotorConfig;

public class HopperSubsystem extends Subsystems1507 {

  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

  private final TalonFXS hopperMotor;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem(MotorConfig motor) {
    this.hopperMotor = new TalonFXS(motor.CAN_ID());
    //declare dependencies
    configureFXSMotor(motor, hopperMotor);
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
