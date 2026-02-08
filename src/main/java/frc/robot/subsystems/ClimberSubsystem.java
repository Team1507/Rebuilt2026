//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.servohub.config.ServoChannelConfig;
import edu.wpi.first.wpilibj.Servo;

import static edu.wpi.first.units.Units.Volts;
import frc.robot.mechanics.GearRatio;
import frc.robot.utilities.MotorConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.lib.Subsystems1507;

public class ClimberSubsystem extends Subsystems1507 {
  private final TalonFX climberMotor;
  private Servo ratchetLock;
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);
  private GearRatio ratio;
  private double targetPosition;

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(MotorConfig motor, int SERVO_PORT) {
    this.climberMotor = new TalonFX(motor.CAN_ID());
    this.ratchetLock = new Servo(SERVO_PORT);
    this.ratio = motor.ratio();
    configureFXMotor(motor, climberMotor);
  }


  public void setPosition(double position){
    // targetPosition updates each time setPosition is called
    targetPosition = position;
    double motorPos = ratio.toMotor(position);
    climberMotor.setControl(positionRequest.withPosition(motorPos));
  }
  public void setServo (double position){
   ratchetLock.set(position);
  }
  public boolean isAtPosition(){
    double outputPos = getPosition();
    double outputTolerance = 1.0;
    return(outputPos >= (targetPosition - outputTolerance) && outputPos <= (targetPosition + outputTolerance));
  }

   public double getPosition() {
    double motorPos = climberMotor.getPosition().getValueAsDouble();
    double outputPos = ratio.toOutput(motorPos);
    return outputPos;
   }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getPosition());
  }
  
}
