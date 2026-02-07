//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kClimber.kGains;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.config.ServoChannelConfig;

import static edu.wpi.first.units.Units.Volts;
import frc.robot.mechanics.GearRatio;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;
  private final ServoHub ratchetLock;
  private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);
  private GearRatio ratio = GearRatio.gearBox(64,1);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(TalonFX climberMotor, ServoHub ratchetLock) {
    this.climberMotor = climberMotor;
    this.ratchetLock = ratchetLock;
    configureMotor();
  }

  private void configureMotor() {
      TalonFXConfiguration cfg = new TalonFXConfiguration();
      ServoHubConfig config = new ServoHubConfig();
      config.channel0.pulseRange(500, 1500, 2500).disableBehavior(ServoChannelConfig.BehaviorWhenDisabled.kSupplyPower);

        cfg.Slot0.kP = kGains.KP;
        cfg.Slot0.kI = kGains.KI;
        cfg.Slot0.kD = kGains.KD;

        cfg.Slot0.kV = kGains.KV;
        cfg.Slot0.kS = kGains.KS;
        cfg.Slot0.kA = kGains.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));
        
        climberMotor.getConfigurator().apply(cfg);
        ratchetLock.configure(config, ServoHub.ResetMode.kNoResetSafeParameters);
  }

  public void setPosition(double position){
    double motorPos = ratio.toMotor(position);
    climberMotor.setControl(positionRequest.withPosition(motorPos));
  }
  public void setservo (double position){
   // ratchetLock.set(position);
  }

   public double getPosition() {
    double motorRot = climberMotor.getPosition().getValueAsDouble();
    double outputRot = ratio.toOutput(motorRot);
    return outputRot;
   }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Position", getPosition());
  }
  
}
