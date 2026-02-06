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
import static edu.wpi.first.units.Units.Volts;
import frc.robot.mechanics.GearRatio;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX climberMotor;

  private GearRatio ratio = GearRatio.gearBox(1,2);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem(TalonFX climberMotor) {
    this.climberMotor = climberMotor;
    configureMotor();
  }

  private void configureMotor() {
      TalonFXConfiguration cfg = new TalonFXConfiguration();

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
  }

  public void setPostition(double degrees){
    double outputRot = degrees / 360.0;
    double motorRot = ratio.toMotor(outputRot);
  }

   public double getPostitionDegrees() {
    double motorRot = climberMotor.getPosition().getValueAsDouble();
    double outputRot = ratio.toOutput(motorRot);
    return outputRot * 360.0;
   }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Climber Angle", getPostitionDegrees());
  }
}
