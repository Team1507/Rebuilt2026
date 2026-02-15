//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// CTRE Libraries
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.framework.base.Subsystems1507;
// Mechanics
import frc.robot.mechanics.GearRatio;
// Utilities
import frc.robot.utilities.MotorConfig;


public class FeederSubsystem extends Subsystems1507 {

    private final TalonFX feedermotor;
    private final GearRatio ratio; 
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);


    public FeederSubsystem(MotorConfig config) {
        this.feedermotor = new TalonFX(config.CAN_ID());
        this.ratio = config.ratio();
        configureFXMotor(config, feedermotor);
    }

   // ------------------------------------------------------------
    // PID Configuration
    // ------------------------------------------------------------

   
    public void run(double rpm) {
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
