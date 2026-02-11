//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// CTRE Imports
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.robot.subsystems.lib.Subsystems1507;
// Utilities
import frc.robot.utilities.MotorConfig;

public class AgitatorSubsystem extends Subsystems1507 {
    private final TalonFXS agitatorMotor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
    
    /** Creates a new HopperSubsystem. */
    public AgitatorSubsystem(MotorConfig motor) {
        this.agitatorMotor = new TalonFXS(motor.CAN_ID());

        configureFXSMotor(motor, agitatorMotor);
    }
 
    public void run(double dutyCycle) {
        agitatorMotor.setControl(dutyRequest.withOutput(dutyCycle));
    }

    /** Stop motor */
    public void stop() {
        agitatorMotor.set(0);
    }

    public double getDutyCycle(){
        return agitatorMotor.getDutyCycle().getValueAsDouble();
    }
}
