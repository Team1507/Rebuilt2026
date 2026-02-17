//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.agitator;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.lib.hardware.AgitatorHardware;
import frc.lib.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of AgitatorIO using a TalonFXS.
 */
public class AgitatorIOReal extends Subsystems1507 implements AgitatorIO {

    private final TalonFXS motor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public AgitatorIOReal(MotorConfig config) {
        this.motor = new TalonFXS(AgitatorHardware.AGITATOR_ID);
        configureFXSMotor(config, motor);
    }

    @Override
    public void updateInputs(AgitatorInputs inputs) {
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void run(double dutyCycle) {
        motor.setControl(dutyRequest.withOutput(dutyCycle));
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
