//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakeroller;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.lib.hardware.IntakeRollerHardware;
import frc.lib.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of IntakeRollerIO using a TalonFX.
 */
public class IntakeRollerIOReal extends Subsystems1507 implements IntakeRollerIO {

    private final TalonFX motor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    public IntakeRollerIOReal(MotorConfig config) {
        this.motor = new TalonFX(IntakeRollerHardware.ROLLER_ID);
        configureFXMotor(config, motor);
    }

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        inputs.dutyCycle = motor.getDutyCycle().getValueAsDouble();
        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void runDuty(double duty) {
        motor.setControl(dutyRequest.withOutput(duty));
    }

    @Override
    public void runPower(double power) {
        motor.set(power);
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
