//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;

import frc.lib.hardware.HopperHardware;
import frc.lib.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of HopperIO using a TalonFXS.
 */
public class HopperIOReal extends Subsystems1507 implements HopperIO {

    private final TalonFXS motor;
    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public HopperIOReal(MotorConfig config) {
        this.motor = new TalonFXS(HopperHardware.HOPPER_ID);
        configureFXSMotor(config, motor);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.motorRot = motor.getPosition().getValueAsDouble();
        inputs.positionDeg = inputs.motorRot * 360.0;

        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setPositionDeg(double degrees) {
        double motorRot = degrees / 360.0;
        motor.setControl(positionRequest.withPosition(motorRot));
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
