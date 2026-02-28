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

import frc.lib.hardware.ClimberHardware;
import frc.lib.hardware.HopperHardware;
import frc.lib.math.GearRatio;
import frc.lib.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Real hardware implementation of HopperIO using a TalonFXS.
 */
public class HopperIOReal extends Subsystems1507 implements HopperIO {

    private final  DigitalInput magSensor = new DigitalInput(0);
    private final TalonFXS motor;
    private final GearRatio ratio;
    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public HopperIOReal(MotorConfig config) {
        this.motor = new TalonFXS(HopperHardware.HOPPER_ID);
        this.ratio = ClimberHardware.RATIO;
        configureFXSMotor(config, motor);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.motorRot = motor.getPosition().getValueAsDouble();
        inputs.positionDeg = inputs.motorRot * 360.0;
        inputs.position = ratio.toOutput(inputs.motorRot);

        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setPositionDeg(double degrees) {
        double motorRot = ratio.toMotor(degrees);
        motor.setControl(positionRequest.withPosition(motorRot));
    }

     @Override
    public boolean getMagSensor() {
        return !magSensor.get(); 
    }

    @Override
    public void hopperStop() {
        if(getMagSensor()){
        motor.set(0);
    }
}
}
