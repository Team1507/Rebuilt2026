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
import frc.lib.math.GearRatio;
import frc.lib.util.MotorConfig;
import frc.robot.Constants.kHopper;
import frc.robot.framework.base.Subsystems1507;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Real hardware implementation of HopperIO using a TalonFXS.
 */
public class HopperIOReal extends Subsystems1507 implements HopperIO {

    private final DigitalInput magSensor = new DigitalInput(0);
    private final TalonFXS motor;
    private final GearRatio ratio;

    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public HopperIOReal(MotorConfig config) {
        this.motor = new TalonFXS(HopperHardware.HOPPER_ID);
        this.ratio = HopperHardware.RATIO; // <-- use hopper ratio, not climber ratio

        configureFXSMotor(motor, config);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        // Raw sensor units from TalonFXS
        inputs.motorRot = motor.getPosition().getValueAsDouble();

        // Convert sensor units → real-world inches using scaling
        inputs.position = ratio.sensorToReal(inputs.motorRot);

        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();

        // Hopper is extended if real-world inches exceed safe threshold
        inputs.hopperExtended = inputs.position > kHopper.SAFE_EXTENDED;
    }

    @Override
    public void setPositionDeg(double degrees) {
        // Hopper is linear, not rotational — convert inches instead
        double targetInches = degrees; // if commands still use degrees, rename later
        double sensorUnits = ratio.realToSensor(targetInches);

        motor.setControl(positionRequest.withPosition(sensorUnits));
    }

    @Override
    public boolean getMagSensor() {
        return !magSensor.get();
    }

    @Override
    public void runPower(double power) {
        motor.set(power);
    }

    @Override
    public void hopperStop() {
        if (getMagSensor()) {
            motor.set(0);
        }
    }
}
