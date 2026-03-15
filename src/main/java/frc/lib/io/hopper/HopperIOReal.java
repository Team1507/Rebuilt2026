//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.hopper;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.Constants.kHopper;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of HopperIO using a TalonFXS.
 */
public class HopperIOReal extends Subsystems1507 implements HopperIO {

    private final TalonFXS motor;
    private final GearRatio ratio;
    private final DigitalInput halSensor;

    // Cached CTRE signals
    private final StatusSignal<Angle> positionSig;
    private final StatusSignal<Current> currentSig;
    private final StatusSignal<Temperature> tempSig;

    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public HopperIOReal(int canID, MotorConfig config, GearRatio ratio, int sensorDIO) {
        this.motor = new TalonFXS(canID);
        this.ratio = ratio;
        this.halSensor = new DigitalInput(sensorDIO);

        configureFXSMotor(motor, config);

        // Grab signals once
        positionSig = motor.getPosition();
        currentSig = motor.getStatorCurrent();
        tempSig = motor.getDeviceTemp();

        // Set CAN update rate (20–50 Hz is ideal)
        BaseStatusSignal.setUpdateFrequencyForAll(
            75,
            positionSig, currentSig, tempSig
        );
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        // Bulk refresh (1 CAN transaction instead of 3)
        BaseStatusSignal.refreshAll(positionSig, currentSig, tempSig);

        // Read cached values
        double motorRot = positionSig.getValueAsDouble();
        inputs.motorRot = motorRot;

        // Convert sensor → real-world inches
        inputs.position = ratio.sensorToReal(motorRot);

        inputs.currentA = currentSig.getValueAsDouble();
        inputs.temperatureC = tempSig.getValueAsDouble();

        // HAL sensor (reverse limit)
        inputs.reverseLimit = !halSensor.get();

        // Auto-zero when hitting the back limit
        if (inputs.reverseLimit) {
            motor.setPosition(0);
        }

        // Hopper extended check
        inputs.hopperExtended = inputs.position > kHopper.SAFE_EXTENDED;
    }

    @Override
    public void setPosition(double position) {
        double safePosition = position;

        // Prevent commanding negative motion into the limit switch
        if (!halSensor.get() && position < 0) {
            safePosition = 0.0;
        }

        double sensorUnits = ratio.realToSensor(safePosition);
        motor.setControl(positionRequest.withPosition(sensorUnits));
    }

    @Override
    public boolean getMagSensor() {
        return !halSensor.get();
    }

    @Override
    public void runPower(double power) {
        boolean atRev = !halSensor.get();
        double safePower = power;

        // Prevent reverse motion into the limit switch
        if (atRev && power < 0) {
            safePower = 0;
        }

        motor.set(safePower);
    }

    @Override
    public void hopperStop() {
        if (getMagSensor()) {
            motor.set(0);
        }
    }
}
