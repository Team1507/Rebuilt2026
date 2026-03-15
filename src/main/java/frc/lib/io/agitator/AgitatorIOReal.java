//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.agitator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.lib.core.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of AgitatorIO using a TalonFXS.
 */
public class AgitatorIOReal extends Subsystems1507 implements AgitatorIO {

    private final TalonFX motor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    // Explicit types — this is what Java requires
    private final StatusSignal<Double> dutySig;
    private final StatusSignal<Temperature> tempSig;
    private final StatusSignal<Current> currentSig;

    public AgitatorIOReal(int canID, MotorConfig config) {
        this.motor = new TalonFX(canID);
        configureFXMotor(motor, config);

        // Grab signals once
        dutySig = motor.getDutyCycle();
        tempSig = motor.getDeviceTemp();
        currentSig = motor.getStatorCurrent();

        // Set CAN update rate (20–50 Hz is plenty)
        BaseStatusSignal.setUpdateFrequencyForAll(
            75,
            dutySig, tempSig, currentSig
        );
    }

    @Override
    public void updateInputs(AgitatorInputs inputs) {
        // Bulk refresh (1 CAN transaction instead of 3)
        BaseStatusSignal.refreshAll(dutySig, tempSig, currentSig);

        // Read cached values
        inputs.dutyCycle = dutySig.getValueAsDouble();
        inputs.temperatureC = tempSig.getValueAsDouble();
        inputs.currentA = currentSig.getValueAsDouble();
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
