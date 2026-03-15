//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakeroller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.lib.core.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of IntakeRollerIO using a TalonFX.
 */
public class IntakeRollerIOReal extends Subsystems1507 implements IntakeRollerIO {

    private final TalonFX motor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    // Cached CTRE signals
    private final StatusSignal<Double> dutySig;
    private final StatusSignal<Current> currentSig;
    private final StatusSignal<Temperature> tempSig;

    public IntakeRollerIOReal(int canID, MotorConfig config) {
        this.motor = new TalonFX(canID);
        configureFXMotor(motor, config);

        // Grab signals once
        dutySig = motor.getDutyCycle();
        currentSig = motor.getStatorCurrent();
        tempSig = motor.getDeviceTemp();

        // Set CAN update rate (20–50 Hz is ideal)
        BaseStatusSignal.setUpdateFrequencyForAll(
            75,
            dutySig, currentSig, tempSig
        );
    }

    @Override
    public void updateInputs(IntakeRollerInputs inputs) {
        // Bulk refresh (1 CAN transaction instead of 3)
        BaseStatusSignal.refreshAll(dutySig, currentSig, tempSig);

        // Read cached values
        inputs.dutyCycle = dutySig.getValueAsDouble();
        inputs.currentA = currentSig.getValueAsDouble();
        inputs.temperatureC = tempSig.getValueAsDouble();
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
