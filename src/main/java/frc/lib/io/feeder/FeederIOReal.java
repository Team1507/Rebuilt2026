//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.lib.hardware.FeederHardware;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of FeederIO using a TalonFX.
 */
public class FeederIOReal extends Subsystems1507 implements FeederIO {

    private final TalonFX motor;
    private final GearRatio ratio = FeederHardware.RATIO;

    // Cached CTRE signals
    private final StatusSignal<AngularVelocity> velocitySig;
    private final StatusSignal<Current> currentSig;
    private final StatusSignal<Temperature> tempSig;

    private final VelocityVoltage velocityRequest =
        new VelocityVoltage(0).withSlot(0);

    public FeederIOReal(int canID, MotorConfig config) {
        this.motor = new TalonFX(canID);
        configureFXMotor(motor, config);

        // Grab signals once
        velocitySig = motor.getVelocity();
        currentSig = motor.getStatorCurrent();
        tempSig = motor.getDeviceTemp();

        // Set CAN update rate (20–50 Hz is plenty)
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            velocitySig, currentSig, tempSig
        );
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        // Bulk refresh (1 CAN transaction instead of 3)
        BaseStatusSignal.refreshAll(velocitySig, currentSig, tempSig);

        // Read cached values
        double motorRPS = velocitySig.getValueAsDouble();
        inputs.motorRPS = motorRPS;

        double motorRPM = motorRPS * 60.0;
        inputs.velocityRPM = ratio.toOutput(motorRPM);

        inputs.currentA = currentSig.getValueAsDouble();
        inputs.temperatureC = tempSig.getValueAsDouble();
    }

    @Override
    public void runRPM(double rpm) {
        double motorRPM = ratio.toMotor(rpm);
        double motorRPS = motorRPM / 60.0;
        motor.setControl(velocityRequest.withVelocity(motorRPS));
    }

    @Override
    public void runDuty(double duty) {
        motor.set(duty);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.set(0);
    }
}
