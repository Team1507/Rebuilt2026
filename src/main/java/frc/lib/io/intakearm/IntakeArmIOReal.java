//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

public class IntakeArmIOReal extends Subsystems1507 implements IntakeArmIO {

    private final MotorConfig bluConfig;
    private final TalonFXS bluMotor;
    private final TalonFXS yelMotor;

    private final GearRatio ratio;

    // -----------------------------
    // Cached CTRE signals (BLU)
    // -----------------------------
    private final StatusSignal<Angle> bluPosSig;
    private final StatusSignal<AngularVelocity> bluVelSig;
    private final StatusSignal<Voltage> bluVoltSig;
    private final StatusSignal<ReverseLimitValue> bluRevLimitSig;
    private final StatusSignal<Current> bluCurrentSig;
    private final StatusSignal<Temperature> bluTempSig;

    // -----------------------------
    // Cached CTRE signals (YEL)
    // -----------------------------
    private final StatusSignal<Angle> yelPosSig;
    private final StatusSignal<AngularVelocity> yelVelSig;
    private final StatusSignal<Voltage> yelVoltSig;
    private final StatusSignal<ReverseLimitValue> yelRevLimitSig;
    private final StatusSignal<Current> yelCurrentSig;
    private final StatusSignal<Temperature> yelTempSig;

    public IntakeArmIOReal(
            int bluCanID, MotorConfig bluConfig,
            int yelCanID, MotorConfig yelConfig,
            GearRatio ratio) {

        this.bluConfig = bluConfig;
        this.bluMotor = new TalonFXS(bluCanID);
        this.yelMotor = new TalonFXS(yelCanID);
        this.ratio = ratio;

        configureFXSMotor(bluMotor, bluConfig);
        configureFXSMotor(yelMotor, yelConfig);

        // -----------------------------
        // BLU signals
        // -----------------------------
        bluPosSig = bluMotor.getPosition();
        bluVelSig = bluMotor.getVelocity();
        bluVoltSig = bluMotor.getMotorVoltage();
        bluRevLimitSig = bluMotor.getReverseLimit();
        bluCurrentSig = bluMotor.getStatorCurrent();
        bluTempSig = bluMotor.getDeviceTemp();

        // -----------------------------
        // YEL signals
        // -----------------------------
        yelPosSig = yelMotor.getPosition();
        yelVelSig = yelMotor.getVelocity();
        yelVoltSig = yelMotor.getMotorVoltage();
        yelRevLimitSig = yelMotor.getReverseLimit();
        yelCurrentSig = yelMotor.getStatorCurrent();
        yelTempSig = yelMotor.getDeviceTemp();

        // -----------------------------
        // Set update frequency
        // -----------------------------
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            bluPosSig, bluVelSig, bluVoltSig, bluRevLimitSig, bluCurrentSig, bluTempSig,
            yelPosSig, yelVelSig, yelVoltSig, yelRevLimitSig, yelCurrentSig, yelTempSig
        );
    }

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        BaseStatusSignal.refreshAll(
            bluPosSig, bluVelSig, bluVoltSig, bluRevLimitSig, bluCurrentSig, bluTempSig,
            yelPosSig, yelVelSig, yelVoltSig, yelRevLimitSig, yelCurrentSig, yelTempSig
        );

        // -----------------------------
        // BLU motor
        // -----------------------------
        inputs.bluMotorRot = bluPosSig.getValueAsDouble();
        inputs.bluPositionDeg = ratio.sensorToReal(inputs.bluMotorRot);
        inputs.bluVelocityDegPerSec = ratio.sensorToReal(bluVelSig.getValueAsDouble());
        inputs.bluAppliedVolts = bluVoltSig.getValueAsDouble();
        inputs.bluReverseLimit = bluRevLimitSig.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.bluCurrentA = bluCurrentSig.getValueAsDouble();
        inputs.bluTempC = bluTempSig.getValueAsDouble();

        // -----------------------------
        // YEL motor
        // -----------------------------
        inputs.yelMotorRot = yelPosSig.getValueAsDouble();
        inputs.yelPositionDeg = ratio.sensorToReal(inputs.yelMotorRot);
        inputs.yelVelocityDegPerSec = ratio.sensorToReal(yelVelSig.getValueAsDouble());
        inputs.yelAppliedVolts = yelVoltSig.getValueAsDouble();
        inputs.yelReverseLimit = yelRevLimitSig.getValue() == ReverseLimitValue.ClosedToGround;
        inputs.yelCurrentA = yelCurrentSig.getValueAsDouble();
        inputs.yelTempC = yelTempSig.getValueAsDouble();
    }

    // -------------------------
    // Gravity Feedforward
    // -------------------------
    private double computeGravityFF(double armDeg) {
        return switch (bluConfig.gravityType()) {
            case NONE -> 0.0;
            case CONSTANT -> bluConfig.kG();
            case COSINE -> bluConfig.kG() * Math.cos(Math.toRadians(armDeg));
            case SINE -> bluConfig.kG() * Math.sin(Math.toRadians(armDeg));
        };
    }

    @Override
    public void setAngle(double degrees) {
        boolean bluAtRev = bluRevLimitSig.getValue() == ReverseLimitValue.ClosedToGround;
        boolean yelAtRev = yelRevLimitSig.getValue() == ReverseLimitValue.ClosedToGround;

        if ((bluAtRev || yelAtRev) && degrees < 0.0) {
            degrees = 0.0;
        }

        double motorRot = ratio.realToSensor(degrees);
        double ffVolts = computeGravityFF(degrees);

        bluMotor.setControl(
            new PositionVoltage(motorRot)
                .withSlot(0)
                .withFeedForward(ffVolts)
        );

        yelMotor.setControl(
            new PositionVoltage(motorRot)
                .withSlot(0)
                .withFeedForward(ffVolts)
        );
    }

    @Override
    public void runPower(double power) {
        // -----------------------------
        // BLU MOTOR SAFETY CHECKS
        // -----------------------------

        // Read the BLU motor's reverse limit switch (true = switch is pressed)
        boolean bluAtRev = bluMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

        // Start with the requested power
        double safeBLUPower = power;

        // If the BLU motor is at the reverse limit AND the command is trying to move further negative,
        // block that movement to prevent mechanical damage.
        if (bluAtRev && power < 0.0) {
            safeBLUPower = 0.0;
        }

        // -----------------------------
        // YEL MOTOR SAFETY CHECKS
        // -----------------------------

        // Read the YEL motor's reverse limit switch (true = switch is pressed)
        boolean yelAtRev = yelMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

        // Start with the requested power
        double safeYELPower = power;

        // If the YEL motor is at the reverse limit AND the command is trying to move further negative,
        // block that movement to prevent mechanical damage.
        if (yelAtRev && power < 0.0) {
            safeYELPower = 0.0;
        }

        // Apply the safe power to the BLU motor
        bluMotor.set(safeBLUPower);

        // Apply the safe power to the YEL motor
        yelMotor.set(safeYELPower);
    }


    @Override
    public void stop() {
        bluMotor.set(0);
        yelMotor.set(0);
    }

}
