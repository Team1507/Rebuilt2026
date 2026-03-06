//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of IntakeArmIO.
 */
public class IntakeArmIOReal extends Subsystems1507 implements IntakeArmIO {

    private final MotorConfig bluConfig;
    private final TalonFXS bluMotor;
    private final TalonFXS yelMotor;

    private final GearRatio ratio;

    public IntakeArmIOReal(int bluCanID, MotorConfig bluConfig, int yelCanID, MotorConfig yelConfig, GearRatio ratio) {
        this.bluConfig = bluConfig;
        this.bluMotor = new TalonFXS(bluCanID);

        this.yelMotor = new TalonFXS(yelCanID);

        this.ratio = ratio;

        configureFXSMotor(bluMotor, bluConfig);
        configureFXSMotor(yelMotor, yelConfig);
    }

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        inputs.bluMotorRot = bluMotor.getPosition().getValueAsDouble();
        inputs.yelMotorRot = yelMotor.getPosition().getValueAsDouble();

        inputs.bluPositionDeg = ratio.sensorToReal(inputs.bluMotorRot);
        inputs.yelPositionDeg = ratio.sensorToReal(inputs.yelMotorRot);

        inputs.bluReverseLimit =
            bluMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

        inputs.yelReverseLimit =
            yelMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

        inputs.bluCurrentA = bluMotor.getStatorCurrent().getValueAsDouble();
        inputs.yelCurrentA = yelMotor.getStatorCurrent().getValueAsDouble();

        inputs.bluTempC = bluMotor.getDeviceTemp().getValueAsDouble();
        inputs.yelTempC = yelMotor.getDeviceTemp().getValueAsDouble();
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
        boolean bluAtRev = bluMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;
        boolean yelAtRev = yelMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround;

        if((bluAtRev || yelAtRev) && degrees < 0.0) {
            degrees = 0.0;
        }
        double motorRot = ratio.realToSensor(degrees);
        
        double ffVolts = computeGravityFF(degrees);
        
        bluMotor.setControl( new PositionVoltage(motorRot) 
            .withSlot(0) 
            .withFeedForward(ffVolts)
        );

        yelMotor.setControl(new PositionVoltage(motorRot) 
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
