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

import frc.lib.hardware.IntakeArmHardware;
import frc.lib.math.GearRatio;
import frc.lib.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of IntakeArmIO.
 */
public class IntakeArmIOReal extends Subsystems1507 implements IntakeArmIO {

    private final MotorConfig bluConfig;
    private final TalonFXS bluMotor;
    private final TalonFXS yelMotor;

    private final GearRatio ratio = IntakeArmHardware.RATIO;

    public IntakeArmIOReal(MotorConfig bluConfig, MotorConfig yelConfig) {
        this.bluConfig = bluConfig;
        this.bluMotor = new TalonFXS(IntakeArmHardware.BLU_ID);

        this.yelMotor = new TalonFXS(IntakeArmHardware.YEL_ID);

        configureFXSMotor(bluMotor, bluConfig);
        configureFXSMotor(yelMotor, yelConfig);
    }

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        inputs.bluMotorRot = bluMotor.getPosition().getValueAsDouble();
        inputs.yelMotorRot = yelMotor.getPosition().getValueAsDouble();

        inputs.bluPositionDeg = ratio.toOutput(inputs.bluMotorRot);
        inputs.yelPositionDeg = ratio.toOutput(inputs.yelMotorRot);

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
    public void setPositionDeg(double degrees) {
        double motorRot = ratio.toMotor(degrees);
        
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
        bluMotor.set(power);
        yelMotor.set(power);
    }

    @Override
    public void stop() {
        bluMotor.set(0);
        yelMotor.set(0);
    }

}
