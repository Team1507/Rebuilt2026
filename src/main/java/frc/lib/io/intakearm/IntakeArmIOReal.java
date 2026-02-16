//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.intakearm;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import frc.lib.hardware.IntakeArmHardware;
import frc.lib.math.GearRatio;
import frc.robot.framework.base.Subsystems1507;
import frc.robot.utilities.MotorConfig;

/**
 * Real hardware implementation of IntakeArmIO.
 */
public class IntakeArmIOReal extends Subsystems1507 implements IntakeArmIO {

    private final TalonFXS bluMotor;
    private final TalonFXS yelMotor;

    private final GearRatio ratio = IntakeArmHardware.RATIO;

    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public IntakeArmIOReal(MotorConfig bluConfig, MotorConfig yelConfig) {
        this.bluMotor = new TalonFXS(IntakeArmHardware.BLU_ID);
        this.yelMotor = new TalonFXS(IntakeArmHardware.YEL_ID);

        configureFXSMotor(bluConfig, bluMotor);
        configureFXSMotor(yelConfig, yelMotor);

        // YEL follows BLU, but reversed
        yelMotor.setControl(new Follower(
            bluMotor.getDeviceID(),
            MotorAlignmentValue.Opposed
        ));
    }

    @Override
    public void updateInputs(IntakeArmInputs inputs) {
        inputs.bluMotorRot = bluMotor.getPosition().getValueAsDouble();
        inputs.yelMotorRot = yelMotor.getPosition().getValueAsDouble();

        inputs.bluPositionDeg = ratio.toOutput(inputs.bluMotorRot);
        inputs.yelPositionDeg = ratio.toOutput(inputs.yelMotorRot);

        inputs.bluCurrentA = bluMotor.getStatorCurrent().getValueAsDouble();
        inputs.yelCurrentA = yelMotor.getStatorCurrent().getValueAsDouble();

        inputs.bluTempC = bluMotor.getDeviceTemp().getValueAsDouble();
        inputs.yelTempC = yelMotor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void setPositionDeg(double degrees) {
        double motorRot = ratio.toMotor(degrees);
        bluMotor.setControl(positionRequest.withPosition(motorRot));
        // YEL follows automatically
    }

    @Override
    public void stop() {
        bluMotor.set(0);
        yelMotor.set(0);
    }
}
