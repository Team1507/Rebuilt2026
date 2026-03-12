//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.io.climber;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.lib.core.math.GearRatio;
import frc.lib.core.util.MotorConfig;
import frc.robot.framework.base.Subsystems1507;

/**
 * Real hardware implementation of ClimberIO.
 */
public class ClimberIOReal extends Subsystems1507 implements ClimberIO {

    private final TalonFX motor;
    private final Servo servo;
    private final DigitalInput limitSwitch;
    private final GearRatio ratio;

    private final PositionDutyCycle positionRequest =
        new PositionDutyCycle(0).withSlot(0);

    public ClimberIOReal(int canID, MotorConfig configSlot0, MotorConfig configSlot1, GearRatio ratio, int servoDIO, int limitSwitchID) {
        this.motor = new TalonFX(canID);
        this.servo = new Servo(servoDIO);
        this.limitSwitch = new DigitalInput(limitSwitchID);
        this.ratio = ratio;

        configureFXMotor(motor, configSlot0, configSlot1);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.motorPosition = motor.getPosition().getValueAsDouble();
        inputs.position = ratio.toOutput(inputs.motorPosition);
        inputs.currentA = motor.getStatorCurrent().getValueAsDouble();
        inputs.temperatureC = motor.getDeviceTemp().getValueAsDouble();
        inputs.limitSwitch = !limitSwitch.get(); // invert if wired NC
        inputs.servoPosition = servo.get();
    }

    @Override
    public void setPosition(double mechanismPosition) {
        double motorPos = ratio.toMotor(mechanismPosition);
        motor.setControl(positionRequest.withPosition(motorPos));
    }

    @Override
    public void setServo(double position) {
        servo.set(position);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
