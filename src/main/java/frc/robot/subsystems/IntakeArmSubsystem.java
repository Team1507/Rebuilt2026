//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// CTRE Imports
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

// Extras
import frc.robot.mechanics.GearRatio;
import frc.robot.utilities.MotorConfig;

/**
 * Intake Arm Subsystem
 */
public class IntakeArmSubsystem extends Subsystems1507 {
    
    private final TalonFXS intakeBLUArmMotor;
    private final TalonFXS intakeYELArmMotor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

    private final GearRatio BLUratio;
    private final GearRatio YELratio;

    /** Creates a new IntakeSubsystem. */
    public IntakeArmSubsystem(MotorConfig motorBLU,MotorConfig motorYEL) {
        this.intakeBLUArmMotor = new TalonFXS(motorBLU.CAN_ID());
        this.intakeYELArmMotor = new TalonFXS(motorYEL.CAN_ID());

        this.BLUratio = motorBLU.ratio();
        this.YELratio = motorYEL.ratio();

        configureFXSMotor(motorBLU, intakeBLUArmMotor);
        configureFXSMotor(motorYEL, intakeYELArmMotor);
    }

    public void setPosition(double degrees){
        double BLUOutputRot = degrees / 360.0;
        double YELOutputRot = -degrees / 360.0;

        double BLUMotorRot = BLUratio.toMotor(BLUOutputRot);
        double YELMotorRot = YELratio.toMotor(YELOutputRot);

        intakeBLUArmMotor.setControl(positionRequest.withPosition(BLUMotorRot));
        intakeYELArmMotor.setControl(positionRequest.withPosition(YELMotorRot));
    }

    public double getBLUPositionDegrees() {
        double motorRot = intakeBLUArmMotor.getPosition().getValueAsDouble();
        //double rightMotorRot = -intakeRightArmMotor.getPosition().getValueAsDouble();
        double outputRot = BLUratio.toOutput(motorRot);
        //double rightOutputRot = -ratio.toOutput(rightMotorRot);

        return outputRot;
    }

    public double getYELPositionDegrees() {
        double motorRot = intakeYELArmMotor.getPosition().getValueAsDouble();
        //double rightMotorRot = -intakeRightArmMotor.getPosition().getValueAsDouble();
        double outputRot = YELratio.toOutput(motorRot);
        //double rightOutputRot = -ratio.toOutput(rightMotorRot);

        return outputRot;
    }
}
