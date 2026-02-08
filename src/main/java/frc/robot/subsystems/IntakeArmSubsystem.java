//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// WPI Imports
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// CTRE Imports
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;
import frc.robot.utilities.MotorConfig;
// Mechanics
import frc.robot.mechanics.GearRatio;

// Constants

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
        //cant figure out how to return both positions.
        
    }

    public double getYELPositionDegrees() {
        double motorRot = intakeYELArmMotor.getPosition().getValueAsDouble();
        //double rightMotorRot = -intakeRightArmMotor.getPosition().getValueAsDouble();
        double outputRot = YELratio.toOutput(motorRot);
        //double rightOutputRot = -ratio.toOutput(rightMotorRot);

        return outputRot;
        //cant figure out how to return both positions.
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Arm/ Blue Angle", getBLUPositionDegrees());
        SmartDashboard.putNumber("Intake/Arm/ Yellow Angle", getYELPositionDegrees());
    }

}
