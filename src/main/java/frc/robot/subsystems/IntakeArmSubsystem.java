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

// Mechanics
import frc.robot.mechanics.GearRatio;

// Constants
import frc.robot.Constants.kIntake.kArm.BLU;
import frc.robot.Constants.kIntake.kArm.YEL;;

/**
 * Intake Arm Subsystem
 */
public class IntakeArmSubsystem extends Subsystems1507 {
    
    private final TalonFXS intakeBLUArmMotor;
    private final TalonFXS intakeYELArmMotor;
    private final PositionDutyCycle positionRequest = new PositionDutyCycle(0).withSlot(0);

    private GearRatio ratio = GearRatio.gearBox(1, 2);

    /** Creates a new IntakeSubsystem. */
    public IntakeArmSubsystem(TalonFXS motorBLU,TalonFXS motorYEL) {
        this.intakeBLUArmMotor = motorBLU;
        this.intakeYELArmMotor = motorYEL;
        configureMotors();
    }

    private void configureMotors() {
            
        TalonFXSConfiguration cfgBLU = new TalonFXSConfiguration();
        cfgBLU.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfgBLU.Slot0.kP = BLU.kGains.KP;
        cfgBLU.Slot0.kI = BLU.kGains.KI;
        cfgBLU.Slot0.kD = BLU.kGains.KD;

        cfgBLU.Slot0.kV = BLU.kGains.KV;
        cfgBLU.Slot0.kS = BLU.kGains.KS;
        cfgBLU.Slot0.kA = BLU.kGains.KA;

        // --- VOLTAGE LIMITS ---
        cfgBLU.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));

        intakeBLUArmMotor.getConfigurator().apply(cfgBLU);

        TalonFXSConfiguration cfgYEL = new TalonFXSConfiguration();
        cfgYEL.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        cfgYEL.Slot0.kP = YEL.kGains.KP;
        cfgYEL.Slot0.kI = YEL.kGains.KI;
        cfgYEL.Slot0.kD = YEL.kGains.KD;

        cfgYEL.Slot0.kV = YEL.kGains.KV;
        cfgYEL.Slot0.kS = YEL.kGains.KS;
        cfgYEL.Slot0.kA = YEL.kGains.KA;

        // --- VOLTAGE LIMITS ---
        cfgYEL.Voltage.withPeakForwardVoltage(Volts.of(8))
                    .withPeakReverseVoltage(Volts.of(-8));
        intakeYELArmMotor.getConfigurator().apply(cfgYEL);
    }

    public void setPosition(double degrees){
        double leftOutputRot = degrees / 360.0;
        double rightOutputRot = -degrees / 360.0;
        double leftMotorRot = ratio.toMotor(leftOutputRot);
        double rightMotorRot = -ratio.toMotor(rightOutputRot);

        intakeBLUArmMotor.setControl(positionRequest.withPosition(leftMotorRot));
        intakeYELArmMotor.setControl(positionRequest.withPosition(rightMotorRot));
    }

    public double getPositionDegrees() {
        double leftMotorRot = intakeBLUArmMotor.getPosition().getValueAsDouble();
        //double rightMotorRot = -intakeRightArmMotor.getPosition().getValueAsDouble();
        double leftOutputRot = ratio.toOutput(leftMotorRot);
        //double rightOutputRot = -ratio.toOutput(rightMotorRot);

        return leftOutputRot;
        //cant figure out how to return both positions.
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake/Arm Angle", getPositionDegrees());
    }

}
