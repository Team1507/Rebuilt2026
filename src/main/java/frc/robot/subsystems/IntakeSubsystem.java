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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

// Constants
import frc.robot.Constants.kIntake.kRoller.kGains;

public class IntakeSubsystem extends Subsystems1507 {
    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(TalonFX intakeMotor) {
        this.intakeMotor = intakeMotor;
        configureMotor();
    }

    private void configureMotor() {

        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.Slot0.kP = kGains.KP;
        cfg.Slot0.kI = kGains.KI;
        cfg.Slot0.kD = kGains.KD;

        cfg.Slot0.kV = kGains.KV;
        cfg.Slot0.kS = kGains.KS;
        cfg.Slot0.kA = kGains.KA;

        // --- VOLTAGE LIMITS ---
        cfg.Voltage.withPeakForwardVoltage(Volts.of(8))
            .withPeakReverseVoltage(Volts.of(-8));
        
        intakeMotor.getConfigurator().apply(cfg);
    }

    public void run(double dutyCycle) {
        SmartDashboard.putNumber("Intake/Roller Target Duty Cycle", dutyCycle);
        intakeMotor.setControl(dutyRequest.withOutput(dutyCycle));
    }

    public void setpower(double power) {
        intakeMotor.set(power);
    }    

    public void stop()  {
        intakeMotor.set(0);
    }

    public double getDutyCycle(){
        return intakeMotor.getDutyCycle().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Intake/Roller Duty Cycle", getDutyCycle());
    }
}