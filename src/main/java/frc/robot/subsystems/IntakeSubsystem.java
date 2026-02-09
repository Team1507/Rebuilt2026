//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

// CTRE Imports
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

// Subsystems
import frc.robot.subsystems.lib.Subsystems1507;

// Extras
import frc.robot.utilities.MotorConfig;
import frc.robot.mechanics.GearRatio;

public class IntakeSubsystem extends Subsystems1507 {
    private final TalonFX intakeMotor;
    private final DutyCycleOut dutyRequest = new DutyCycleOut(0);
    private final GearRatio ratio;

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem(MotorConfig config) {
        this.intakeMotor = new TalonFX (config.CAN_ID());
        this.ratio = config.ratio();
        configureFXMotor(config, intakeMotor);
    }

    public void run(double dutyCycle) {
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
}