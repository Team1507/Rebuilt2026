//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.utilities;

public record MotorConfig(
        MotorConfig.ControlMode mode,

        double kP,
        double kI,
        double kD,

        double kV,
        double kS,
        double kA,

        double peakForwardVoltage,
        double peakReverseVoltage
) {

    public static enum ControlMode {
        DUTY_CYCLE,
        VELOCITY,
        POSITION,
        MOTION_MAGIC
    }

    // Duty-cycle constructor
    public MotorConfig(double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.DUTY_CYCLE,
             0,0,0,
             0,0,0,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    // Velocity constructor
    public MotorConfig(double kP, double kI, double kD,
                       double kV, double kS, double kA,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.VELOCITY,
             kP,kI,kD,
             kV,kS,kA,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    // Position constructor
    public MotorConfig(double kP, double kI, double kD,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.POSITION,
             kP,kI,kD,
             0,0,0,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    public MotorConfig(ControlMode controlMode,
                       double kP, double kI, double kD,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(controlMode,
             kP,kI,kD,
             0,0,0,
             peakForwardVoltage,
             peakReverseVoltage);
    }
}
