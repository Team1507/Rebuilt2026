//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.util;

public record MotorConfig(
        MotorConfig.ControlMode mode,
        boolean motorInverted,

        double kP,
        double kI,
        double kD,

        double kV,
        double kS,
        double kA,

        double kG,
        MotorConfig.GravityType gravityType,

        double peakForwardVoltage,
        double peakReverseVoltage
) {

    public static enum ControlMode {
        DUTY_CYCLE,
        VELOCITY,
        POSITION,
        MOTION_MAGIC
    }

    public static enum GravityType {
        NONE,
        COSINE,
        SINE,
        CONSTANT
    }

    // Duty-cycle constructor
    public MotorConfig(double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.DUTY_CYCLE, false,
             0,0,0,
             0,0,0,
             0.0, GravityType.NONE,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    // Velocity constructor
    public MotorConfig(double kP, double kI, double kD,
                       double kV, double kS, double kA,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.VELOCITY, false,
             kP,kI,kD,
             kV,kS,kA,
             0.0, GravityType.NONE,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    // Position constructor
    public MotorConfig(boolean motorInverted, double kP, double kI, double kD,
                       double kG, GravityType gravityType,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(ControlMode.POSITION,motorInverted,
             kP,kI,kD,
             0,0,0,
             kG, gravityType,
             peakForwardVoltage,
             peakReverseVoltage);
    }

    public MotorConfig(ControlMode controlMode, boolean motorInverted,
                       double kP, double kI, double kD,
                       double kG, GravityType gravityType,
                       double peakForwardVoltage, double peakReverseVoltage) {
        this(controlMode, motorInverted,
             kP,kI,kD,
             0,0,0,
             kG, gravityType,
             peakForwardVoltage,
             peakReverseVoltage);
    }
}
