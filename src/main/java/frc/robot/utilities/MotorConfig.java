//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.utilities;

import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.mechanics.GearRatio;

public record MotorConfig(
        int CAN_ID,
        ControlMode mode,

        double kP,
        double kI,
        double kD,

        double kV,
        double kS,
        double kA,

        double peakForwardVoltage,
        double peakReverseVoltage,

        GearRatio ratio,
        Transform2d robotToMechanism
) {

    // -------------------------------
    // Control mode enum
    // -------------------------------
    public static enum ControlMode {
        DUTY_CYCLE,
        VELOCITY,
        POSITION,
        MOTION_MAGIC
    }

    // -------------------------------
    // Duty-cycle constructor
    // -------------------------------
    public MotorConfig(
        int CAN_ID,
        double peakForwardVoltage,
        double peakReverseVoltage
    ) {
        this(
            CAN_ID,
            ControlMode.DUTY_CYCLE,
            0,0,0,
            0,0,0,
            peakForwardVoltage,
            peakReverseVoltage,
            GearRatio.gearBox(1,1),
            new Transform2d()
        );
    }

    // -------------------------------
    // Velocity constructor
    // -------------------------------
    public MotorConfig(
        int CAN_ID,
        double kP, double kI, double kD,
        double kV, double kS, double kA,
        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio
    ) {
        this(
            CAN_ID,
            ControlMode.VELOCITY,
            kP,kI,kD,
            kV,kS,kA,
            peakForwardVoltage,
            peakReverseVoltage,
            ratio,
            new Transform2d()
        );
    }

    // -------------------------------
    // Position constructor
    // -------------------------------
    public MotorConfig(
        int CAN_ID,
        double kP, double kI, double kD,
        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio
    ) {
        this(
            CAN_ID,
            ControlMode.POSITION,
            kP,kI,kD,
            0,0,0,
            peakForwardVoltage,
            peakReverseVoltage,
            ratio,
            new Transform2d()
        );
    }

    // -------------------------------
    // MotionMagic constructor
    // -------------------------------
    public MotorConfig(
        int CAN_ID,
        ControlMode mode,
        double kP, double kI, double kD,
        double kV, double kS, double kA,
        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio
    ) {
        this(
            CAN_ID,
            mode,
            kP, kI, kD,
            kV, kS, kA,
            peakForwardVoltage,
            peakReverseVoltage,
            ratio,
            new Transform2d() // default, unused
        );
    }

    // -------------------------------
    // Full constructor (rarely used)
    // -------------------------------
    public MotorConfig(
        int CAN_ID,
        double kP, double kI, double kD,
        double kV, double kS, double kA,
        double peakForwardVoltage,
        double peakReverseVoltage,
        GearRatio ratio,
        Transform2d robotToMechanism
    ) {
        this(
            CAN_ID,
            ControlMode.VELOCITY,
            kP,kI,kD,
            kV,kS,kA,
            peakForwardVoltage,
            peakReverseVoltage,
            ratio,
            robotToMechanism
        );
    }
}
