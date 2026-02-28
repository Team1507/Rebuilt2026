//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.hardware;

import frc.lib.math.GearRatio;

/**
 * Hardware definition for the hopper motor.
 */
public final class HopperHardware {
    //hopper magnet
    public static final int HOPPER_DIO = 8;

    /** CAN ID for the hopper motor. */
    public static final int HOPPER_ID = 16;

    /** Gear ratio for the hopper gearbox. */
    public static final GearRatio RATIO = GearRatio.gearBox(20, 1);

    public interface RobotIO {
        boolean getMagSensor();
    }

    private HopperHardware() {}
}
