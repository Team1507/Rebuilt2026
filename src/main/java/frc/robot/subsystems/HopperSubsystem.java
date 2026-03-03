//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.lib.io.hopper.HopperIO;
import frc.lib.io.hopper.HopperInputs;
import frc.robot.Constants.kHopper;

/**
 * Thin, IO-based hopper subsystem.
 */
public class HopperSubsystem extends SubsystemBase {

    private final HopperIO io;
    private final HopperInputs inputs = new HopperInputs();

    public HopperSubsystem(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void setPosition(double degrees) {
        io.setPositionDeg(degrees);
    }

    public double getPositionDegrees() {
        return inputs.position;
    }

    public void runPower(double power)
    {
        if(power > 0.5) {
            io.runPower(kHopper.MANUAL_POSITIVE_POWER);
        }
        else if(power< -0.5){
            io.runPower(kHopper.MANUAL_NEGATIVE_POWER);
        }
        else {
            io.runPower(0);
        }
    }
    public void stop() {
        io.hopperStop();
    }
    public void MagnetOffset (){
        
    }

    public boolean isHopperExtended(){
        return inputs.hopperExtended;
    }
}
