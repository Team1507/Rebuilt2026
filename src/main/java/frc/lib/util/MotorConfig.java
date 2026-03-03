//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package frc.lib.util;

public record MotorConfig(
        int slotNumber,

        ControlMode mode,
        boolean motorInverted,

        double kP, double kI, double kD,
        double kV, double kS, double kA,

        double kG,
        GravityType gravityType,

        double peakForwardVoltage,
        double peakReverseVoltage
) {

    public static enum ControlMode { DUTY_CYCLE, VELOCITY, POSITION, MOTION_MAGIC }
    public static enum GravityType { NONE, COSINE, SINE, CONSTANT }

    public static Builder builder() {
        return new Builder();
    }

    public static Builder builder(ControlMode mode) {
        return new Builder().mode(mode);
    }

    public static final class Builder {
        
        private int slotNumber = 0;

        private ControlMode mode = ControlMode.DUTY_CYCLE;
        private boolean motorInverted = false;

        private double kP = 0, kI = 0, kD = 0;
        private double kV = 0, kS = 0, kA = 0;

        private double kG = 0;
        private GravityType gravityType = GravityType.NONE;

        private double peakForwardVoltage = 12;
        private double peakReverseVoltage = -12;


        public Builder mode(ControlMode m) { this.mode = m; return this; }
        public Builder inverted(boolean inv) { this.motorInverted = inv; return this; }

        public Builder pid(double p, double i, double d) {
            this.kP = p; this.kI = i; this.kD = d; return this;
        }

        public Builder feedforward(double kS, double kV, double kA) {
            this.kS = kS; this.kV = kV; this.kA = kA; return this;
        }

        public Builder gravity(double kG, GravityType type) {
            this.kG = kG; this.gravityType = type; return this;
        }

        public Builder voltageLimits(double fwd, double rev) {
            this.peakForwardVoltage = fwd; this.peakReverseVoltage = rev; return this;
        }

        public Builder slot(int slot) {
            this.slotNumber = slot; return this;
        }

        public MotorConfig build() {
            return new MotorConfig(
                slotNumber,
                mode, motorInverted,
                kP, kI, kD,
                kV, kS, kA,
                kG, gravityType,
                peakForwardVoltage, peakReverseVoltage
            );
        }
    }
}
