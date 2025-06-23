package util.BasicMotor.Configuration;

import util.BasicMotor.Gains.CurrentLimits;
import util.BasicMotor.Gains.CurrentLimitsREV;

public class BasicSparkBaseConfig extends BasicMotorConfig{

    public final CurrentLimitConfig currentLimitConfig = new CurrentLimitConfig();

    public static class CurrentLimitConfig {
        /**
         * the maximum current output of the motor controller while in free speed (in amps)
         * free speed can be defined in {@link #freeSpeedRPS}
         * <p>
         * this is the max current the motor windings will allow before stopping,
         * this will usually be higher than the supply current limit by a wide margin.
         */
        public int freeSpeedCurrentLimit = 0;
        /**
         * the maximum current draw of the motor controller while in stall (in amps)
         * anything below the free speed is considered stall, free speed can be defined in
         * {@link #freeSpeedRPS}
         */
        public int stallCurrentLimit = 0;
        /**
         * the free speed of the motor controller (in RPS (revolutions per second))
         * any speed above this speed is considered free speed and the free speed current limit will be applied,
         * if below this speed the motor is considered in stall and the stall current limit is applied.
         * gear rato is automatically applied to the free speed so no need to divide by it
         */
        public double freeSpeedRPS = 0;
        /**
         * the max current output of the motor controller (in amps)
         * if the current output of the motor controller exceeds this limit, the motor controller will stop for a short time
         */
        public int secondaryCurrentLimit = 0;

        /**
         * creates the current limit configuration with the given values
         * @return the current limits of the motor controller
         */
        public CurrentLimits getCurrentLimits() {
            return new CurrentLimitsREV(freeSpeedCurrentLimit, stallCurrentLimit, freeSpeedRPS, secondaryCurrentLimit);
        }
    }
}
