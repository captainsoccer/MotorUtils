package com.basicMotor.gains.currentLimits;

/**
 * This class represents the current limits of a Spark Base motor controller.
 * It can be used on a Spark Max or a Spark Flex motor controller.
 * It has specific current limits offered by the Spark Base motor controller.
 * Use this for Spark Max and Spark Flex motor controllers instead of the generic {@link CurrentLimits} interface.
 */
public class CurrentLimitsSparkBase implements CurrentLimits {

    /**
     * The maximum current output of the motor controller (in amps) when not in stall.
     * If the stall current limit is not set, the motor will use this as the current limit at all times.
     * Free speed is any speed above {@link #freeSpeed}.
     * If the motor reaches this current limit, it will lower the output voltage to prevent exceeding this limit.
     */
    private final int freeSpeedCurrentLimit;

    /**
     * The maximum current output of the motor controller (in amps) while in stall.
     * If this value is not set, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * Stall is defined as any speed below {@link #freeSpeed}.
     * If the motor reaches this current limit, it will lower the output voltage to prevent exceeding this limit.
     */
    private final int stallCurrentLimit;

    /**
     * The secondary current limit of the motor controller (in amps).
     * If the motor reaches this current limit, it will stop for a short time.
     * This is output current limit, not supply current limit.
     * Useful for small motors like the Neo 550, which can burn out quickly.
     */
    private final int secondaryCurrentLimit;

    /**
     * The speed of the mechanism in Hz (revolutions per second) that is considered as free speed.
     * Any speed below this speed is considered stall and the stall current limit is applied.
     * Otherwise, the free speed current limit is applied.
     * This value is in the mechanism's velocity after the gear ratio and unit conversion are applied.
     * If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     */
    private final double freeSpeed;

    /**
     * Creates a current limit with the given values
     *
     * @param freeSpeedCurrentLimit The maximum current output of the motor controller (in amps) when
     *                              not in stall.
     *                              If the stall current limit is not set, the motor will use this as the current limit at all times.
     *                              If this and stallCurrentLimit are zero, the motor will not limit the current output.
     * @param stallCurrentLimit     The maximum current output of the motor controller (in amps) while in stall.
     *                              If this value is zero, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * @param freeSpeed             The speed of the mechanism (units by default are in Hz (revolutions per second) but can be changed by the unit conversion).
     *                              If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *                              Otherwise, the free speed current limit is applied.
     *                              If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     * @param secondaryCurrentLimit The secondary current limit of the motor controller (in amps).
     *                              When the motor reaches this current limit, it will stop for a short time.
     */
    public CurrentLimitsSparkBase(int freeSpeedCurrentLimit, int stallCurrentLimit, double freeSpeed, int secondaryCurrentLimit) {

        if (freeSpeedCurrentLimit < 0) {
            throw new IllegalArgumentException("Free speed current limit must be non-negative.");
        }
        this.freeSpeedCurrentLimit = freeSpeedCurrentLimit;

        if (stallCurrentLimit < 0) {
            throw new IllegalArgumentException("Stall current limit must be non-negative.");
        }
        this.stallCurrentLimit = stallCurrentLimit;

        if (freeSpeed < 0) {
            throw new IllegalArgumentException("Free speed RPS must be non-negative.");
        }
        this.freeSpeed = freeSpeed;

        if (secondaryCurrentLimit < 0) {
            throw new IllegalArgumentException("Secondary current limit must be non-negative.");
        }
        this.secondaryCurrentLimit = secondaryCurrentLimit;
    }

    /**
     * Creates a current limit with the given values
     *
     * @param freeSpeedCurrentLimit The maximum current output of the motor controller (in amps) when
     *                              not in stall.
     *                              If this and stallCurrentLimit are zero, the motor will not limit the current output.
     * @param stallCurrentLimit     The maximum current output of the motor controller (in amps) while in
     *                              stall.
     *                              If this value is zero, the motor will use only the {@link #freeSpeedCurrentLimit} as the current limit at all times.
     * @param freeSpeed             The speed of the mechanism (units by default are in Hz (revolutions per second) but can be changed by the unit conversion).
     *                              If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *                              Otherwise, the free speed current limit is applied.
     *                              If this value is zero, the motor will linearly interpolate between the free speed current limit and the stall current limit.
     */
    public CurrentLimitsSparkBase(
            int freeSpeedCurrentLimit,
            int stallCurrentLimit,
            double freeSpeed) {
        this(freeSpeedCurrentLimit, stallCurrentLimit, freeSpeed, 0);
    }

    /**
     * Creates a current limit with the given values
     *
     * @param currentLimit          The maximum current output of the motor controller (in amps).
     *                              If this is zero, there will be no current limit.
     * @param secondaryCurrentLimit The secondary current limit of the motor controller (in amps).
     *                              When the motor reaches this current limit, it will stop for a short time.
     *                              If this value is zero, the motor will not limit the current output.
     */
    public CurrentLimitsSparkBase(int currentLimit, int secondaryCurrentLimit) {
        this(currentLimit, 0, 0, secondaryCurrentLimit);
    }

    /**
     * Creates a current limit with the given values
     *
     * @param currentLimit          The maximum current output of the motor controller (in amps).
     */
    public CurrentLimitsSparkBase(int currentLimit) {
        this(currentLimit, 0, 0, 0);
    }

    @Override
    public int getCurrentLimit() {
        return freeSpeedCurrentLimit;
    }

    /**
     * Gets the stall current limit of the motor controller.
     * If it is not set, the motor will use only the {@link #getCurrentLimit()} as the current limit at all times.
     */
    public int getStallCurrentLimit() {
        return stallCurrentLimit;
    }

    /**
     * Gets the free speed of the motor controller.
     * Any speed above this speed is considered free speed and the free speed current limit will be applied.
     * If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *
     * @return The free speed of the motor controller.
     * (default units are in Hz (revolutions per second) but can be changed by the unit conversion)
     */
    public double getFreeSpeed() {
        return freeSpeed;
    }

    /**
     * Gets the secondary current limit of the motor controller
     *
     * @return The secondary current limit of the motor controller (in amps)
     */
    public int getSecondaryCurrentLimit() {
        return secondaryCurrentLimit;
    }
}
