package com.basicMotor.gains.currentLimits;

/**
 * this class takes the current limits class and makes it fully used for REV motor controllers the
 * rev motor controllers ignore the supply current limit and supply lower time if you don't need to
 * use the free speed RPM or stall current limit, you can use the normal CurrentLimits class
 */
public class CurrentLimitsREV implements CurrentLimits {

    /**
     * the maximum current output of the motor controller (in amps) when not in stall this is
     * different from stallCurrentLimit
     */
    private final int freeSpeedCurrentLimit;

    /**
     * the maximum current output of the motor controller (in amps) while in stall
     */
    private final int stallCurrentLimit;

    /**
     * the secondary current limit if the current output of the motor controller exceeds this limit,
     * the motor controller will stop for a short time
     */
    private final int secondaryCurrentLimit;

    /**
     * the free speed of the motor controller (in RPS) if below this speed the motor is considered in
     * stall and the stall current limit is applied
     */
    private final double freeSpeedRPS;

    /**
     * creates a current limit with the given values
     *
     * @param freeSpeedCurrentLimit the maximum current output of the motor controller (in amps) when
     *                              not in stall
     * @param stallCurrentLimit     the maximum current output of the motor controller (in amps) while in
     *                              stall
     * @param freeSpeed             The speed of the mechanism in Hz (revolutions per second) that is considered as free speed.
     *                              If the motor speed is below this speed, the motor is considered in stall and the stall current limit is applied.
     *                              Otherwise, the free speed current limit is applied.
     * @param secondaryCurrentLimit a secondary current limit that is not used by REV controllers
     */
    public CurrentLimitsREV(
            int freeSpeedCurrentLimit,
            int stallCurrentLimit,
            double freeSpeed,
            int secondaryCurrentLimit) {

        if(freeSpeedCurrentLimit < 0) {
            throw new IllegalArgumentException("Free speed current limit must be non-negative.");
        }
        this.freeSpeedCurrentLimit = freeSpeedCurrentLimit;

        if(stallCurrentLimit < 0) {
            throw new IllegalArgumentException("Stall current limit must be non-negative.");
        }
        this.stallCurrentLimit = stallCurrentLimit;

        if(freeSpeed < 0) {
            throw new IllegalArgumentException("Free speed RPS must be non-negative.");
        }
        this.freeSpeedRPS = freeSpeed;

        if(secondaryCurrentLimit < 0) {
            throw new IllegalArgumentException("Secondary current limit must be non-negative.");
        }
        this.secondaryCurrentLimit = secondaryCurrentLimit;

    }

    @Override
    public int getCurrentLimit() {
        return freeSpeedCurrentLimit;
    }

    /**
     * gets the stall current limit of the motor controller
     *
     * @return the stall current limit of the motor controller (in amps)
     */
    public int getStallCurrentLimit() {
        return stallCurrentLimit;
    }

    /**
     * gets the free speed of the motor controller
     *
     * @return the free speed of the motor controller (in RPS (revolutions per second))
     */
    public double getFreeSpeedRPS() {
        return freeSpeedRPS;
    }

    /**
     * gets the secondary current limit of the motor controller
     *
     * @return the secondary current limit of the motor controller (in amps)
     */
    public int getSecondaryCurrentLimit() {
        return secondaryCurrentLimit;
    }
}
