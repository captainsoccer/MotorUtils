package com.basicMotor.Manager;

/**
 * a class to hold the configuration for the motor manager
 * you can change the values in this class to change the constants used in the motor manager
 */
public class MotorManagerConfig {

    /**
     * the frequency of the PID loop (in Hz) used for the PID loop
     */
    public final double PID_LOOP_HZ;
    /**
     * the frequency of the profile loop (in Hz) used for the profile loop used for the run function
     * when the controller runs on the motor
     */
    public final double PROFILE_LOOP_HZ;
    /**
     * the frequency of the sensor loop (in Hz) used for the sensor loop used for the updateSensorData
     * function when the controller runs on the motor
     */
    public final double SENSOR_LOOP_HZ;

    /**
     * the ideal voltage fed into the motor when it is not moving (in volts) used for duty cycle
     * calculations
     */
    public final double motorIdealVoltage;

    /**
     * the maximum output of the motor (in volts)
     */
    public final double defaultMaxMotorOutput;

    /**
     * constructor for the motor manager config
     *
     * @param PID_LOOP_HZ           the frequency of the PID loop (in Hz) used for the PID loop
     * @param PROFILE_LOOP_HZ       the frequency of the profile loop (in Hz) used for the profile loop
     * @param SENSOR_LOOP_HZ        the frequency of the sensor loop (in Hz) used for the sensor loop
     * @param defaultMaxMotorOutput the maximum output of the motor (in volts)
     * @param motorIdealVoltage      the idle voltage fed into the motor when it is not moving (in volts)
     */
    public MotorManagerConfig(double PID_LOOP_HZ, double PROFILE_LOOP_HZ, double SENSOR_LOOP_HZ, double defaultMaxMotorOutput, double motorIdealVoltage) {
        this.PID_LOOP_HZ = PID_LOOP_HZ;
        this.PROFILE_LOOP_HZ = PROFILE_LOOP_HZ;
        this.SENSOR_LOOP_HZ = SENSOR_LOOP_HZ;
        this.defaultMaxMotorOutput = defaultMaxMotorOutput;
        this.motorIdealVoltage = motorIdealVoltage;
    }

    /**
     * default constructor for the motor manager config
     */
    public MotorManagerConfig() {
        this.PID_LOOP_HZ = 100;
        this.PROFILE_LOOP_HZ = 50;
        this.SENSOR_LOOP_HZ = 4;
        this.defaultMaxMotorOutput = 12;
        this.motorIdealVoltage = 13;
    }

}
