package com.basicMotor.motors.talonFX;

import com.basicMotor.LogFrame;
import com.basicMotor.MotorManager.MotorManager;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

/**
 * This class is used to manage the sensors of a TalonFX motor controller.
 * It handles updating the sensor data at a specified refresh rate
 * and provides methods to retrieve the sensor data.
 */
public class TalonFXSensors {
    /**
     * The multiplier for the timeout when using waitForAll.
     * If the user is using waitForAll (pro with canivore), this will be the multiplier for the timeout time.
     * This will be multiplied by the refresh rate to determine the timeout time.
     * Change this only if you know what you are doing.
     */
    public static int TIMEOUT_REFRESH_MULTIPLIER = 4;

    /**
     * The refresh rate of the sensors in Hz.
     * This is how often the sensors will be updated.
     * This should be the same as the speed of the sensor thread.
     */
    private final double refreshHZ;

    /**
     * The timeout for waiting for all signals to update.
     * Used only when Pro features are enabled.
     * This is the refresh rate divided by {@link #TIMEOUT_REFRESH_MULTIPLIER}.
     */
    private final double timeout;

    /**
     * The location of the motor controller.
     * This is used to determine if there is need to log the motor's built-in PID output.
     * Useful for logging and debugging purposes.
     */
    private final MotorManager.ControllerLocation location;

    /**
     * The status signal containing the temperature
     */
    private final StatusSignal<Temperature> temperatureSignal;
    /**
     * The current draw of the motor controller
     */
    private final StatusSignal<Current> supplyCurrentSignal;
    /**
     * The current output of the motor controller
     */
    private final StatusSignal<Current> statorCurrentSignal;
    /**
     * The voltage output of the motor controller
     */
    private final StatusSignal<Voltage> motorVoltageSignal;
    /**
     * The voltage input of the motor controller
     */
    private final StatusSignal<Voltage> supplyVoltageSignal;
    /**
     * The duty cycle of the motor controller
     */
    private final StatusSignal<Double> dutyCycleSignal;

    /**
     * The total output of the pid controller
     */
    private final StatusSignal<Double> totalOutput;
    /**
     * The P output of the pid controller
     */
    private final StatusSignal<Double> kpOutput;
    /**
     * The I output of the pid controller
     */
    private final StatusSignal<Double> kiOutput;
    /**
     * The D output of the pid controller
     */
    private final StatusSignal<Double> kdOutput;

    /**
     * The collection of status signals that will be updated at the refresh rate.
     */
    private final BaseStatusSignal[] statusSignals;

    /**
     * The latest PID output from the motor controller.
     * This is used for logging purposes when the controller is on the motor controller.
     */
    private LogFrame.PIDOutput latestPIDOutput = LogFrame.PIDOutput.EMPTY;

    /**
     * Whether to wait for all status signals to update before returning the sensor data.
     * This requires Pro features to be enabled and is only used if the controller is on a CANivore.
     */
    private boolean waitForAll = false;

    /**
     * Constructs a TalonFXSensors object with the given motor and refresh rate.
     *
     * @param motor     The TalonFX motor controller to get the sensors from.
     * @param refreshHZ The refresh rate of the sensors in Hz.
     * @param location  The location of the pid controller (RIO or MOTOR).
     *                  This is used to determine if there is need to log the motor's built-in PID output.
     */
    public TalonFXSensors(TalonFX motor, double refreshHZ, MotorManager.ControllerLocation location) {
        this.refreshHZ = refreshHZ;
        this.location = location;

        timeout = 1 / (refreshHZ * TIMEOUT_REFRESH_MULTIPLIER);

        temperatureSignal = motor.getDeviceTemp(false);
        supplyCurrentSignal = motor.getSupplyCurrent(false);
        statorCurrentSignal = motor.getStatorCurrent(false);
        motorVoltageSignal = motor.getMotorVoltage(false);
        supplyVoltageSignal = motor.getSupplyVoltage(false);
        dutyCycleSignal = motor.getDutyCycle(false);

        kpOutput = motor.getClosedLoopProportionalOutput(false);
        kiOutput = motor.getClosedLoopIntegratedOutput(false);
        kdOutput = motor.getClosedLoopDerivativeOutput(false);
        totalOutput = motor.getClosedLoopOutput(false);

        // if the controller is on the rio
        if (location == MotorManager.ControllerLocation.RIO) {
            statusSignals = new BaseStatusSignal[]{
                    temperatureSignal,
                    supplyCurrentSignal,
                    statorCurrentSignal,
                    motorVoltageSignal,
                    supplyVoltageSignal,
                    dutyCycleSignal
            };
        } else {
            statusSignals = new BaseStatusSignal[]{
                    temperatureSignal,
                    supplyCurrentSignal,
                    statorCurrentSignal,
                    motorVoltageSignal,
                    supplyVoltageSignal,
                    dutyCycleSignal,
                    totalOutput,
                    kpOutput,
                    kiOutput,
                    kdOutput
            };
        }

        for (BaseStatusSignal signal : statusSignals) {
            signal.setUpdateFrequency(refreshHZ);
        }
    }

    /**
     * Gets the sensor data from the motor controller.
     * This refreshes the status signals based on the configured refresh rate.
     *
     * @return The sensor data.
     */
    public LogFrame.SensorData getSensorData() {
        if (waitForAll) BaseStatusSignal.waitForAll(timeout);
        else BaseStatusSignal.refreshAll(statusSignals);

        double temperature = temperatureSignal.getValueAsDouble();
        double currentDraw = supplyCurrentSignal.getValueAsDouble();
        double currentOutput = statorCurrentSignal.getValueAsDouble();
        double voltageOutput = motorVoltageSignal.getValueAsDouble();
        double voltageInput = supplyVoltageSignal.getValueAsDouble();
        double powerDraw = currentDraw * voltageInput;
        double powerOutput = currentOutput * voltageOutput;
        double dutyCycle = dutyCycleSignal.getValueAsDouble();

        // updates the latest pid output if the controller is on the motor controller
        // used for logging
        if (location == MotorManager.ControllerLocation.MOTOR) {
            double pOutput = kpOutput.getValueAsDouble();
            double iOutput = kiOutput.getValueAsDouble();
            double dOutput = kdOutput.getValueAsDouble();
            double pidOutput = totalOutput.getValueAsDouble();

            latestPIDOutput = new LogFrame.PIDOutput(pOutput, iOutput, dOutput, pidOutput);
        }

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                currentOutput,
                voltageOutput,
                voltageInput,
                powerDraw,
                powerOutput,
                dutyCycle);
    }

    /**
     * Sets the update frequency of the duty cycle signal.
     * This is used when the motor is followed by another motor and needs a fast duty cycle update.
     *
     * @param defaultRate Whether to set the duty cycle to the default rate (100 Hz) or to the refresh rate.
     */
    public void setDutyCycleToDefaultRate(boolean defaultRate) {
        dutyCycleSignal.setUpdateFrequency(defaultRate ? 100 : refreshHZ);
    }

    /**
     * Sets whether to enable waiting for all signals to update before returning values.
     * This is only used if the controller is on a CANivore and Pro features are enabled.
     * If enabled without a CANivore, it will slow down the system significantly
     *
     * @param enable True to enable waiting for all signals to update before returning values,
     */
    public void setWaitForAll(boolean enable) {
        this.waitForAll = enable;
    }

    /**
     * Gets the latest PID output from the motor controller.
     *
     * @return The latest PID output.
     */
    public LogFrame.PIDOutput getPIDLatestOutput() {
        return latestPIDOutput;
    }
}
