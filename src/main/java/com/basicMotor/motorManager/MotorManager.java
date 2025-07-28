package com.basicMotor.motorManager;

import com.basicMotor.LogFrame;
import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * This class is used to manage the {@link com.basicMotor.BasicMotor} instances.
 * It handles each motor's threads for the PID loop and sensor loop.
 * It also periodically logs the data of the motors.
 * You need to call the {@link #periodic()} method in the periodic method of your robot.java.
 * If you want to change some of the default values or the frequency of the loops, check {@link MotorManagerConfig}.
 */
public class MotorManager {

    /**
     * The configuration for the motor manager.
     * This stores the default values for some of the parameters of the motor manager.
     * It Also stores the frequency of the sensor loop and the PID loop.
     * If you want to change one of the parameters, You must change the value before constructing the motors.
     * This is static to make simpler calls to the motor manager.
     */
    public static MotorManagerConfig config = MotorManagerConfig.DEFAULT_CONFIG;

    /**
     * The singleton instance of the MotorManager.
     */
    private static MotorManager instance;

    /**
     * The list of motors that are registered with the motor manager.
     */
    private final ArrayList<MotorHandler> motors = new ArrayList<>();

    /**
     * Private constructor to enforce singleton pattern.
     * Use {@link #getInstance()} to get the instance of the MotorManager.
     */
    private MotorManager() {
    }

    /**
     * Gets the singleton instance of the MotorManager.
     *
     * @return The instance of the MotorManager.
     */
    public static MotorManager getInstance() {
        if (instance == null) {
            instance = new MotorManager();
        }
        return instance;
    }

    /**
     * The periodic method that should be called in the periodic method of your robot.java.
     * This method sends all the motor data to the logger.
     * This method is called with the codes periodic due to the logger not supporting multithreading.
     */
    public void periodic() {
        for (MotorHandler motor : motors) {
            var frame = motor.frameSupplier.get();
            Logger.processInputs("Motors/" + motor.motorName, frame);
        }
    }

    /**
     * Stops the sensor loop for all motors.
     * Should only be used if using replay simulation.
     */
    public void stopSensorLoop() {
        for (MotorHandler motor : motors) {
            motor.sensorLoop.stop();
        }
    }

    /**
     * Stops the PID loop for all motors.
     * Should only be used if using replay simulation.
     */
    public void stopPIDLoop() {
        for (MotorHandler motor : motors) {
            motor.pidLoop.stop();
        }
    }

    /**
     * Starts the sensor loop for all motors.
     */
    public void startSensorLoop() {
        for (MotorHandler motor : motors) {
            motor.sensorLoop.startPeriodic(1 / config.SENSOR_LOOP_HZ);
        }
    }

    /**
     * Starts the PID loop for all motors.
     */
    public void startPIDLoop() {
        for (MotorHandler motor : motors) {
            motor.pidLoop.startPeriodic(1 / motor.location.getHZ());
        }
    }

    /**
     * Registers a motor with the motor manager.
     * Use only once per motor.
     *
     * @param name               The name of the motor. (will be used in the logs)
     * @param location           The location of the pid loop. (RIO or Motor Controller).
     * @param run                The function to run for the main loop.
     * @param sensorLoopFunction The function to run for the sensor loop.
     * @param frameSupplier      The function to get the latest frame for the motor.
     */
    public void registerMotor(
            String name,
            ControllerLocation location,
            Runnable run,
            Runnable sensorLoopFunction,
            Supplier<LogFrame.LogFrameAutoLogged> frameSupplier) {

        var handler = new MotorHandler(name, location, run, sensorLoopFunction, frameSupplier);

        motors.add(handler);

        handler.sensorLoop.startPeriodic(1 / config.SENSOR_LOOP_HZ);
        handler.pidLoop.startPeriodic(1 / location.getHZ());
    }

    /**
     * A class that stores the motor and it's threads.
     */
    private static class MotorHandler {
        /**
         * The name of the motor.
         */
        private final String motorName;
        /**
         * The location of the motor's pid loop.
         */
        private final ControllerLocation location;

        /**
         * The thread to run for the main (PID) loop.
         */
        private final Notifier pidLoop;

        /**
         * The thread to run for the sensor loop.
         */
        private final Notifier sensorLoop;

        /**
         * The supplier for the frame of the motor.
         */
        private final Supplier<LogFrame.LogFrameAutoLogged> frameSupplier;

        public MotorHandler(
                String motorName,
                ControllerLocation location,
                Runnable run,
                Runnable sensorLoopFunction,
                Supplier<LogFrame.LogFrameAutoLogged> frameSupplier) {
            this.motorName = motorName;
            this.location = location;

            pidLoop = new Notifier(run);
            sensorLoop = new Notifier(sensorLoopFunction);

            this.frameSupplier = frameSupplier;
        }
    }

    /**
     * An enum that represents the location of the pid controller.
     * This is used to determine how fast the pid loop runs.
     * Check the specific location for more information.
     */
    public enum ControllerLocation {
        /**
         * The pid controller is running on the motor controller.
         * This means that the code that is running on the rio is only checking for constraints, feedforwards and motion profiling.
         * This is the default location for most motors.
         * For most mechanisms, this is the best location to run the pid controller.
         * But it lacks control over the measurement source of the pid controller.
         * Takes the frequency from the {@link MotorManagerConfig#PROFILE_LOOP_HZ} which is the default profile loop frequency.
         */
        MOTOR(() -> config.PROFILE_LOOP_HZ),
        /**
         * The pid controller is running on the rio.
         * This means that the code on the rio is calculating everything and sending the output to the motor controller.
         * This is useful when you want to have more control over the measurement source of the pid controller.
         * But it can lead to more canbus traffic and robo rio cpu usage.
         * Takes the frequency from the {@link MotorManagerConfig#PID_LOOP_HZ} which is the default pid loop frequency.
         */
        RIO(() -> config.PID_LOOP_HZ),;

        /**
         * The supplier of the frequency of the controller loop in Hz.
         */
        private final DoubleSupplier hzSupplier;

        /**
         * Constructor for the ControllerLocation enum.
         *
         * @param hzSupplier The supplier of the frequency of the controller loop in Hz.
         */
        ControllerLocation(DoubleSupplier hzSupplier) {
            this.hzSupplier = hzSupplier;
        }

        /**
         * Gets the frequency of the controller loop in Hz.
         *
         * @return The frequency of the controller loop in Hz.
         */
        public double getHZ() {
            return hzSupplier.getAsDouble();
        }

        /**
         * Gets the time in seconds for one iteration of the controller loop.
         * @return The time in seconds for one iteration of the controller loop.
         */
        public double getSeconds() {
            return 1 / hzSupplier.getAsDouble();
        }

    }
}
