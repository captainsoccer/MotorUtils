package com.basicMotor.Manager;

import com.basicMotor.LogFrame;
import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * a class to manage the motors in the robot
 * it handles the PID loop, sensor loop, and logging of the motors
 * you can register motors with this class to manage them
 * you need to call the periodic method in the periodic method of your subsystem to log the motors
 */
public class MotorManager {

    /**
     * the configuration for the motor manager
     * it contains the constants used in the motor manager (loop times, idle voltage, etc.)
     * you can change the values in this class to change the constants used in the motor manager
     */
    public static MotorManagerConfig config = new MotorManagerConfig();

    /**
     * the instance of the motor manager used to manage the motors
     */
    private static MotorManager instance;

    /**
     * the list of motors used to manage the motors
     */
    private final ArrayList<MotorHandler> motors = new ArrayList<>();

    private MotorManager() {
    }

    /**
     * get the instance of the motor manager
     *
     * @return the instance of the motor manager
     */
    public static MotorManager getInstance() {
        if (instance == null) {
            instance = new MotorManager();
        }
        return instance;
    }

    /**
     * you need to call this method in the periodic method of your subsystem to log the
     */
    public void periodic() {
        for (MotorHandler motor : motors) {
            var frame = motor.frameSupplier.get();
            Logger.processInputs("Motors/" + motor.motorName, frame);
        }
    }

    /**
     * stop the PID loop for all motors useful for replay simulation
     */
    public void stopSensorLoop() {
        for (MotorHandler motor : motors) {
            motor.sensorLoop.stop();
        }
    }

    /**
     * stop the PID loop for all motors useful for replay simulation
     */
    public void stopPIDLoop() {
        for (MotorHandler motor : motors) {
            motor.pidLoop.stop();
        }
    }

    /**
     * start the sensor loop for all motors if you accidentally do something dumb
     */
    public void startSensorLoop() {
        for (MotorHandler motor : motors) {
            motor.sensorLoop.startPeriodic(1 / config.SENSOR_LOOP_HZ);
        }
    }

    /**
     * start the PID loop for all motors if you accidentally do something dumb
     */
    public void startPIDLoop() {
        for (MotorHandler motor : motors) {
            motor.pidLoop.startPeriodic(1 / motor.location.getHZ());
        }
    }

    /**
     * register a motor with the motor manager
     *
     * @param name               the name of the motor
     * @param location           the location of the motor (RIO or Motor Controller)
     * @param run                the function to run for the PID loop
     * @param sensorLoopFunction the function to run for the sensor loop
     * @param frameSupplier      the function to get the frame for the motor
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
     * a class to handle the motor
     */
    private static class MotorHandler {
        /**
         * the name of the motor
         */
        private final String motorName;
        /**
         * the location of the motor (RIO or Motor Controller)
         */
        private final ControllerLocation location;

        /**
         * the thread to run for the PID loop
         */
        private final Notifier pidLoop;

        /**
         * the thread to run for the sensor loop
         */
        private final Notifier sensorLoop;

        /**
         * the function to get the frame for the motor
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
     * an enum to represent the location of the controller.
     * the location of the controller is where the PID loop is run.
     * if the loop is running on the rio, it needs to run faster than usual.
     * if the loop is running on the motor controller, it only handles limits, motion profiles.
     * so it can run slower.
     */
    public enum ControllerLocation {
        /**
         * the controller is running on the motor controller
         * that means that the main loop is running slower, and the pid calculations are done on the motor controller.
         * use this for most cases as it reduces the load on the rio, canbus load.
         */
        MOTOR(() -> config.PROFILE_LOOP_HZ),
        /**
         * the controller is running on the rio
         * that means that the main loop is running faster, and the pid calculations are done on the rio.
         * use this if you are using an external sensor that you want to drive directly (for example, turn motor on swerve using a CANcoder).
         * or if you want to have more control over the PID loop.
         */
        RIO(() -> config.PID_LOOP_HZ),;

        /**
         * the frequency of the controller loop in Hz
         * this can change depending on user configuration.
         */
        private final DoubleSupplier hzSupplier;

        /**
         * constructor for the controller location
         *
         * @param hzSupplier the frequency of the controller loop in Hz
         */
        ControllerLocation(DoubleSupplier hzSupplier) {
            this.hzSupplier = hzSupplier;
        }

        /**
         * get the frequency of the controller loop in Hz
         *
         * @return the frequency of the controller loop in Hz
         */
        public double getHZ() {
            return hzSupplier.getAsDouble();
        }

    }
}
