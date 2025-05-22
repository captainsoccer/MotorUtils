package util.BasicMotor;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class MotorManager extends SubsystemBase {

    private static final double PID_LOOP_HZ = 100;
    private static final double PROFILE_LOOP_HZ = 50;
    private static final double SENSOR_LOOP_HZ = 4;

    private static MotorManager instance;

    private final ArrayList<MotorHandler> motors = new ArrayList<>();

    private MotorManager() {
        // Private constructor to prevent instantiation
    }

    public static MotorManager getInstance() {
        if (instance == null) {
            instance = new MotorManager();
        }
        return instance;
    }

    @Override
    public void periodic() {
        for (MotorHandler motor : motors) {
            var frame = motor.motor.getLatestFrame();
            Logger.processInputs("Motors/" + motor.motorName, frame);
        }
    }

    public void stopSensorLoop() {
        for (MotorHandler motor : motors) {
            motor.sensorLoop.stop();
        }
    }

    public void stopPIDLoop() {
        for (MotorHandler motor : motors) {
            motor.pidLoop.stop();
        }
    }

    public void registerMotor(BasicMotor motor, String name, ControllerLocation location) {
        var handler = new MotorHandler(name, motor);

        motors.add(handler);

        handler.sensorLoop.startPeriodic(1 / SENSOR_LOOP_HZ);
        handler.pidLoop.startPeriodic(1 / location.HZ);
    }

    private static class MotorHandler {
        private final String motorName;
        private final BasicMotor motor;

        private final Notifier pidLoop;
        private final Notifier sensorLoop;

        public MotorHandler(String motorName, BasicMotor motor) {
            this.motorName = motorName;
            this.motor = motor;

            pidLoop = new Notifier(motor::run);

            sensorLoop = new Notifier(motor::updateSensorData);
        }
    }

    public enum ControllerLocation {
        MOTOR(PROFILE_LOOP_HZ),
        RIO(PID_LOOP_HZ);

        public final double HZ;

        ControllerLocation(double hz) {
            this.HZ = hz;
        }
    }
}
