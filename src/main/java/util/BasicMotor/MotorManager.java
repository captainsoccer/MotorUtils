package util.BasicMotor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

public class MotorManager extends SubsystemBase {
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
            motor.motor.getLatestFrame();
        }
    }

    public void registerMotor(BasicMotor motor, String name) {
        motors.add(new MotorHandler(name, motor));
    }

    private static class MotorHandler{
        private final String motorName;
        private final BasicMotor motor;

        public MotorHandler(String motorName, BasicMotor motor) {
            this.motorName = motorName;
            this.motor = motor;
        }

    }
}
