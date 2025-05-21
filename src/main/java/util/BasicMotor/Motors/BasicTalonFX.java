package util.BasicMotor.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.MeasurementsCTRE;
import util.BasicMotor.MotorManager;

public class BasicTalonFX extends BasicMotor {
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final TalonFXSensors sensors;

    public BasicTalonFX(ControllerGains controllerGains, int id, double gearRatio, String name, MotorManager.ControllerLocation controllerLocation) {
        super(controllerGains, name, controllerLocation);

        motor = new TalonFX(id);
        config = new TalonFXConfiguration();

        setMeasurements(new MeasurementsCTRE(
                motor.getPosition(),
                motor.getVelocity(),
                motor.getAcceleration(),
                controllerLocation.HZ,
                gearRatio
        ));
        sensors = new TalonFXSensors(motor, controllerLocation.HZ);

        motor.optimizeBusUtilization();

        updatePIDGainsToMotor(controllerGains.getPidGains());
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        var pidConfig = pidGains.convertToDutyCycle();

        config.MotorOutput.PeakForwardDutyCycle = pidConfig.getMaxOutput();
        config.MotorOutput.PeakReverseDutyCycle = pidConfig.getMinOutput();
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {

    }

    @Override
    protected LogFrame.SensorData getSensorData() {
        return sensors.getSensorData();
    }
}
