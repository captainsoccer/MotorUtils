package util.BasicMotor.Motors;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.MeasurementsCTRE;
import util.BasicMotor.MotorManager;

public class BasicTalonFX extends BasicMotor {
    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final TalonFXSensors sensors;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withEnableFOC(false);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(false);
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0);

    public BasicTalonFX(
            ControllerGains controllerGains,
            int id,
            double gearRatio,
            String name,
            MotorManager.ControllerLocation controllerLocation) {
        super(controllerGains, name, controllerLocation);

        motor = new TalonFX(id);
        config = new TalonFXConfiguration();

        setMeasurements(
                new MeasurementsCTRE(
                        motor.getPosition(),
                        motor.getVelocity(),
                        motor.getAcceleration(),
                        controllerLocation.HZ,
                        gearRatio));
        sensors = new TalonFXSensors(motor, controllerLocation.HZ, controllerLocation);

        motor.optimizeBusUtilization();

        updatePIDGainsToMotor(controllerGains.getPidGains());
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        config.Slot0.kP = pidGains.getK_P();
        config.Slot0.kI = pidGains.getK_I();
        config.Slot0.kD = pidGains.getK_D();

        motor.getConfigurator().apply(config);
    }

    private void updateConstraints(ControllerConstrains constraints) {
        config.Voltage.PeakForwardVoltage = constraints.getMaxMotorOutput();
        config.Voltage.PeakReverseVoltage = constraints.getMinMotorOutput();

        config.MotorOutput.PeakForwardDutyCycle = constraints.getMaxMotorOutput() / MotorManager.motorIdleVoltage;
        config.MotorOutput.PeakReverseDutyCycle = constraints.getMinMotorOutput() / MotorManager.motorIdleVoltage;
        config.ClosedLoopGeneral.ContinuousWrap = false;

        if (constraints.getConstraintType() == ControllerConstrains.ConstraintType.LIMITED) {
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constraints.getMaxValue();
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constraints.getMinValue();
        }
        else{
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
        }

        motor.getConfigurator().apply(config);
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
        switch (mode) {
            case STOP -> motor.stopMotor();

            case POSITION, PROFILED_POSITION ->
                    motor.setControl(positionRequest.withPosition(setpoint).withFeedForward(feedForward));

            case VELOCITY -> motor.setControl(velocityRequest.withVelocity(setpoint).withFeedForward(feedForward));

            case VOLTAGE -> motor.setControl(voltageRequest.withOutput(setpoint));

            case PRECENT_OUTPUT -> motor.setControl(dutyCycleRequest.withOutput(setpoint));

        }
    }

    @Override
    protected void stopMotorOutput() {
        motor.stopMotor();
    }

    @Override
    protected LogFrame.SensorData getSensorData() {
        return sensors.getSensorData();
    }

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return sensors.getPIDLatestOutput();
    }

    /**
     * this enables or disables the FOC on the motor
     * foc is only supported with phoenix pro (sad)
     * if true, but no foc is supported, it will be ignored
     * @param enable true to enable foc, false to disable
     */
    public void enableFOC(boolean enable) {
        velocityRequest.EnableFOC = enable;
        positionRequest.EnableFOC = enable;
    }
}
