package util.BasicMotor.Motors.Simulation;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.CurrentLimits.CurrentLimits;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.MotorManager;

public abstract class BasicSimSystem extends BasicMotor {
    protected final LinearSystemSim<N2, N1, N2> system;

    private double voltageOutput = 0.0;

    public BasicSimSystem(LinearSystemSim<N2, N1, N2> system, String name, ControllerGains gains) {
        super(gains, name, MotorManager.ControllerLocation.RIO);
        this.system = system;
    }

    @Override
    protected void updatePIDGainsToMotor(PIDGains pidGains) {
        //does nothing, as this is a simulation system
    }

    @Override
    protected void updateConstraints(ControllerConstrains constraints) {
        // does nothing, as this is a simulation system
    }

    @Override
    public void setCurrentLimits(CurrentLimits currentLimits) {
        // does nothing, as this is a simulation system
    }

    @Override
    public void setIdleMode(IdleMode mode) {
        // does nothing, as this is a simulation system
    }

    @Override
    public void setMotorInverted(boolean inverted) {
        // does nothing, as this is a simulation system
    }

    @Override
    protected void stopRecordingMeasurements() {
        // does nothing, as this is a simulation system
    }

    @Override
    protected void startRecordingMeasurements(double HZ) {
        // does nothing, as this is a simulation system
    }

    @Override
    protected void setMotorFollow(BasicMotor master, boolean inverted) {
        // does nothing, as this is a simulation system
    }

    @Override
    protected void stopMotorFollow() {
        // does nothing, as this is a simulation system
    }

    @Override
    protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
        if(mode.requiresPID()) throw new IllegalArgumentException("Simulation system does not support PID mode.");

        double output = switch(mode){
            case STOP -> 0;
            case PRECENT_OUTPUT -> setpoint * RobotController.getBatteryVoltage();
            default -> setpoint;
        };

        setOutput(output);
    }

    private void setOutput(double output) {
        voltageOutput = output;
        system.setInput(voltageOutput);
    }

    @Override
    protected void stopMotorOutput() {
        setOutput(0);
    }

    @Override
    protected LogFrame.SensorData getSensorData() {
        double voltageInput = RobotController.getBatteryVoltage();
        double voltageOutput = this.voltageOutput;

        double currentDraw = getCurrentDraw();

        double powerDraw = voltageInput * currentDraw; //also power output because this is a simulation system

        double currentOutput = powerDraw / voltageOutput;

        double temp = 0; //no temperature in simulation

        double dutyCycle = voltageOutput / voltageInput; //duty cycle is the ratio of output to input voltage

        return new LogFrame.SensorData(
                temp,
                voltageInput,
                voltageOutput,
                currentDraw,
                currentOutput,
                powerDraw,
                powerDraw,
                dutyCycle,
                ""
        );
    }

    protected abstract double getCurrentDraw();

    @Override
    protected LogFrame.PIDOutput getPIDLatestOutput() {
        return LogFrame.PIDOutput.EMPTY;
    }
}
