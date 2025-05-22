package util.BasicMotor.Motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import util.BasicMotor.LogFrame;
import util.BasicMotor.MotorManager;

import java.util.ArrayList;

public class TalonFXSensors {
    private final double refreshHZ;
    private final MotorManager.ControllerLocation location;

    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Double> dutyCycleSignal;

    private final StatusSignal<Double> totalOutput;
    private final StatusSignal<Double> kpOutput;
    private final StatusSignal<Double> kiOutput;
    private final StatusSignal<Double> kdOutput;

    private final BaseStatusSignal[] statusSignals;

    /**
     * The latest PID output from the motor controller
     * used for logging if the pid controller is on the motor controller
     */
    private LogFrame.PIDOutput latestPIDOutput = new LogFrame.PIDOutput();
    /**
     * Constructor for TalonFX Sensors
     * @param motor the motor to get the sensors from
     * @param refreshHZ the refresh rate of the sensors (how often to update the sensors)
     * @param location the location of the motor (if running on motor controller it will also update the pid output)
     */
    public TalonFXSensors(TalonFX motor, double refreshHZ, MotorManager.ControllerLocation location) {
        this.refreshHZ = refreshHZ;
        this.location = location;

        temperatureSignal = motor.getDeviceTemp();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        motorVoltageSignal = motor.getMotorVoltage();
        supplyVoltageSignal = motor.getSupplyVoltage();
        dutyCycleSignal = motor.getDutyCycle();

        kpOutput = motor.getClosedLoopProportionalOutput();
        kiOutput = motor.getClosedLoopIntegratedOutput();
        kdOutput = motor.getClosedLoopDerivativeOutput();
        totalOutput = motor.getClosedLoopOutput();

        if(location == MotorManager.ControllerLocation.RIO){
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

        dutyCycleSignal.setUpdateFrequency(100);
    }

    public LogFrame.SensorData getSensorData() {
        BaseStatusSignal.waitForAll(1  / (refreshHZ * 4), statusSignals);

        double temperature = temperatureSignal.getValueAsDouble();
        double currentDraw = supplyCurrentSignal.getValueAsDouble();
        double currentOutput = statorCurrentSignal.getValueAsDouble();
        double voltageOutput = motorVoltageSignal.getValueAsDouble();
        double voltageInput = supplyVoltageSignal.getValueAsDouble();
        double powerDraw = currentDraw * voltageInput;
        double powerOutput = currentOutput * voltageOutput;
        double dutyCycle = dutyCycleSignal.getValueAsDouble();

        //updates the latest pid output if the controller is on the motor controller
        //used for logging
        if(location == MotorManager.ControllerLocation.MOTOR){
            double pOutput = kpOutput.getValueAsDouble();
            double iOutput = kiOutput.getValueAsDouble();
            double dOutput = kdOutput.getValueAsDouble();
            double pidOutput = totalOutput.getValueAsDouble();

            latestPIDOutput = new LogFrame.PIDOutput(
                    pOutput,
                    iOutput,
                    dOutput,
                    pidOutput
            );
        }

        return new LogFrame.SensorData(
                temperature,
                currentDraw,
                currentOutput,
                voltageOutput,
                voltageInput,
                powerDraw,
                powerOutput,
                dutyCycle,
                "Not supported by TalonFX" // TODO: Implement fault handling
        );
    }

    public LogFrame.PIDOutput getPIDLatestOutput() {
        return latestPIDOutput;
    }
}
