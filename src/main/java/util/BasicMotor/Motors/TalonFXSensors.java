package util.BasicMotor.Motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import util.BasicMotor.LogFrame;

public class TalonFXSensors {
    private final double refreshHZ;

    private final StatusSignal<Temperature> temperatureSignal;
    private final StatusSignal<Current> supplyCurrentSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Double> dutyCycleSignal;

    private final BaseStatusSignal[] statusSignals;

    public TalonFXSensors(TalonFX motor, double refreshHZ) {
        this.refreshHZ = refreshHZ;

        temperatureSignal = motor.getDeviceTemp();
        supplyCurrentSignal = motor.getSupplyCurrent();
        statorCurrentSignal = motor.getStatorCurrent();
        motorVoltageSignal = motor.getMotorVoltage();
        supplyVoltageSignal = motor.getSupplyVoltage();
        dutyCycleSignal = motor.getDutyCycle();

        statusSignals = new StatusSignal[]{
                temperatureSignal,
                supplyCurrentSignal,
                statorCurrentSignal,
                motorVoltageSignal,
                supplyVoltageSignal,
                dutyCycleSignal
        };

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
}
