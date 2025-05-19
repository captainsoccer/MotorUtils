package util.BasicMotor;

import org.littletonrobotics.junction.AutoLog;
import util.BasicMotor.Measurements.Measurements;

@AutoLog
public class LogFrame {
    public ControllerFrame controllerFrame;
    public SensorData sensorData;
    public Measurements.Measurement measurement;

    public record SensorData(
            double temperature,
            double currentDraw,
            double currentOutput,
            double voltageOutput,
            double voltageInput,
            double powerDraw,
            double powerOutput,
            double dutyCycle,
            String faults) {
    }


    public record PIDOutput(
            double pOutput,
            double iOutput,
            double dOutput,
            double totalOutput
    ) {
        public PIDOutput() {
            this(0, 0, 0, 0);
        }
    }

    public record FeedForwardOutput(
            double simpleFeedForward,
            double frictionFeedForward,
            double kV,
            double calculatedFeedForward,
            double totalOutput
    ) {
        public FeedForwardOutput() {
            this(0, 0, 0, 0, 0);
        }

        public FeedForwardOutput(double simpleFeedForward, double frictionFeedForward, double kV, double calculatedFeedForward) {
            this(
                    simpleFeedForward,
                    frictionFeedForward,
                    kV,
                    calculatedFeedForward,
                    simpleFeedForward + frictionFeedForward + kV + calculatedFeedForward);
        }

    }

    public record ControllerFrame(
            double totalOutput,
            PIDOutput pidOutput,
            FeedForwardOutput feedForwardOutput,
            double setpoint,
            double measurement,
            double error,
            double goal,
            Controller.RequestType mode
    ) {
        public ControllerFrame() {
            this(
                    0,
                    new PIDOutput(),
                    new FeedForwardOutput(),
                    0,
                    0,
                    0,
                    0,
                    Controller.RequestType.STOP);

        }

        public ControllerFrame(ControllerFrame frame, PIDOutput output) {
            this(
                    frame.totalOutput + output.totalOutput,
                    output,
                    frame.feedForwardOutput,
                    frame.setpoint,
                    frame.measurement,
                    frame.error,
                    frame.goal,
                    frame.mode);
        }

        public ControllerFrame(double output, double wanted, double measurement, Controller.RequestType mode) {
            this(
                    output,
                    new PIDOutput(),
                    new FeedForwardOutput(),
                    wanted,
                    measurement,
                    0,
                    0,
                    mode);
        }
    }
}
