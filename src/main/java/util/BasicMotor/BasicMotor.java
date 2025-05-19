package util.BasicMotor;

import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.MotorManager.ControllerLocation;

public abstract class BasicMotor {
    private final Controller controller;
    private Measurements measurements;
    private final LogFrameAutoLogged logFrame = new LogFrameAutoLogged();
    private final MotorManager.ControllerLocation controllerLocation;

    private boolean hasPIDGainsChanged = false;


    private void setHasPIDGainsChanged() {
        hasPIDGainsChanged = true;
    }

    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);

    public void setMeasurements(Measurements measurements) {
        this.measurements = measurements;
    }

    public Measurements getMeasurements() {
        return measurements;
    }

    public Controller getController() {
        return controller;
    }

    public LogFrameAutoLogged getLatestFrame() {
        return logFrame;
    }

    public void run() {
        var measurement = measurements.update(1 / controllerLocation.HZ);
        logFrame.measurement = measurement;

        //the controller frame (for logging)
        LogFrame.ControllerFrame motorOutput;
        //the reference to give to the motor (can be any mode)
        double reference;
        //the feedforward (this is used if the controller is on the motor and there is any feedforward calculation)
        double feedForward;
        //the output mode (this is used to set the motor output)
        Controller.RequestType outputMode;

        //if the controller is not using PID, we can just set the reference to the setpoint
        if (!controller.getRequestType().requiresPID()) {
            var request = controller.getRequestType();
            reference = controller.getRequest().goal().position;
            feedForward = 0;

            //if the controller is using voltage control, we need to set the reference to the setpoint
            if(request == Controller.RequestType.VOLTAGE){
                motorOutput = new LogFrame.ControllerFrame(reference, reference, measurement.position(), request);
                outputMode = Controller.RequestType.VOLTAGE;
            }
            //if using precent output, then the total output needs to be multiplied by the input voltage to be in volts
            else{
                //multiplies the setpoint (duty cycle) by the input voltage to get the output voltage
                motorOutput = new LogFrame.ControllerFrame(reference * logFrame.sensorData.voltageInput(),
                        reference, measurement.position(), request);


                outputMode = Controller.RequestType.PRECENT_OUTPUT;
            }
        }
        //if the controller is using PID, we need to calculate the output
        else {
            //if calculating on the rio then calculate the output in volts and set it
            if (controllerLocation == ControllerLocation.RIO) {
                motorOutput = controller.calculate(measurement, 1 / controllerLocation.HZ);

                reference = motorOutput.totalOutput();
                feedForward = 0;
                outputMode = Controller.RequestType.VOLTAGE;
            }
            //if calculating on the motor controller, then calculate the feedforward and the setpoint and give to the motor
            else {
                motorOutput = controller.calculateWithOutPID(measurement, 1 / controllerLocation.HZ);

                reference = motorOutput.setpoint();
                feedForward = motorOutput.totalOutput();
                outputMode = motorOutput.mode();
            }
        }
        //updates the log frame with the motor output
        logFrame.controllerFrame = motorOutput;
        //sets the motor output
        setMotorOutput(reference, feedForward, outputMode);
    }

    protected abstract void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode);

    /**
     * gets all the sensor data from the motor (this runs on a separate slower thread)
     */
    public void updateSensorData() {
        logFrame.sensorData = getSensorData();

        //if the pid has changed, then update the built-in motor pid
        if (hasPIDGainsChanged) {
            hasPIDGainsChanged = false;
            updatePIDGainsToMotor(controller.getControllerGains().getPidGains());
        }
    }


    protected abstract LogFrame.SensorData getSensorData();

    public BasicMotor(ControllerGains controllerGains, int id, String name, ControllerLocation controllerLocation) {
        controller = new Controller(controllerGains, this::setHasPIDGainsChanged);

        initializeMotor(id);
        measurements = initializeMeasurements();

        this.controllerLocation = controllerLocation;

        MotorManager.getInstance().registerMotor(this, name, controllerLocation);
    }

    public BasicMotor(int id, String name) {
        this(new ControllerGains(), id, name, ControllerLocation.MOTOR);
    }

    /**
     * initializes the motor (this is called in the constructor)
     */
    protected abstract void initializeMotor(int id);

    /**
     * initializes the measurements (this is called in the constructor)
     */
    protected abstract Measurements initializeMeasurements();

}
