package util.BasicMotor;

import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.MotorManager.ControllerLocation;

public abstract class BasicMotor {
    /**
     * the controller of the motor
     */
    private final Controller controller;

    /**
     * the measurements of the motor
     */
    private Measurements measurements;

    /**
     * the log frame of the motor
     */
    protected final LogFrameAutoLogged logFrame = new LogFrameAutoLogged();

    /**
     * the location of the controller (RIO or motor controller)
     */
    protected final MotorManager.ControllerLocation controllerLocation;

    /**
     * if the PID gains have changed (then it updates the motor controller on the slower thread)
     */
    private boolean hasPIDGainsChanged = false;
    private void setHasPIDGainsChanged() {
        hasPIDGainsChanged = true;
    }
    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);


    //constructors

    /**
     * creates the motor.
     * the motor child needs to register the measurements
     * @param controllerGains the gains of the controller
     * @param name the name of the motor (used for logging)
     * @param controllerLocation the location of the controller (RIO or motor controller)
     */
    public BasicMotor(ControllerGains controllerGains, String name, ControllerLocation controllerLocation) {
        this.controllerLocation = controllerLocation;
        controller = new Controller(controllerGains, this::setHasPIDGainsChanged);

        MotorManager.getInstance().registerMotor(this, name, controllerLocation);
    }

    //getters and setters

    /**
     * sets the measurements of the motor this is used when you want to switch a source of the measurements
     * @param measurements the new measurements
     */
    public void setMeasurements(Measurements measurements) {
        this.measurements = measurements;
    }

    /**
     * gets the measurements of the motor
     * @return the measurements of the motor
     */
    public Measurements getMeasurements() {
        return measurements;
    }

    /**
     * gets the latest measurement of the motor
     * @return the latest measurement of the motor
     */
    public Measurements.Measurement getMeasurement() {
        return measurements.getMeasurement();
    }

    /**
     * gets the controller of the motor
     * @return the controller of the motor
     */
    public Controller getController() {
        return controller;
    }

    /**
     * gets the latest log frame
     * this is used for logging
     * @return the latest log frame
     */
    public LogFrameAutoLogged getLatestFrame() {
        return logFrame;
    }

    /**
     * this runs the main loop for the controller
     * runs pid if needed and updates the measurements
     * this is called on a separate thread
     */
    public void run() {
        //updates the measurements
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
            //if calculating on the rio, then calculate the output in volts and set it
            if (controllerLocation == ControllerLocation.RIO) {
                motorOutput = controller.calculate(measurement, 1 / controllerLocation.HZ);

                reference = motorOutput.totalOutput();
                feedForward = 0;
                outputMode = Controller.RequestType.VOLTAGE;
            }
            //if calculating on the motor controller, then calculate the feedforward and the setpoint and give to the motor
            else {
                var feedForwardOutput = controller.calculateWithOutPID(measurement, 1 / controllerLocation.HZ);

                //adds the pid output from the motor controller (could be outdated by a cycle or two)
                motorOutput = new LogFrame.ControllerFrame(feedForwardOutput, getPIDLatestOutput());

                reference = motorOutput.setpoint();
                feedForward = motorOutput.feedForwardOutput().totalOutput();
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
    /**
     * gets the latest sensor data from the motor
     * @return the latest sensor data
     */
    protected abstract LogFrame.SensorData getSensorData();

    /**
     * gets the pid output of the built-in controller
     * used only when the pid is on the motor controller
     * this returns the stored pid output that should be updated in {@link #getSensorData()} function
     * @return the pid output
     */
    protected abstract LogFrame.PIDOutput getPIDLatestOutput();

}
