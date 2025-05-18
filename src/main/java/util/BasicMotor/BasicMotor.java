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
    var measurement = measurements.update(0.02);
    logFrame.measurement = measurement;

    LogFrame.ControllerFrame motorOutput;
    if(controllerLocation == ControllerLocation.RIO){
      motorOutput = controller.calculate(measurement, 1 / controllerLocation.HZ);
      setMotorVoltage(motorOutput.totalOutput());
    }
    else{
      motorOutput = controller.calculateWithOutPID(measurement, 1 / controllerLocation.HZ);
      setMotorOutput(motorOutput.setpoint(), motorOutput.totalOutput(), motorOutput.mode());
    }
    logFrame.controllerFrame = motorOutput;

  }

  public void updateSensorData() {
    logFrame.sensorData = getSensorData();

    if (hasPIDGainsChanged) {
      hasPIDGainsChanged = false;
      updatePIDGainsToMotor(controller.getControllerGains().getPidGains());
    }
  }

  protected abstract void setMotorVoltage(double voltage);
  protected abstract void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode);

  protected abstract LogFrame.SensorData getSensorData();

  public BasicMotor(ControllerGains controllerGains, String name, ControllerLocation controllerLocation) {
    controller = new Controller(controllerGains, this::setHasPIDGainsChanged);

    initializeMotor();
    measurements = initializeMeasurements();

    this.controllerLocation = controllerLocation;

    MotorManager.getInstance().registerMotor(this, name, controllerLocation);
  }

  public BasicMotor(String name) {
    this(new ControllerGains(), name, ControllerLocation.MOTOR);
  }

  protected abstract void initializeMotor();

  protected abstract Measurements initializeMeasurements();

}
