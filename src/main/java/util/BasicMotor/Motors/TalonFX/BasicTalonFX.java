package util.BasicMotor.Motors.TalonFX;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import util.BasicMotor.BasicMotor;
import util.BasicMotor.Controllers.Controller;
import util.BasicMotor.Gains.ControllerConstrains;
import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.CurrentLimits;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.LogFrame;
import util.BasicMotor.Measurements.Measurements;
import util.BasicMotor.Measurements.MeasurementsCTRE;
import util.BasicMotor.MotorManager;

/**
 * this class is the talonFX implementation of the basic motor class it is used to control the
 * talonFX
 */
public class BasicTalonFX extends BasicMotor {
  /**
   * the talonFX motor controller
   *
   * <p>this is the motor controller used to control the motor
   */
  private final TalonFX motor;
  /**
   * the configuration of the talonFX motor controller
   *
   * <p>this is used to configure the motor controller
   */
  private final TalonFXConfiguration config;

  /**
   * the sensors of the talonFX motor controller
   *
   * <p>this is used to get the sensors of the motor controller
   */
  private final TalonFXSensors sensors;

  /**
   * used to store the default measurements of the motor controller
   */
  private final Measurements defaultMeasurements;

  /** velocity request for the motor controller */
  private final VelocityVoltage velocityRequest =
      new VelocityVoltage(0).withEnableFOC(false).withUpdateFreqHz(0);
  /** position request for the motor controller */
  private final PositionVoltage positionRequest =
      new PositionVoltage(0).withEnableFOC(false).withUpdateFreqHz(0);

  /** voltage request for the motor controller */
  private final VoltageOut voltageRequest = new VoltageOut(0).withUpdateFreqHz(0);

  /** duty cycle request for the motor controller */
  private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withUpdateFreqHz(0);

  /**
   * Constructor for the TalonFX motor controller
   *
   * @param controllerGains the gains for the motor controller
   * @param id the id of the motor controller
   * @param gearRatio the gear ratio of the motor
   * @param name the name of the motor controller
   * @param controllerLocation the location of the motor controller (rio, motor controller)
   */
  public BasicTalonFX(
      ControllerGains controllerGains,
      int id,
      double gearRatio,
      String name,
      MotorManager.ControllerLocation controllerLocation) {

    super(controllerGains, name, controllerLocation);

    motor = new TalonFX(id);
    config = new TalonFXConfiguration();

    applyConfig();

    defaultMeasurements = new MeasurementsCTRE(
            motor.getPosition(),
            motor.getVelocity(),
            motor.getAcceleration(),
            controllerLocation.HZ,
            gearRatio);

    sensors = new TalonFXSensors(motor, controllerLocation.HZ, controllerLocation);

    motor.optimizeBusUtilization();

    updatePIDGainsToMotor(controllerGains.getPidGains());
    updateConstraints(controllerGains.getControllerConstrains());
  }

  @Override
  protected void updatePIDGainsToMotor(PIDGains pidGains) {
    config.Slot0.kP = pidGains.getK_P();
    config.Slot0.kI = pidGains.getK_I();
    config.Slot0.kD = pidGains.getK_D();

    if(controllerLocation == MotorManager.ControllerLocation.MOTOR){
      //changes made in phoenix 6 api https://v6.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/feature-replacements-guide.html#integral-zone-and-max-integral-accumulator

      if(pidGains.getI_MaxAccum() != Double.POSITIVE_INFINITY)
        DriverStation.reportWarning( name + " does not need i max accum when running on motor therefor not used (TalonFX check phoenix 6 docs)", false);

      if(pidGains.getTolerance() != 0)
        DriverStation.reportWarning( name + " does not need tolerance when running on motor therefor not used (TalonFX check phoenix 6 docs)", false);

      if(pidGains.getI_Zone() != Double.POSITIVE_INFINITY)
        DriverStation.reportWarning( name + " does not need i zone when running on motor therefor not used (TalonFX check phoenix 6 docs)", false);
    }

    applyConfig();
  }

  @Override
  protected void updateConstraints(ControllerConstrains constraints) {
    // sets the max voltage to the max motor output
    config.Voltage.PeakForwardVoltage = constraints.getMaxMotorOutput();
    config.Voltage.PeakReverseVoltage = constraints.getMinMotorOutput();

    // sets the max duty cycle to the max motor output (same as voltage)
    config.MotorOutput.PeakForwardDutyCycle =
        constraints.getMaxMotorOutput() / MotorManager.motorIdleVoltage;
    config.MotorOutput.PeakReverseDutyCycle =
        constraints.getMinMotorOutput() / MotorManager.motorIdleVoltage;

    // sets the voltage deadband to the voltage deadband
    config.MotorOutput.DutyCycleNeutralDeadband =
        constraints.getVoltageDeadband() / MotorManager.motorIdleVoltage;

    // sets continuous wrap to false (it is calculated on the rio if needed)
    config.ClosedLoopGeneral.ContinuousWrap = false;

    // checks if it needs to apply soft limits
    if (constraints.getConstraintType() == ControllerConstrains.ConstraintType.LIMITED) {
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constraints.getMaxValue();
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constraints.getMinValue();
    } else {
      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    }

    // applies the config to the motor
    applyConfig();
  }

  @Override
  protected Measurements getDefaultMeasurements() {
    return defaultMeasurements;
  }

  @Override
  protected void setMotorOutput(double setpoint, double feedForward, Controller.RequestType mode) {
    if(mode == Controller.RequestType.STOP){
      motor.stopMotor();
      return;
    }

    StatusCode error = switch (mode) {
      case POSITION, PROFILED_POSITION -> motor.setControl(
          positionRequest.withPosition(setpoint).withFeedForward(feedForward));

      case VELOCITY -> motor.setControl(
          velocityRequest.withVelocity(setpoint).withFeedForward(feedForward));

      case VOLTAGE -> motor.setControl(voltageRequest.withOutput(setpoint));

      case PRECENT_OUTPUT -> motor.setControl(dutyCycleRequest.withOutput(setpoint));

      default -> StatusCode.OK;
    };

    if (error != StatusCode.OK) {
      DriverStation.reportError(
          "Failed to set motor output for motor: " + super.name + " Error: " + error.name(), false);
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

  @Override
  public void setCurrentLimits(CurrentLimits currentLimits) {
    var currentConfig = config.CurrentLimits;

    currentConfig.SupplyCurrentLimit = currentLimits.getSupplyCurrentLimit();
    currentConfig.StatorCurrentLimit = currentLimits.getStatorCurrentLimit();
    currentConfig.SupplyCurrentLowerLimit = currentLimits.getSupplyLowerLimit();
    currentConfig.SupplyCurrentLowerTime = currentLimits.getSupplyLowerTime();

    currentConfig.SupplyCurrentLimitEnable = currentLimits.getStatorCurrentLimit() != 0;
    currentConfig.StatorCurrentLimitEnable = currentLimits.getStatorCurrentLimit() != 0;

    applyConfig();
  }

  @Override
  public void setIdleMode(IdleMode mode) {
    config.MotorOutput.NeutralMode =
        switch (mode) {
          case COAST -> NeutralModeValue.Coast;
          case BRAKE -> NeutralModeValue.Brake;
        };

    applyConfig();
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    config.MotorOutput.Inverted =
            inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    applyConfig();
  }

  @Override
  protected void setMotorPosition(double position) {
    motor.setPosition(position);
  }

  @Override
  protected void stopRecordingMeasurements() {
    if(getMeasurements() instanceof MeasurementsCTRE measurements) {
      measurements.setUpdateFrequency(0);
    }
  }

  @Override
  protected void startRecordingMeasurements(double HZ) {
    if (getMeasurements() instanceof MeasurementsCTRE measurements) {
      measurements.setUpdateFrequency(HZ);
    }
  }

  @Override
  protected void setMotorFollow(BasicMotor master, boolean inverted) {
    BasicTalonFX motor = (BasicTalonFX) master;

    Follower follower = new Follower(motor.motor.getDeviceID(), inverted);

    this.motor.setControl(follower);
  }

  @Override
  protected void stopMotorFollow() {
    motor.stopMotor();
  }

  /**
   * this enables or disables the FOC on the motor foc is only supported with phoenix pro (sad) if
   * true, but no foc is supported, it will be ignored
   *
   * @param enable true to enable foc, false to disable
   */
  public void enableFOC(boolean enable) {
    velocityRequest.EnableFOC = enable;
    positionRequest.EnableFOC = enable;
    voltageRequest.EnableFOC = enable;
    dutyCycleRequest.EnableFOC = enable;
  }

  /**
   * applies the configuration to the motor controller
   */
  private void applyConfig() {
    var error = motor.getConfigurator().apply(config);

    if(error != StatusCode.OK){
      DriverStation.reportError(
            "Failed to apply config to motor: " + super.name + " Error: " + error.name(), false
      );
    }
  }
}
