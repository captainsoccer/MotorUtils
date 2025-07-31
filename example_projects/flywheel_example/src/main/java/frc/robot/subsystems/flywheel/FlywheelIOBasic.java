package frc.robot.subsystems.flywheel;

import com.basicMotor.BasicMotor;
import com.basicMotor.controllers.Controller;
import com.basicMotor.motors.talonFX.BasicTalonFX;

public class FlywheelIOBasic implements  FlywheelIO {
    private final BasicMotor motor;

    public FlywheelIOBasic() {
        this.motor = new BasicTalonFX(FlywheelConstants.motorConfig);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.velocityMetersPerSecond = motor.getVelocity();
        inputs.atTarget = motor.atSetpoint() && motor.getController().getControlMode().requiresPID();
    }

    @Override
    public void setTargetVelocity(double targetMetersPerSecond) {
        motor.setControl(targetMetersPerSecond, Controller.ControlMode.VELOCITY);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void setPrecentOutput(double percentOutput) {
        motor.setPrecentOutput(percentOutput);
    }
}
