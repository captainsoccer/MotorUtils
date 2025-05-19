package util.BasicMotor;

import edu.wpi.first.math.MathUtil;
import util.BasicMotor.Gains.PIDGains;

public class BasicPIDController {
    private PIDGains gains;

    private double lastError = 0;

    private double integral;

    public BasicPIDController(PIDGains gains) {
        this.gains = gains;
    }

    /**
     * calculates the PID output based on the setpoint and measurement
     * @param setpoint the target of the PID controller
     * @param measurement the current measurement of the PID controller
     * @param dt the time since the last calculation
     * @return the PID output of the PID controller
     */
    public LogFrame.PIDOutput calculate(double setpoint, double measurement, double dt){
        double error = setpoint - measurement;

        // Calculate the derivative of the error
        double derivative = (error - lastError) / dt;
        lastError = error;

        // checks if the error is within the I_Zone
        if(Math.abs(error) > gains.getI_Zone()){
            integral = 0;
        }
        // If the error is within the I_Zone, accumulate the integral and clamp it
        else if(gains.getK_I() != 0){
            integral += MathUtil.clamp(
                            integral + error * dt,
                            -gains.getI_MaxAccum() / gains.getK_I(),
                            gains.getI_MaxAccum() / gains.getK_I());
        }

        double pOutput = gains.getK_P() * error;
        double iOutput = gains.getK_I() * integral;
        double dOutput = gains.getK_D() * derivative;

        // Calculate the total output
        double totalOutput = pOutput + iOutput + dOutput;

        // If the error is within the tolerance, set the output to 0
        if(Math.abs(error) <= gains.getTolerance()){
            totalOutput = 0;
        }
        // clamp the output to the min and max output
        else{
            totalOutput = MathUtil.clamp(totalOutput, gains.getMinOutput(), gains.getMaxOutput());
        }

        // create the PID output
        return new LogFrame.PIDOutput(
                pOutput,
                iOutput,
                dOutput,
                totalOutput
        );
    }

    /**
     * resets the PID controller (sets the last error and integral to 0)
     */
    public void reset(){
        lastError = 0;
        integral = 0;
    }

    /**
     * sets the gains of the PID controller
     * @param gains the new gains of the PID controller
     */
    public void setGains(PIDGains gains){
        this.gains = gains;
    }
}
