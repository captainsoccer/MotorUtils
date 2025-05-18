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

    public LogFrame.PIDOutput calculate(double setpoint, double measurement, double dt){
        double error = setpoint - measurement;

        double derivative = (error - lastError) / dt;
        lastError = error;

        if(Math.abs(error) > gains.getI_Zone()){
            integral = 0;
        }
        else if(gains.getK_I() != 0){
            integral += MathUtil.clamp(
                            integral + error * dt,
                            -gains.getI_MaxAccum() / gains.getK_I(),
                            gains.getI_MaxAccum() / gains.getK_I());
        }

        double pOutput = gains.getK_P() * error;
        double iOutput = gains.getK_I() * integral;
        double dOutput = gains.getK_D() * derivative;

        double totalOutput = pOutput + iOutput + dOutput;

        if(Math.abs(error) <= gains.getTolerance()){
            totalOutput = 0;
        }
        else{
            totalOutput = MathUtil.clamp(totalOutput, gains.getMinOutput(), gains.getMaxOutput());
        }

        return new LogFrame.PIDOutput(
                pOutput,
                iOutput,
                dOutput,
                totalOutput
        );
    }

    public void reset(){
        lastError = 0;
        integral = 0;
    }

    public void setGains(PIDGains gains){
        this.gains = gains;
    }
}
