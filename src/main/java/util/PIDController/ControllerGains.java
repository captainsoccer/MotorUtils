package util.PIDController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllerGains{

    private PIDGains pidGains = new PIDGains();
    private ControllerConstrains controllerConstrains = new ControllerConstrains();
    private ControllerFeedForwards controllerFeedForwards = new ControllerFeedForwards();

    private TrapezoidProfile.Constraints controllerProfile =
            new TrapezoidProfile.Constraints(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);

    public ControllerGains(){}

    public ControllerGains(double k_P, double k_I, double k_D){
        pidGains = new PIDGains(k_P, k_I, k_D);
    }

    public ControllerGains(PIDGains pidGains){
        this.pidGains = pidGains;
    }

    public ControllerGains(ControllerFeedForwards controllerFeedForwards){
        this.controllerFeedForwards = controllerFeedForwards;
    }

    public ControllerGains(PIDGains pidGains, ControllerConstrains controllerConstrains){
        this.pidGains = pidGains;
        this.controllerConstrains = controllerConstrains;
    }

    public ControllerGains(PIDGains pidGains, ControllerFeedForwards controllerFeedForwards){
        this.pidGains = pidGains;
        this.controllerFeedForwards = controllerFeedForwards;
    }

    public ControllerGains(PIDGains pidGains, ControllerConstrains controllerConstrains, ControllerFeedForwards controllerFeedForwards){
        this.pidGains = pidGains;
        this.controllerConstrains = controllerConstrains;
        this.controllerFeedForwards = controllerFeedForwards;
    }

    public PIDGains getPidGains() {
        return pidGains;
    }

    public ControllerConstrains getControllerConstrains() {
        return controllerConstrains;
    }

    public ControllerFeedForwards getControllerFeedForwards() {
        return controllerFeedForwards;
    }

    public void setPidGains(double k_P, double k_I, double k_D) {
        this.pidGains = new PIDGains(k_P, k_I, k_D);
    }

    public void setPidGains(PIDGains pidGains) {
        this.pidGains = pidGains;
    }

    public void setControllerConstrains(ControllerConstrains controllerConstrains) {
        this.controllerConstrains = controllerConstrains;
    }

    public void setControllerFeedForwards(ControllerFeedForwards controllerFeedForwards) {
        this.controllerFeedForwards = controllerFeedForwards;
    }

    public void publishToDashboard(String name){
        SmartDashboard.putData(name + " PID", pidGains);
        ProfiledPIDController
    }
}
