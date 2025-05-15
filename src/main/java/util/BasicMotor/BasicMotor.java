package util.PIDController;

import util.PIDController.Gains.PIDGains;

public abstract class BasicMotor {
    private final Controller controller;
    private boolean hasPIDGainsChanged = false;


    private void setHasPIDGainsChanged(){
        hasPIDGainsChanged = true;
    }

    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);
}
