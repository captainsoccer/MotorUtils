package util.BasicMotor;

import util.BasicMotor.Gains.ControllerGains;
import util.BasicMotor.Gains.PIDGains;
import util.BasicMotor.Measurements.Measurements;

public abstract class BasicMotor {
    private final Controller controller;
    private Measurements measurements;

    private boolean hasPIDGainsChanged = false;


    private void setHasPIDGainsChanged(){
        hasPIDGainsChanged = true;
    }

    protected abstract void updatePIDGainsToMotor(PIDGains pidGains);

    public void setMeasurements(Measurements measurements){
        this.measurements = measurements;
    }

    public Measurements getMeasurements(){
        return measurements;
    }

    public Controller getController(){
        return controller;
    }

    public LogFrame getLatestFrame(){
        return controller.getLatestFrame();
    }

    public void run(){

    }

    public BasicMotor(ControllerGains controllerGains){
        controller = new Controller(controllerGains, this::setHasPIDGainsChanged);

        initializeMotor();
        measurements = initializeMeasurements();
    }

    public BasicMotor(){
        this(new ControllerGains());
    }

    protected abstract void initializeMotor();
    protected abstract Measurements initializeMeasurements();
}
