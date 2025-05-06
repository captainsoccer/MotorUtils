package util.MarinersController;

import java.util.function.Supplier;

/**
 * A class to measure the position, velocity, and acceleration of a system
 * used in conjunction with the {@link MarinersController} class
 * can calculate the velocity and acceleration from the position
 * or can use the built-in measurements of the motor controller
 */
public class MarinersMeasurements {
    /**
     * the supplier for the position of the system
     */
    private Supplier<Double> position;

    /**
     * the supplier for the velocity of the system
     * (can be calculated from the position)
     */
    private Supplier<Double> velocity;

    /**
     * the supplier for the acceleration of the system
     * (can be calculated from the velocity)
     */
    private Supplier<Double> acceleration;


    /**
     * the current position of the system (after gear ratio)
     * update when {@link #update(double)} is called
     */
    private double currentPosition;

    /**
     * the current velocity of the system (after gear ratio)
     * update when {@link #update(double)} is called
     */
    private double currentVelocity;

    /**
     * the current acceleration of the system (after gear ratio)
     * update when {@link #update(double)} is called
     */
    private double currentAcceleration;

    /**
     * the gear ratio of the system
     * each measurement is divided by this value
     */
    private double gearRatio;

    /**
     * the time in seconds between the last update and the current update
     */
    private double dt;


    /**
     * update the measurements of the system
     * NEEDS TO BE CALLED PERIODICALLY
     * @param dt the time in seconds between the last update and the current update
     */
    public void update(double dt){
        this.dt = dt;

        currentPosition = position.get() / gearRatio;
        currentVelocity = velocity.get() / gearRatio;
        currentAcceleration = acceleration.get() / gearRatio;
    }

    /**
     * gets the current position of the system
     * @return the current position of the system
     */
    public double getPosition(){
        return currentPosition;
    }

    /**
     * gets the current velocity of the system
     * @return the current velocity of the system
     */
    public double getVelocity(){
        return currentVelocity;
    }

    /**
     * gets the current acceleration of the system
     * @return the current acceleration of the system
     */
    public double getAcceleration(){
        return currentAcceleration;
    }

    /**
     * gets the gear ratio of the system
     * @return the value the original measurements are divided by
     */
    public double getGearRatio(){
        return gearRatio;
    }

    /**
     * sets the supplier for the position of the system
     * @param position the supplier for the position of the system (before gear ratio)
     */
    public void setPositionSupplier(Supplier<Double> position){
        this.position = position;
    }

    /**
     * sets the supplier for the velocity of the system
     * @param velocity the supplier for the velocity of the system (before gear ratio)
     */
    public void setVelocitySupplier(Supplier<Double> velocity){
        this.velocity = velocity;
    }

    /**
     * sets the supplier for the acceleration of the system
     * @param acceleration the supplier for the acceleration of the system (before gear ratio)
     */
    public void setAccelerationSupplier(Supplier<Double> acceleration){
        this.acceleration = acceleration;
    }

    /**
     * sets the gear ratio of the system
     * @param gearRatio the value the original measurements are divided by
     */
    public void setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
    }

    /**
     * creates a new MarinersMeasurements object
     * @param position the supplier for the position of the system (before gear ratio)
     * @param velocity the supplier for the velocity of the system (before gear ratio)
     * @param acceleration the supplier for the acceleration of the system (before gear ratio)
     * @param gearRatio the value the original measurements are divided by
     */
    public MarinersMeasurements(Supplier<Double> position, Supplier<Double> velocity, Supplier<Double> acceleration, double gearRatio){
        this.gearRatio = gearRatio;

        setPositionSupplier(position);
        setVelocitySupplier(velocity);
        setAccelerationSupplier(acceleration);
    }

    /**
     * creates a new MarinersMeasurements object
     * @param position the supplier for the position of the system (before gear ratio)
     * @param gearRatio the value the original measurements are divided by
     */
    public MarinersMeasurements(Supplier<Double> position, double gearRatio){
        this.gearRatio = gearRatio;

        setPositionSupplier(position);

        createVelocitySupplier(position);
        createAccelerationSupplier(velocity);
    }

    /**
     * creates a new MarinersMeasurements object
     * @param position the supplier for the position of the system (before gear ratio)
     * @param velocity the supplier for the velocity of the system (before gear ratio)
     * @param gearRatio the value the original measurements are divided by
     */
    public MarinersMeasurements(Supplier<Double> position, Supplier<Double> velocity, double gearRatio){
        this.gearRatio = gearRatio;

        setPositionSupplier(position);
        setVelocitySupplier(velocity);

        createAccelerationSupplier(velocity);
    }

    /**
     * create the velocity supplier from the position supplier
     * @param position the supplier for the position of the system (before gear ratio)
     */
    private void createVelocitySupplier(Supplier<Double> position){
        double[] lastPosition = {position.get()};

        velocity = () -> {
            double currentPosition = position.get();

            double velocity = (currentPosition - lastPosition[0]) / dt;

            lastPosition[0] = currentPosition;

            return velocity;
        };
    }

    /**
     * create the acceleration supplier from the velocity supplier
     * @param velocity the supplier for the velocity of the system (before gear ratio)
     */
    private void createAccelerationSupplier(Supplier<Double> velocity){
        double[] lastVelocity = {velocity.get()};

        acceleration = () -> {
            double currentVelocity = velocity.get();

            double acceleration = (currentVelocity - lastVelocity[0]) / dt;

            lastVelocity[0] = currentVelocity;

            return acceleration;
        };
    }
}
