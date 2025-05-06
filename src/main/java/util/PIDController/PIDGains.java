package util.PIDController;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class PIDGains{
    /**
     * the PID elements of the gains
     */
    private double k_P, k_I, K_D;

    /**
     * the integrator modifies
     */
    private double i_Zone, i_maxAccum;

    public PIDGains(double k_p, double k_I, double k_D, double i_Zone, double i_maxAccum){
        this.k_P = k_p;
        this.k_I = k_I;
        this.K_D = k_D;

        this.i_Zone = i_Zone;
        this.i_maxAccum = i_maxAccum;
    }

    public PIDGains(double k_P, double k_I, double k_D){
        this(k_P, k_I, k_D, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    /**
     * creates an empty PID gains object (no k_P, no k_I, no k_D)
     */
    public PIDGains(){
        this(0, 0, 0);
    }

    public double getK_P() {
        return k_P;
    }

    public double getK_I() {
        return k_I;
    }

    public double getK_D() {
        return K_D;
    }

    public double getI_Zone() {
        return i_Zone;
    }

    public double getI_MaxAccum() {
        return i_maxAccum;
    }

    private void setK_P(double k_P) {
        this.k_P = k_P;
    }

    private void setK_I(double k_I) {
        this.k_I = k_I;
    }

    private void setK_D(double k_D) {
        this.K_D = k_D;
    }

    private void setI_Zone(double i_Zone) {
        this.i_Zone = i_Zone;
    }

    private void setI_MaxAccum(double i_maxAccum) {
        this.i_maxAccum = i_maxAccum;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof PIDGains && ((PIDGains) obj).getK_P() == this.k_P
                && ((PIDGains) obj).getK_I() == this.k_I
                && ((PIDGains) obj).getK_D() == this.K_D
                && ((PIDGains) obj).getI_Zone() == this.i_Zone
                && ((PIDGains) obj).getI_MaxAccum() == this.i_maxAccum;
    }

    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("p", this::getK_P, this::setK_P);
        builder.addDoubleProperty("i", this::getK_I, this::setK_I);
        builder.addDoubleProperty("d", this::getK_D, this::setK_D);
        builder.addDoubleProperty("izone", this::getI_Zone, this::setI_Zone);
    }
}
