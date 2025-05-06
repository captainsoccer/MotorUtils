package util.PIDController;

public class PIDGains {
    /**
     * the PID elements of the gains
     */
    private final double k_P, k_I, K_D;

    /**
     * the integrator modifies
     */
    private final double i_Zone, i_maxAccum;

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

    @Override
    public boolean equals(Object obj) {
        return obj instanceof PIDGains && ((PIDGains) obj).getK_P() == this.k_P
                && ((PIDGains) obj).getK_I() == this.k_I
                && ((PIDGains) obj).getK_D() == this.K_D
                && ((PIDGains) obj).getI_Zone() == this.i_Zone
                && ((PIDGains) obj).getI_MaxAccum() == this.i_maxAccum;
    }
}
