package rocketsim;

import java.util.ArrayList;
import java.util.List;

/**
 * Model rocket motor with thrust curve (public domain data, e.g., Estes A8-3).
 * Thrust is given as a list of (time, thrust) points.
 * Mass includes propellant; propellant mass is consumed over burn time.
 */
public class Motor extends RocketComponent {
    private double burnTime;       // s
    private double totalImpulse;   // Ns
    private double propellantMass; // kg
    private double dryMass;        // kg
    private List<double[]> thrustCurve; // (time, thrust) in seconds and Newtons

    public Motor(double dryMass, double propellantMass, double burnTime, List<double[]> thrustCurve) {
        this.dryMass = dryMass;
        this.propellantMass = propellantMass;
        this.burnTime = burnTime;
        this.thrustCurve = thrustCurve;
        this.mass = dryMass + propellantMass; // initial mass
        // CG of motor (simple: at its center, will be placed later)
        this.cgPosition = 0.0; // placeholder
        this.cpPosition = Double.POSITIVE_INFINITY; // motor contributes no aerodynamic CP
    }

    /**
     * Returns thrust at time t (seconds) by interpolating the thrust curve.
     */
    public double getThrust(double t) {
        if (t < 0 || t > burnTime) return 0.0;
        // Linear interpolation between given points
        for (int i = 0; i < thrustCurve.size() - 1; i++) {
            double[] p1 = thrustCurve.get(i);
            double[] p2 = thrustCurve.get(i + 1);
            if (t >= p1[0] && t <= p2[0]) {
                double frac = (t - p1[0]) / (p2[0] - p1[0]);
                return p1[1] + frac * (p2[1] - p1[1]);
            }
        }
        return 0.0;
    }

    /**
     * Returns current mass at time t (propellant consumed linearly with impulse).
     * Simplified: mass = dryMass + propellantMass * (1 - t / burnTime)
     */
    public double getMass(double t) {
        if (t < 0) return dryMass + propellantMass;
        if (t > burnTime) return dryMass;
        return dryMass + propellantMass * (1.0 - t / burnTime);
    }

    public double getBurnTime() { return burnTime; }
    public double getDryMass() { return dryMass; }
}