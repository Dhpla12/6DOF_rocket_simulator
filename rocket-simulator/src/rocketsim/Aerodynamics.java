package rocketsim;

/**
 * Computes aerodynamic forces and moments.
 * Uses simplified models: drag and normal force.
 */
public class Aerodynamics {

    /**
     * Computes drag force magnitude (N).
     * Drag = 0.5 * rho * v^2 * Cd * A_ref
     */
    public static double dragForce(double rho, double speed, double cd, double area) {
        return 0.5 * rho * speed * speed * cd * area;
    }

    /**
     * Computes normal force (lift) magnitude (N) perpendicular to velocity.
     * Simplified: normal force = 0.5 * rho * v^2 * CN * A_ref, where CN ~ sin(2*alpha)
     */
    public static double normalForce(double rho, double speed, double alpha, double cnAlpha, double area) {
        // alpha in radians
        double cn = cnAlpha * Math.sin(2.0 * alpha); // very simple
        return 0.5 * rho * speed * speed * cn * area;
    }

    /**
     * Estimate drag coefficient for subsonic speeds.
     * Very rough: Cd ~ 0.5 for model rocket.
     */
    public static double estimateCd() {
        return 0.5;
    }

    /**
     * Estimate normal force coefficient slope (CN_alpha) per radian.
     * Typical value ~ 2.0 for fins.
     */
    public static double estimateCnAlpha() {
        return 2.0;
    }

    /**
     * Reference area based on body diameter.
     */
    public static double referenceArea(double diameter) {
        return Math.PI * diameter * diameter / 4.0;
    }
}