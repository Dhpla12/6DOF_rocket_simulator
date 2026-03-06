package rocketsim;

/**
 * Simple atmosphere model: density as function of altitude.
 * Uses US Standard Atmosphere approximation up to 11 km.
 */
public class Atmosphere {
    private static final double RHO0 = 1.225; // kg/m³ at sea level
    private static final double H_SCALE = 8500.0; // scale height (m) – simplified

    public static double getDensity(double altitude) {
        return RHO0 * Math.exp(-altitude / H_SCALE);
    }

    // Speed of sound (m/s) – simple constant for subsonic
    public static double getSpeedOfSound(double altitude) {
        return 340.0; // approximate
    }
}