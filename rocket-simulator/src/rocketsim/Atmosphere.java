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

    public static double[] getWind(double altitude) {

    double windX;
    double windY;

    // Simple layered wind model

    if (altitude < 100) {
        windX = 2.0;
        windY = 0.5;
    }
    else if (altitude < 500) {
        windX = 5.0;
        windY = 1.5;
    }
    else {
        windX = 8.0;
        windY = 3.0;
    }

    return new double[]{windX, windY, 0.0};
}
}
