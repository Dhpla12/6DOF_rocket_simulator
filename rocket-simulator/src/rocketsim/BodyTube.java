package rocketsim;

/**
 * Cylindrical body tube.
 * Mass computed from material density and dimensions.
 * CP is at the geometric center (for subsonic flow, simplified).
 */
public class BodyTube extends RocketComponent {
    private double length;   // m
    private double diameter; // m
    private double wallThickness; // m

    public BodyTube(double length, double diameter, double wallThickness, double materialDensity) {
        this.length = length;
        this.diameter = diameter;
        this.wallThickness = wallThickness;
        // mass = volume * density
        double outerRadius = diameter / 2.0;
        double innerRadius = outerRadius - wallThickness;
        double volume = Math.PI * (outerRadius * outerRadius - innerRadius * innerRadius) * length;
        this.mass = volume * materialDensity;
        // CG is at the center of the tube (distance from nose tip depends on placement)
        this.cgPosition = length / 2.0; // will be adjusted later relative to nose tip
        // CP for a cylinder alone is approximately at its center (for subsonic flow)
        this.cpPosition = length / 2.0;
    }

    public double getLength() { return length; }
    public double getDiameter() { return diameter; }
}