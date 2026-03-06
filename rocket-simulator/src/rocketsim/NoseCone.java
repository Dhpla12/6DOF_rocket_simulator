package rocketsim;

/**
 * Conical nose cone.
 * Mass computed from hollow cone approximation.
 * CP is at the center of pressure of a cone (approx. 2/3 of length from tip).
 */
public class NoseCone extends RocketComponent {
    private double length;
    private double baseDiameter;

    public NoseCone(double length, double baseDiameter, double wallThickness, double materialDensity) {
        this.length = length;
        this.baseDiameter = baseDiameter;
        // Surface area of cone (approx) * thickness * density
        double slant = Math.sqrt(length * length + (baseDiameter / 2.0) * (baseDiameter / 2.0));
        double area = Math.PI * (baseDiameter / 2.0) * slant; // lateral surface area
        this.mass = area * wallThickness * materialDensity;
        // CG of a hollow cone is at 1/3 of length from base? Actually solid cone CG from tip is 3/4 length.
        // For thin wall, CG is about 2/3 from tip? We'll use 2/3 from tip for simplicity.
        this.cgPosition = 0.66 * length; // from tip
        // CP for a cone alone is at 2/3 length from tip (common approximation)
        this.cpPosition = 0.66 * length;
    }

    public double getLength() { return length; }
}