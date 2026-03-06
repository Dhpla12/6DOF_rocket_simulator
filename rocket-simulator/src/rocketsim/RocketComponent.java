package rocketsim;

/**
 * Abstract base class for all rocket components.
 * Each component contributes mass and aerodynamic properties.
 */
public abstract class RocketComponent {
    protected double mass;          // kg
    protected double cgPosition;    // distance from nose tip (m)
    protected double cpPosition;    // distance from nose tip (m) – simplified

    public double getMass() { return mass; }
    public double getCG()   { return cgPosition; }
    public double getCP()   { return cpPosition; }
}