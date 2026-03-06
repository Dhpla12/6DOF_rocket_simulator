package rocketsim;

/**
 * A set of fins (typically 3 or 4).
 * Mass estimated from planform area.
 * CP is at the quarter chord of the mean aerodynamic chord (simplified).
 */
public class FinSet extends RocketComponent {
    private int finCount;
    private double rootChord;   // m
    private double tipChord;    // m
    private double semiSpan;    // m
    private double sweep;       // m (simplified)
    private double thickness;   // m
    private double materialDensity;

    public FinSet(int finCount, double rootChord, double tipChord, double semiSpan,
                  double sweep, double thickness, double materialDensity) {
        this.finCount = finCount;
        this.rootChord = rootChord;
        this.tipChord = tipChord;
        this.semiSpan = semiSpan;
        this.sweep = sweep;
        this.thickness = thickness;
        this.materialDensity = materialDensity;

        // Planform area of one fin (trapezoid)
        double finArea = (rootChord + tipChord) / 2.0 * semiSpan;
        // Mass: volume * density (assuming solid fins)
        double volume = finArea * thickness;
        this.mass = volume * materialDensity * finCount;

        // CG of a trapezoidal fin (approximate at quarter chord of MAC) – simplified:
        // For now set CG at root chord midpoint in span direction? We'll just place it at 1/2 span from root.
        // But we need distance from nose tip; that will be set later by position on body.
        this.cgPosition = 0.0; // placeholder, will be set after assembly
        // CP for fins (subsonic) is at the quarter chord of the mean aerodynamic chord.
        // We'll compute a placeholder and later shift.
        this.cpPosition = 0.0; // placeholder
    }

    public int getFinCount() { return finCount; }
    public double getRootChord() { return rootChord; }
    public double getTipChord() { return tipChord; }
    public double getSemiSpan() { return semiSpan; }
    public double getSweep() { return sweep; }
    public double getThickness() { return thickness; }
}