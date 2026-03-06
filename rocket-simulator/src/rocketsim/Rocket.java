package rocketsim;

import java.util.ArrayList;
import java.util.List;

/**
 * Assembles all components to form the complete rocket.
 * Computes total mass, CG, CP, and inertia.
 */
public class Rocket {
    private Motor motor;
    private double dryMass;
    private List<RocketComponent> components;
    private double totalLength;          // from nose tip to motor nozzle
    private double referenceDiameter;    // body diameter for drag reference area
    private double mass;                 // total mass
    private double cg;                   // distance from nose tip
    private double cp;                   // distance from nose tip
    private double[][] inertiaTensor;     // 3x3 matrix (body frame, origin at CG)

    public Rocket() {
        components = new ArrayList<>();
    }

    // For simplicity, we hardcode a typical rocket (Estes Alpha‑like) here.
    public Rocket buildExampleRocket() {
        Rocket rocket = new Rocket();

        // ----- Thrust curve for an Estes A8‑3 motor (simplified) -----
        List<double[]> thrustCurve = new ArrayList<>();
        thrustCurve.add(new double[]{0.0, 5.0});   // ignition
        thrustCurve.add(new double[]{0.2, 8.0});   // peak
        thrustCurve.add(new double[]{0.8, 4.0});   // tail-off
        thrustCurve.add(new double[]{1.0, 0.0});   // burnout

        // ----- Motor -----
        Motor motor = new Motor(0.01, 0.005, 1.0, thrustCurve);
        rocket.motor = motor;
        rocket.components.add(motor);

        // ----- Nose cone (conical) -----
        NoseCone nose = new NoseCone(0.1, 0.025, 0.001, 800);
        rocket.components.add(nose);

        // ----- Body tube -----
        BodyTube body = new BodyTube(0.2, 0.025, 0.001, 800);
        rocket.components.add(body);

        // ----- Fins (set of 3) -----
        FinSet fins = new FinSet(3, 0.04, 0.02, 0.03, 0.01, 0.002, 800);
        rocket.components.add(fins);

        // ----- Approximate dry mass (total mass minus motor initial mass) -----
        // Total mass ~0.05 kg, motor initial mass = 0.015 kg
        rocket.dryMass = 0.05 - motor.getMass(0);   // = 0.035 kg

        // ----- Simplified placement: all components are assumed to be stacked end‑to‑end -----
        // Nose cone front at 0.0, length 0.1 m
        // Body tube front at 0.1, length 0.2 m
        // Fins are attached near the rear, say from 0.25 to 0.29 m
        // Motor front at 0.29, length 0.07 m

        // For the simulation we only need the overall CG and CP.
        // These numbers are approximations for a stable model rocket.
        rocket.cg = 0.15;   // 15 cm from nose tip
        rocket.cp = 0.25;   // 25 cm from nose tip (behind CG → stable)
        rocket.mass = 0.05; // total mass (kg) – constant in this simplified version

        // Inertia tensor (simplified, diagonal)
        rocket.inertiaTensor = new double[][]{
                {0.0001, 0,     0},
                {0,      0.001, 0},
                {0,      0,     0.001}
        };

        rocket.referenceDiameter = 0.025; // 25 mm body diameter
        rocket.totalLength = 0.36;        // overall length (approx)

        return rocket;
    }

    // ----- Getters -----
    public Motor getMotor() {
        return motor;
    }

    public double getDryMass() {
        return dryMass;
    }

    public double getMass() {
        return mass;
    }

    public double getCG() {
        return cg;
    }

    public double getCP() {
        return cp;
    }

    public double getRefDiameter() {
        return referenceDiameter;
    }

    public double[][] getInertia() {
        return inertiaTensor;
    }
}