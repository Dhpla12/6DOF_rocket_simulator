package rocketsim;

/**
 * Holds the full state of the rocket at a given time.
 * Uses quaternion for orientation.
 */
public class State {
    public double time;           // s
    public double[] position;     // [x, y, z] in Earth frame (z up)
    public double[] velocity;     // [vx, vy, vz]
    public double[] quaternion;   // [q0, q1, q2, q3] representing body orientation
    public double[] angVelocity;  // [p, q, r] in body frame (rad/s)

    public State() {
        position = new double[3];
        velocity = new double[3];
        quaternion = new double[]{1.0, 0.0, 0.0, 0.0}; // identity
        angVelocity = new double[3];
    }

    public State copy() {
        State s = new State();
        s.time = time;
        System.arraycopy(position, 0, s.position, 0, 3);
        System.arraycopy(velocity, 0, s.velocity, 0, 3);
        System.arraycopy(quaternion, 0, s.quaternion, 0, 4);
        System.arraycopy(angVelocity, 0, s.angVelocity, 0, 3);
        return s;
    }
}