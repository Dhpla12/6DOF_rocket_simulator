package rocketsim;

import java.io.FileWriter;
import java.io.PrintWriter;

/**
 * Main simulation class.
 * Integrates 6DOF equations of motion using 4th-order Runge-Kutta.
 */
public class Simulator6DOF {
    private Rocket rocket;
    private Motor motor;
    private double dryMass;          // mass of rocket without motor (kg)
    private State state;
    private double dt;                // time step (s)

    public Simulator6DOF(Rocket rocket, double dt) {
        this.rocket = rocket;
        this.dt = dt;
        this.state = new State();

        // Retrieve motor and dry mass from the rocket
        this.motor = rocket.getMotor();
        this.dryMass = rocket.getDryMass();

        // Initial conditions: launch from ground, vertical
        state.position[2] = 0.0;               // on ground
        state.velocity = new double[]{0.0, 0.0, 0.0};
        // Pointing upward (body z aligned with world z) – identity quaternion
        state.quaternion = new double[]{1.0, 0.0, 0.0, 0.0};
        state.angVelocity = new double[]{0.0, 0.0, 0.0};
    }

    /**
     * Compute derivatives for the state.
     */
    private State derivatives(State s) {
        State d = new State();
        d.time = 1.0; // time derivative

        // Extract state
        double[] pos = s.position;
        double[] vel = s.velocity;
        double[] q = s.quaternion;
        double[] omega = s.angVelocity; // body rates

        // Current total mass (dry mass + motor mass at this time)
        double mass = dryMass + motor.getMass(s.time);

        // Convert quaternion to rotation matrix (body to world)
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        double[][] R = new double[3][3];
        R[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        R[0][1] = 2*(q1*q2 - q0*q3);
        R[0][2] = 2*(q1*q3 + q0*q2);
        R[1][0] = 2*(q1*q2 + q0*q3);
        R[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        R[1][2] = 2*(q2*q3 - q0*q1);
        R[2][0] = 2*(q1*q3 - q0*q2);
        R[2][1] = 2*(q2*q3 + q0*q1);
        R[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Gravity force (mass * g, downward)
        double[] gravityForce = {0, 0, -9.81 * mass};

        // Thrust from motor (acts along body z-axis)
        double thrustMag = motor.getThrust(s.time);
        double[] thrustWorld = new double[3];
        thrustWorld[0] = R[0][2] * thrustMag;
        thrustWorld[1] = R[1][2] * thrustMag;
        thrustWorld[2] = R[2][2] * thrustMag;

        // Aerodynamics
        double altitude = pos[2];
        double rho = Atmosphere.getDensity(altitude);
        double speed = Math.sqrt(vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2]);
        double area = Aerodynamics.referenceArea(rocket.getRefDiameter());
        double Cd = Aerodynamics.estimateCd();

        // Velocity direction in world frame
        double[] velDir = new double[3];
        if (speed > 1e-6) {
            velDir[0] = vel[0]/speed;
            velDir[1] = vel[1]/speed;
            velDir[2] = vel[2]/speed;
        }

        // Angle of attack: need body z-axis direction in world frame
        double[] bodyZ = {R[0][2], R[1][2], R[2][2]}; // world frame
        double cosAlpha = 0;
        if (speed > 1e-6) {
            cosAlpha = (velDir[0]*bodyZ[0] + velDir[1]*bodyZ[1] + velDir[2]*bodyZ[2]);
        } else {
            cosAlpha = 1.0; // no velocity, assume aligned
        }
        double alpha = Math.acos(Math.max(-1, Math.min(1, cosAlpha))); // clamp

        // Drag force magnitude
        double dragMag = Aerodynamics.dragForce(rho, speed, Cd, area);
        // Drag acts opposite velocity direction
        double[] dragWorld = new double[3];
        dragWorld[0] = -dragMag * velDir[0];
        dragWorld[1] = -dragMag * velDir[1];
        dragWorld[2] = -dragMag * velDir[2];

        // Normal force (lift) – approximate
        double cnAlpha = Aerodynamics.estimateCnAlpha();
        double normalMag = Aerodynamics.normalForce(rho, speed, alpha, cnAlpha, area);
        // Normal force acts perpendicular to velocity in the plane containing velocity and body z-axis.
        double[] normalDir = new double[3];
        if (speed > 1e-6 && Math.abs(Math.sin(alpha)) > 1e-6) {
            // Compute perpendicular direction
            double[] temp = new double[3];
            temp[0] = bodyZ[0] - cosAlpha * velDir[0];
            temp[1] = bodyZ[1] - cosAlpha * velDir[1];
            temp[2] = bodyZ[2] - cosAlpha * velDir[2];
            double norm = Math.sqrt(temp[0]*temp[0] + temp[1]*temp[1] + temp[2]*temp[2]);
            normalDir[0] = temp[0]/norm;
            normalDir[1] = temp[1]/norm;
            normalDir[2] = temp[2]/norm;
        } else {
            normalDir = new double[3]; // zero
        }
        double[] normalWorld = new double[3];
        normalWorld[0] = normalMag * normalDir[0];
        normalWorld[1] = normalMag * normalDir[1];
        normalWorld[2] = normalMag * normalDir[2];

        // Total force in world frame
        double[] totalForce = new double[3];
        for (int i = 0; i < 3; i++) {
            totalForce[i] = gravityForce[i] + thrustWorld[i] + dragWorld[i] + normalWorld[i];
        }

        // Translational acceleration
        double[] accel = new double[3];
        for (int i = 0; i < 3; i++) {
            accel[i] = totalForce[i] / mass;
        }

        d.velocity = accel; // derivative of velocity is acceleration
        d.position = vel;   // derivative of position is velocity

        // Rotational dynamics
        double cpOffset = rocket.getCP() - rocket.getCG(); // positive if CP behind CG (stable)

        // Transform normal force direction to body frame
        double[] normalDirBody = new double[3];
        normalDirBody[0] = R[0][0]*normalDir[0] + R[1][0]*normalDir[1] + R[2][0]*normalDir[2];
        normalDirBody[1] = R[0][1]*normalDir[0] + R[1][1]*normalDir[1] + R[2][1]*normalDir[2];
        normalDirBody[2] = R[0][2]*normalDir[0] + R[1][2]*normalDir[1] + R[2][2]*normalDir[2];

        double[] normalBody = new double[3];
        normalBody[0] = normalMag * normalDirBody[0];
        normalBody[1] = normalMag * normalDirBody[1];
        normalBody[2] = normalMag * normalDirBody[2];

        // Moment arm in body frame from CG to CP
        double[] rCP = {0, 0, cpOffset};

        // Moment = rCP x normalBody
        double[] moment = new double[3];
        moment[0] = rCP[1]*normalBody[2] - rCP[2]*normalBody[1]; // = -cpOffset * normalBody[1]
        moment[1] = rCP[2]*normalBody[0] - rCP[0]*normalBody[2]; // =  cpOffset * normalBody[0]
        moment[2] = rCP[0]*normalBody[1] - rCP[1]*normalBody[0]; // = 0 (since rCP only has z)

        // Add damping moment (simplified, proportional to angular velocity)
        double damping = 0.001;
        moment[0] -= damping * omega[0];
        moment[1] -= damping * omega[1];
        moment[2] -= damping * omega[2];

        // Inertia tensor (assume diagonal for simplicity)
        double[][] inertia = rocket.getInertia();
        double Ixx = inertia[0][0];
        double Iyy = inertia[1][1];
        double Izz = inertia[2][2];

        // Angular acceleration in body frame
        double[] alphaAng = new double[3];
        alphaAng[0] = (moment[0] - (Izz - Iyy) * omega[1] * omega[2]) / Ixx;
        alphaAng[1] = (moment[1] - (Ixx - Izz) * omega[2] * omega[0]) / Iyy;
        alphaAng[2] = (moment[2] - (Iyy - Ixx) * omega[0] * omega[1]) / Izz;

        d.angVelocity = alphaAng; // derivative of angular velocity

        // Quaternion derivative
        double q0dot = 0.5 * (-q1*omega[0] - q2*omega[1] - q3*omega[2]);
        double q1dot = 0.5 * ( q0*omega[0] + q2*omega[2] - q3*omega[1]);
        double q2dot = 0.5 * ( q0*omega[1] + q3*omega[0] - q1*omega[2]);
        double q3dot = 0.5 * ( q0*omega[2] + q1*omega[1] - q2*omega[0]);
        d.quaternion = new double[]{q0dot, q1dot, q2dot, q3dot};

        return d;
    }

    // ... (the RK4 step() and run() methods remain unchanged from the previous version)

    /**
     * Perform one RK4 integration step.
     */
    private void step() {
        State s = state;

        State k1 = derivatives(s);
        State s2 = s.copy();
        s2.time = s.time + 0.5*dt;
        for (int i=0;i<3;i++) s2.velocity[i] = s.velocity[i] + 0.5*dt*k1.velocity[i];
        for (int i=0;i<3;i++) s2.position[i] = s.position[i] + 0.5*dt*k1.position[i];
        for (int i=0;i<4;i++) s2.quaternion[i] = s.quaternion[i] + 0.5*dt*k1.quaternion[i];
        for (int i=0;i<3;i++) s2.angVelocity[i] = s.angVelocity[i] + 0.5*dt*k1.angVelocity[i];
        // Normalize quaternion
        double norm = Math.sqrt(s2.quaternion[0]*s2.quaternion[0] + s2.quaternion[1]*s2.quaternion[1] + s2.quaternion[2]*s2.quaternion[2] + s2.quaternion[3]*s2.quaternion[3]);
        for (int i=0;i<4;i++) s2.quaternion[i] /= norm;

        State k2 = derivatives(s2);
        State s3 = s.copy();
        s3.time = s.time + 0.5*dt;
        for (int i=0;i<3;i++) s3.velocity[i] = s.velocity[i] + 0.5*dt*k2.velocity[i];
        for (int i=0;i<3;i++) s3.position[i] = s.position[i] + 0.5*dt*k2.position[i];
        for (int i=0;i<4;i++) s3.quaternion[i] = s.quaternion[i] + 0.5*dt*k2.quaternion[i];
        for (int i=0;i<3;i++) s3.angVelocity[i] = s.angVelocity[i] + 0.5*dt*k2.angVelocity[i];
        norm = Math.sqrt(s3.quaternion[0]*s3.quaternion[0] + s3.quaternion[1]*s3.quaternion[1] + s3.quaternion[2]*s3.quaternion[2] + s3.quaternion[3]*s3.quaternion[3]);
        for (int i=0;i<4;i++) s3.quaternion[i] /= norm;

        State k3 = derivatives(s3);
        State s4 = s.copy();
        s4.time = s.time + dt;
        for (int i=0;i<3;i++) s4.velocity[i] = s.velocity[i] + dt*k3.velocity[i];
        for (int i=0;i<3;i++) s4.position[i] = s.position[i] + dt*k3.position[i];
        for (int i=0;i<4;i++) s4.quaternion[i] = s.quaternion[i] + dt*k3.quaternion[i];
        for (int i=0;i<3;i++) s4.angVelocity[i] = s.angVelocity[i] + dt*k3.angVelocity[i];
        norm = Math.sqrt(s4.quaternion[0]*s4.quaternion[0] + s4.quaternion[1]*s4.quaternion[1] + s4.quaternion[2]*s4.quaternion[2] + s4.quaternion[3]*s4.quaternion[3]);
        for (int i=0;i<4;i++) s4.quaternion[i] /= norm;

        State k4 = derivatives(s4);

        // Combine
        state.time += dt;
        for (int i=0;i<3;i++) state.velocity[i] += (dt/6.0)*(k1.velocity[i] + 2*k2.velocity[i] + 2*k3.velocity[i] + k4.velocity[i]);
        for (int i=0;i<3;i++) state.position[i] += (dt/6.0)*(k1.position[i] + 2*k2.position[i] + 2*k3.position[i] + k4.position[i]);
        for (int i=0;i<4;i++) state.quaternion[i] += (dt/6.0)*(k1.quaternion[i] + 2*k2.quaternion[i] + 2*k3.quaternion[i] + k4.quaternion[i]);
        for (int i=0;i<3;i++) state.angVelocity[i] += (dt/6.0)*(k1.angVelocity[i] + 2*k2.angVelocity[i] + 2*k3.angVelocity[i] + k4.angVelocity[i]);

        // Normalize quaternion after update
        norm = Math.sqrt(state.quaternion[0]*state.quaternion[0] + state.quaternion[1]*state.quaternion[1] + state.quaternion[2]*state.quaternion[2] + state.quaternion[3]*state.quaternion[3]);
        for (int i=0;i<4;i++) state.quaternion[i] /= norm;
    }

    public void run(double simTime, String outputFile) {
        try (PrintWriter out = new PrintWriter(new FileWriter(outputFile))) {
            out.println("time(s),x(m),y(m),z(m),vx(m/s),vy(m/s),vz(m/s),q0,q1,q2,q3,roll(rad),pitch(rad),yaw(rad)");
            while (state.time <= simTime) {
                // Write state
                double[] euler = quatToEuler(state.quaternion);
                out.printf("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                        state.time,
                        state.position[0], state.position[1], state.position[2],
                        state.velocity[0], state.velocity[1], state.velocity[2],
                        state.quaternion[0], state.quaternion[1], state.quaternion[2], state.quaternion[3],
                        euler[0], euler[1], euler[2]);
                step();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
     */
    private double[] quatToEuler(double[] q) {
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
        double roll = Math.atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
        double pitch = Math.asin(2*(q0*q2 - q3*q1));
        double yaw = Math.atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
        return new double[]{roll, pitch, yaw};
    }

    public static void main(String[] args) {
        Rocket rocket = new Rocket().buildExampleRocket();
        Simulator6DOF sim = new Simulator6DOF(rocket, 0.01); // 10 ms time step
        sim.run(5.0, "rocket_trajectory.csv"); // simulate 5 seconds
        System.out.println("Simulation finished. Output written to rocket_trajectory.csv");
    }
}