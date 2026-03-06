# 6DOF Rocket Simulator

This is a **basic skeleton** of a 6‑degree‑of‑freedom model rocket simulator, inspired by OpenRocket. It is built as a part‑time learning project and currently implements core flight dynamics. The code is written in Java and outputs trajectory data to a CSV file.

## Current Features

- **6DOF dynamics** using quaternions (no gimbal lock)
- **RK4 integration** for accurate time stepping
- **Thrust curve** for a public‑domain Estes A8‑3 motor
- **Gravity, drag, and simplified normal force (lift)**
- **Mass variation** due to propellant consumption
- **CSV output** with position, velocity, orientation, and Euler angles

## Project Structure

All source files are located in the `rocket-simulator/` directory.  
The main class is `rocketsim.Simulator6DOF`.  
An example output file `rocket_trajectory.csv` is included to show the expected format.

## How to Run

1. Navigate to the `rocket-simulator/` folder.
2. Compile the Java files:
   ```bash
   javac -d out src/main/java/rocketsim/*.java
   ```
   (If you are using the structure as in the repository, adjust the path accordingly. The source files are directly under `rocket-simulator/`.)
3. Run the simulator:
   ```bash
   java -cp out rocketsim.Simulator6DOF
   ```
4. A new `rocket_trajectory.csv` file will be created (or you can examine the included sample).

## Future Enhancements

This is just the beginning. Planned additions include:

- A **graphical user interface (GUI)** to visualise the flight in real time
- **Interactive component selection** – choose different nose cones, body tubes, fins, and motors
- **More accurate aerodynamics** (e.g., Barrowman formulas)
- **Wind and atmospheric variations**
- **3D visualisation** using JavaFX or similar

## Contributing

As a learning project, suggestions and contributions are welcome! Feel free to open issues or submit pull requests.

---

*Happy simulating!*
