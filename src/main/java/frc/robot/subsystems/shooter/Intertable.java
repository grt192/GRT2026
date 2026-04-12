package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class Intertable {

    private static final Intertable INSTANCE = new Intertable();

    private static InterpolatingDoubleTreeMap shooterAngle = new InterpolatingDoubleTreeMap();
    private static InterpolatingDoubleTreeMap shooterRPS = new InterpolatingDoubleTreeMap();

    private static boolean initialized = false;

    private Intertable() {
        if (!initialized) {
            // Hood angles must be within limits: 0.06 to 0.169 rotations
            // Closer distance = higher angle, farther = lower angle
            shooterAngle.put(1.524, 0.14); // closest - highest angle
            shooterAngle.put(1.651, 0.23);
            shooterAngle.put(1.981, 0.32);
            shooterAngle.put(2.261, 0.41);
            shooterAngle.put(2.616, 0.5); // farthest - lowest angle

            shooterRPS.put(1.524, 40.0);
            shooterRPS.put(1.651, 42.25);
            shooterRPS.put(1.981, 44.5);
            shooterRPS.put(2.261, 46.75);
            shooterRPS.put(2.616, 49.0);

            initialized = true;
        }
    }

    public static Intertable getInstance() {
        return INSTANCE;
    }

    public double getRPS(double dis) {
        return shooterRPS.get(dis);
    }

    public double getAngle(double dis) {
        return shooterAngle.get(dis);
    }
}
