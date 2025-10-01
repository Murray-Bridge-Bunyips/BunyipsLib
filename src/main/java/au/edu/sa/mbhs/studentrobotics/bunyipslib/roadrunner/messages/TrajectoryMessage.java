package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;
import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Meters;

/**
 * RoadRunner v1.0 logging message for a trajectory sequence.
 *
 * @author Lucas Bubner, 2025
 * @since 7.5.0
 */
public final class TrajectoryMessage {
    /**
     * (x, y) pairs of trajectory points.
     */
    public double[] translation2d;

    @SuppressWarnings("MissingJavadoc")
    public TrajectoryMessage(double[] xPoints, double[] yPoints) {
        assert xPoints.length == yPoints.length;
        translation2d = new double[xPoints.length * 2];
        for (int i = 0, j = 0; i < xPoints.length; i++, j += 2) {
            translation2d[j] = Meters.convertFrom(xPoints[i], Inches);
            translation2d[j + 1] = Meters.convertFrom(yPoints[i], Inches);
        }
    }
}
