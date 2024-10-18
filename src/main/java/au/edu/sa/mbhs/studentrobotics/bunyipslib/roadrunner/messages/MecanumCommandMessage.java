package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.messages;

/**
 * RoadRunner v1.0 logging message for a mecanum drive command.
 *
 * @since 6.0.0
 */
public final class MecanumCommandMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The battery voltage at this time.
     */
    public double voltage;
    /**
     * The power of the left front wheel at this time.
     */
    public double leftFrontPower;
    /**
     * The power of the left back wheel at this time.
     */
    public double leftBackPower;
    /**
     * The power of the right back wheel at this time.
     */
    public double rightBackPower;
    /**
     * The power of the right front wheel at this time.
     */
    public double rightFrontPower;

    @SuppressWarnings("MissingJavadoc")
    public MecanumCommandMessage(double voltage, double leftFrontPower, double leftBackPower, double rightBackPower, double rightFrontPower) {
        timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftFrontPower = leftFrontPower;
        this.leftBackPower = leftBackPower;
        this.rightBackPower = rightBackPower;
        this.rightFrontPower = rightFrontPower;
    }
}
