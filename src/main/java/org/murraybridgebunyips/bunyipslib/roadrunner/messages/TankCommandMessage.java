package org.murraybridgebunyips.bunyipslib.roadrunner.messages;

/**
 * RoadRunner v1.0 logging message for a tank drive command.
 *
 * @since 6.0.0
 */
public final class TankCommandMessage {
    /**
     * The timestamp this message was created.
     */
    public long timestamp;
    /**
     * The battery voltage at this time.
     */
    public double voltage;
    /**
     * The power of the left wheels at this time.
     */
    public double leftPower;
    /**
     * The power of the right wheels at this time.
     */
    public double rightPower;

    @SuppressWarnings("MissingJavadoc")
    public TankCommandMessage(double voltage, double leftPower, double rightPower) {
        timestamp = System.nanoTime();
        this.voltage = voltage;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
    }
}
