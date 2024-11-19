package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * A {@link TouchSensor} where the readings can be inverted.
 * <p>
 * This is useful for sensors that are normally true, but are false when pressed, while various implementations
 * expect the opposite to represent a sensor press.
 *
 * @author Lucas Bubner, 2024
 * @see TouchSensor
 * @since 6.0.0
 */
public class InvertibleTouchSensor implements TouchSensor {
    private final TouchSensor sensor;
    private boolean inverted = true;

    /**
     * Creates a new InvertibleTouchSensor that inverts the readings of the given {@link TouchSensor}.
     *
     * @param sensor the sensor to invert
     */
    public InvertibleTouchSensor(TouchSensor sensor) {
        this.sensor = sensor;
    }

    /**
     * Sets the inversion state of the sensor.
     *
     * @param inverted whether to invert the readings, default is true as using this class implies that the sensor
     *                 needs to be inverted initially for all readings to be correct
     */
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public double getValue() {
        return inverted ? 1 - sensor.getValue() : sensor.getValue();
    }

    @Override
    public boolean isPressed() {
        return inverted != sensor.isPressed();
    }

    @Override
    public Manufacturer getManufacturer() {
        return sensor.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return sensor.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return sensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return sensor.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        sensor.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        sensor.close();
    }
}
