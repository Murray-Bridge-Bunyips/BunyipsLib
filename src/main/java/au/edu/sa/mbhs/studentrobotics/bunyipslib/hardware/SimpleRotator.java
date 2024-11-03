package au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Nanoseconds;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;

/**
 * Drop-in replacement for {@link DcMotorSimple} (and by extension, {@link DcMotor} and {@link CRServo}) to support
 * write caching, refresh rates, and internal power clamps/scales.
 * <p>
 * This class is internally implemented by {@link Motor}. This class is exposed for {@link CRServo} uses and other
 * motor implementations that do not need the full suite of {@link Motor} utilities.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class SimpleRotator implements DcMotorSimple {
    private final DcMotorSimple dms;
    private double maxMagnitude = 1;
    private double powerDeltaTolerance = 0;
    private double lastPower = 0;
    private long refreshRateNanos = 0;
    private long lastUpdate = 0;

    /**
     * Wrap a DcMotorSimple to use in the SimpleCachedRotator class.
     *
     * @param dms the motor or CRServo instance to use
     */
    public SimpleRotator(DcMotorSimple dms) {
        this.dms = dms;
    }

    /**
     * Set the delta in power required to propagate a hardware write.
     * A commanded power of 0 to the motor will ignore this threshold for safety.
     *
     * @param magnitude absolute magnitude of delta in power, 0/default will disable
     */
    public void setPowerDeltaThreshold(double magnitude) {
        powerDeltaTolerance = Math.abs(magnitude);
    }

    /**
     * Set the refresh rate of the motor that will be a minimum time between hardware writes.
     * Consistent calls to {@link #setPower(double)} is required for this refresh rate to be effective.
     * A commanded power of 0 to the motor will ignore this refresh rate for safety.
     *
     * @param refreshRate the refresh rate interval, <=0/default will disable
     */
    public void setPowerRefreshRate(@NonNull Measure<Time> refreshRate) {
        refreshRateNanos = (long) refreshRate.in(Nanoseconds);
    }

    /**
     * Sets the maximum power magnitude (applies for both negative and positive powers) this motor can run at.
     * This will ensure all calls to {@link #setPower} will never exceed the maximum boundary as defined by this function.
     * <p>
     * Note: Further calls to {@link #setPower} that exceed the new domain will be scaled (for example, if the max power
     * was defined as 0.8, {@code setPower(1)} will set the motor power to 0.8 in {@code RUN_WITHOUT_ENCODER} mode
     * for a DcMotor)
     *
     * @param magnitude maximum absolute magnitude of {@link #setPower}, default of 1.0 (SDK), applies bidirectionally
     */
    public void setMaxPower(double magnitude) {
        maxMagnitude = Math.min(1, Math.abs(magnitude));
    }

    @Override
    public Direction getDirection() {
        return dms.getDirection();
    }

    @Override
    public synchronized void setDirection(Direction direction) {
        dms.setDirection(direction);
    }

    @Override
    public synchronized double getPower() {
        return dms.getPower();
    }

    /**
     * @apiNote SimpleRotator implementation sets power with scaling, refresh rate, and power delta tolerance.
     * Zero power calls are bypassed for safety.
     */
    @Override
    public synchronized void setPower(double power) {
        // Clamp and rescale depending on the maximum magnitude
        power = Mathf.scale(Mathf.clamp(power, -1, 1), -1, 1, -maxMagnitude, maxMagnitude);
        // Always pass through to the motor if 0 power is requested
        if (power != 0) {
            // Can check refresh rate and apply cache for a non-zero power
            if (refreshRateNanos > 0 && Math.abs(lastUpdate - System.nanoTime()) < refreshRateNanos) {
                return;
            }
            // We also trigger if 1.0 or -1.0 power is desired, since a delta tolerance can leave the actual power near the limit
            if (powerDeltaTolerance != 0 && Math.abs(lastPower - power) < powerDeltaTolerance && power != -1 && power != 1) {
                return;
            }
            if (power == lastPower) {
                // Useless operation
                return;
            }
        }
        // Update to new state of system
        lastUpdate = System.nanoTime();
        lastPower = power;
        // Write to the hardware
        dms.setPower(power);
    }

    @Override
    public Manufacturer getManufacturer() {
        return dms.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return dms.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return dms.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return dms.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        dms.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        dms.close();
    }
}
