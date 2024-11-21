package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Filter;

/**
 * Represents a motor or external encoder that takes in suppliers of position and velocity to calculate encoder ticks.
 * The velocity methods of this class have been adjusted to include acceleration and corrected velocity receivers.
 * <p>
 * This class uses an internal accumulator to determine the current position of the encoder, allowing for the current
 * position to be asserted by the end user.
 *
 * @author Lucas Bubner, 2024
 * @see RawEncoder RawEncoder - used in drivebases and for RoadRunner
 * @since 4.0.0
 */
public class Encoder {
    private static final int CPS_STEP = 0x10000;
    private final Supplier<Integer> position;
    private final Supplier<Double> velocity;
    private Filter.LowPass accelFilter = new Filter.LowPass(0.95);
    private int resetVal, lastPosition;
    private DcMotorSimple.Direction direction;
    private double lastTimestamp, veloEstimate, accel, lastVelo;
    private boolean overflowCorrection;
    private Supplier<DcMotorSimple.Direction> directionSupplier;
    private int accumulation;

    /**
     * The encoder object for some positional and velocity supplier.
     *
     * @param position the position supplier which points to the
     *                 current position of the encoder in ticks ({@code obj::getCurrentPosition})
     * @param velocity the velocity supplier which points to the
     *                 current velocity the encoder in ticks per second ({@code obj::getVelocity})
     */
    public Encoder(@NonNull Supplier<Integer> position, @NonNull Supplier<Double> velocity) {
        this.position = position;
        this.velocity = velocity;
        resetVal = 0;
        lastPosition = 0;
        veloEstimate = 0;
        direction = DcMotorSimple.Direction.FORWARD;
        lastTimestamp = System.nanoTime() / 1.0E9;
    }

    /**
     * Set a supplier that will be used to auto-set encoder direction.
     *
     * @param supplier the supplier to use
     */
    public void trackDirection(@NonNull Supplier<DcMotorSimple.Direction> supplier) {
        directionSupplier = supplier;
    }

    /**
     * Gets the current direction of this encoder.
     *
     * @return the logical direction this encoder operates
     */
    @NonNull
    public DcMotorSimple.Direction getDirection() {
        if (directionSupplier != null)
            direction = directionSupplier.get();
        return direction;
    }

    /**
     * Sets the direction of the encoder to forward or reverse
     *
     * @param direction the desired direction
     */
    public void setDirection(@NonNull DcMotorSimple.Direction direction) {
        this.direction = direction;
    }

    /**
     * Call to use encoder overflow (exceeding 32767 ticks/sec) correction on {@link #getVelocity()}.
     */
    public void useEncoderOverflowCorrection() {
        overflowCorrection = true;
    }

    /**
     * @return the current position of the encoder
     */
    public int getPosition() {
        int currentPosition = (getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1) * position.get();
        accumulation += currentPosition - lastPosition;
        if (currentPosition != lastPosition) {
            double currentTime = System.nanoTime() / 1.0E9;
            double dt = currentTime - lastTimestamp;
            veloEstimate = (currentPosition - lastPosition) / dt;
            lastPosition = currentPosition;
            lastTimestamp = currentTime;
        }
        return accumulation - resetVal;
    }

    /**
     * Resets the encoder without having to adjust output speed.
     */
    public void reset() {
        resetVal += getPosition();
    }

    /**
     * Sets the current (known) position of the encoder to this value.
     *
     * @param position the known position to set this encoder to
     */
    public void setKnownPosition(int position) {
        resetVal = 0;
        accumulation = position;
    }

    /**
     * Set a new Low Pass filter gain for use with acceleration estimation readings.
     *
     * @param gain the gain in the interval (0, 1) exclusive; default of 0.95
     */
    public void setAccelLowPassGain(double gain) {
        accelFilter = new Filter.LowPass(gain);
    }

    /**
     * @return the estimated acceleration of the encoder in ticks per second squared. this method will internally
     * call {@link #getRawVelocity()} to update the velocity information which is required.
     */
    public double getAcceleration() {
        getRawVelocity();
        return accel;
    }

    /**
     * @return the velocity of the encoder, will correct for overflow if told to do so
     * by {@link #useEncoderOverflowCorrection()}.
     */
    public double getVelocity() {
        return overflowCorrection ? getCorrectedVelocity() : getRawVelocity();
    }

    /**
     * @return the raw velocity of the encoder, may overflow if ticks/sec exceed 32767/sec
     */
    public double getRawVelocity() {
        double velo = (getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1) * velocity.get();
        double currentTime = System.nanoTime() / 1.0E9;
        double dt = currentTime - lastTimestamp;
        // Too small of measurements are incalculable due to floating-point error
        if (dt > 1.0E-3) {
            accel = accelFilter.apply((velo - lastVelo) / dt);
            lastVelo = velo;
            lastTimestamp = currentTime;
        }
        return velo;
    }

    /**
     * Gets the current encoder velocity while also correcting for velocity overflow
     * (REV hardware limitation workaround)
     *
     * @return the corrected velocity
     * @see #getRawVelocity()
     */
    public double getCorrectedVelocity() {
        double real = getRawVelocity();
        while (Math.abs(veloEstimate - real) > CPS_STEP / 2.0) {
            real += Math.signum(veloEstimate - real) * CPS_STEP;
        }
        return real;
    }
}