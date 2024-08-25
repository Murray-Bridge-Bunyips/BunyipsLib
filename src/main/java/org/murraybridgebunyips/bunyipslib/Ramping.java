package org.murraybridgebunyips.bunyipslib;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.external.units.Measure;
import org.murraybridgebunyips.bunyipslib.external.units.Time;

import java.util.function.DoubleSupplier;

/**
 * A collection of ramping functions and values for smoothing out deltas over time.
 *
 * @author Lucas Bubner, 2024
 */
public interface Ramping {
    /**
     * Set whether the ramping function is enabled.
     * If disabled, the value will instantly set its value to the target value.
     *
     * @param enabled whether the ramping function is enabled, default is on
     * @return this
     */
    Ramping setRampingEnabled(boolean enabled);

    /**
     * Set the ramping parameters of the value.
     *
     * @param time  the time it takes for the value to reach the target value.
     * @param delta the maximum change in value per second
     * @return this
     */
    Ramping setRampingParameters(Measure<Time> time, double delta);

    /**
     * Set the time it takes for the value to reach the target.
     *
     * @param smoothTime the time it takes for the value to reach the target.
     * @return this
     */
    Ramping setRampingTime(Measure<Time> smoothTime);

    /**
     * Set the maximum change in units per second.
     *
     * @param maxDelta the maximum change in units per second
     * @return this
     */
    Ramping setMaxRampingDelta(double maxDelta);

    /**
     * SmoothDamp implementation for a {@code double} value.
     *
     * @author Lucas Bubner, 2024
     */
    class Value implements Ramping {
        private final ElapsedTime timer = new ElapsedTime();
        private final Reference<Double> v = Reference.of(0.0);
        private Measure<Time> smoothTime = Seconds.of(0.1);
        private double maxDelta = 1.0;
        // Enabled by default
        private boolean enabled = true;

        /**
         * Set whether the ramping function is enabled.
         * If disabled, the value will instantly set its value to the target value.
         *
         * @param enabled whether the ramping function is enabled, default is on
         */
        @Override
        public Value setRampingEnabled(boolean enabled) {
            this.enabled = enabled;
            return this;
        }

        /**
         * Set the ramping parameters of the value.
         *
         * @param time  the time it takes for the value to reach the target value.
         * @param delta the maximum change in value per second
         * @return this
         */
        @Override
        public Value setRampingParameters(Measure<Time> time, double delta) {
            smoothTime = time;
            maxDelta = delta;
            return this;
        }

        /**
         * Set the time it takes for the value to reach the target.
         *
         * @param smoothTime the time it takes for the value to reach the target.
         */
        @Override
        public Value setRampingTime(Measure<Time> smoothTime) {
            this.smoothTime = smoothTime;
            return this;
        }

        /**
         * Set the maximum change in units per second.
         *
         * @param maxDelta the maximum change in units per second
         */
        @Override
        public Value setMaxRampingDelta(double maxDelta) {
            this.maxDelta = maxDelta;
            return this;
        }

        /**
         * Gets a SmoothDamped result from the value supplier.
         * Must be called regularly to update the SmoothDamp velocities.
         *
         * @param current the current value
         * @param target  the target value
         * @return a result
         */
        public double get(double current, double target) {
            if (!enabled)
                return target;

            if (current == target) {
                timer.reset();
                return target;
            }

            return Mathf.smoothDamp(current, target, v, smoothTime, maxDelta, deltaTime());
        }

        /**
         * Gets a SmoothDamped result from the value supplier.
         * Must be called regularly to update the SmoothDamp velocities.
         *
         * @param current the current value
         * @param target  the target value
         * @return a result
         */
        public float get(float current, float target) {
            return (float) get((double) current, target);
        }

        private Measure<Time> deltaTime() {
            double dt = timer.seconds();
            timer.reset();
            return Seconds.of(dt);
        }
    }

    /**
     * A {@link Value} that uses a SmoothDamp function to smooth out values over time.
     * Uses a supplier for the current value, based on a target set by the loop.
     *
     * @author Lucas Bubner, 2024
     */
    class Supplier implements Ramping {
        private final Value v = new Value();
        private DoubleSupplier current;

        /**
         * Create a new Ramping Supplier object, wrapping a DoubleSupplier that will be used
         * in place of the current value when {@link #get(double)} is called.
         *
         * @param current the DoubleSupplier object to wrap. By default, the ramping function is enabled.
         */
        public Supplier(@NonNull DoubleSupplier current) {
            this.current = current;
        }

        public Supplier setCurrentSupplier(@NonNull DoubleSupplier current) {
            this.current = current;
            return this;
        }

        /**
         * Set whether the ramping function is enabled.
         * If disabled, the value will instantly set its value to the target value.
         *
         * @param enabled whether the ramping function is enabled, default is on
         */
        @Override
        public Supplier setRampingEnabled(boolean enabled) {
            v.setRampingEnabled(enabled);
            return this;
        }

        /**
         * Set the ramping parameters of the value.
         *
         * @param time  the time it takes for the value to reach the target value.
         * @param delta the maximum change in value per second
         * @return this
         */
        @Override
        public Supplier setRampingParameters(Measure<Time> time, double delta) {
            v.setRampingParameters(time, delta);
            return this;
        }

        /**
         * Set the time it takes for the value to reach the target.
         *
         * @param smoothTime the time it takes for the value to reach the target.
         * @return this
         */
        @Override
        public Supplier setRampingTime(Measure<Time> smoothTime) {
            v.setRampingTime(smoothTime);
            return this;
        }

        /**
         * Set the maximum change in units per second.
         *
         * @param maxDelta the maximum change in units per second
         * @return this
         */
        @Override
        public Supplier setMaxRampingDelta(double maxDelta) {
            v.setMaxRampingDelta(maxDelta);
            return this;
        }

        /**
         * Gets a ramped value from the value supplier and target value.
         * Must be called regularly to update the SmoothDamp velocities.
         *
         * @param target the target value
         * @return a ramped result
         */
        public double get(double target) {
            return v.get(current.getAsDouble(), target);
        }


        /**
         * Gets a ramped value from the value supplier and target value.
         * Must be called regularly to update the SmoothDamp velocities.
         *
         * @param target the target value
         * @return a ramped result
         */
        public float get(float target) {
            return (float) get((double) target);
        }
    }

    /**
     * Wrapper class for a DcMotor where all motor speed is passed through a configurable SmoothDamp function.
     * This class is designed to be downcasted to the SDK DcMotor type.
     *
     * @author Lucas Bubner, 2024
     */
    class DcMotor extends DcMotorImpl implements Ramping {
        private final Supplier v = new Supplier(this::getPower);

        /**
         * Create a new Ramping DcMotor object, wrapping a DcMotor object and
         * returning a new object that can be used in place of the original.
         * By default, the ramping function time is set to 0.1 seconds, with a maximum delta of 1.0 power units per second.
         *
         * @param motor the DcMotor object to wrap. By default, the ramping function is enabled.
         */
        public DcMotor(com.qualcomm.robotcore.hardware.DcMotor motor) {
            super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
        }

        /**
         * Create a new Ramping DcMotor object, wrapping a DcMotor object and
         * returning a new object that can be used in place of the original.
         *
         * @param motor      the DcMotor object to wrap. By default, the ramping function is enabled.
         * @param smoothTime the time it takes for the motor to reach the target power level
         * @param maxDelta   the maximum change in power level per second
         */
        public DcMotor(com.qualcomm.robotcore.hardware.DcMotor motor, Measure<Time> smoothTime, double maxDelta) {
            super(motor.getController(), motor.getPortNumber(), motor.getDirection(), motor.getMotorType());
            v.setRampingParameters(smoothTime, maxDelta);
        }

        /**
         * Set whether the ramping function is enabled.
         * If disabled, the motor will instantly set its power level to the target power level.
         *
         * @param enabled whether the ramping function is enabled, default is on
         */
        @Override
        public DcMotor setRampingEnabled(boolean enabled) {
            v.setRampingEnabled(enabled);
            return this;
        }

        /**
         * Set the ramping parameters of the motor.
         *
         * @param smoothTime the time it takes for the motor to reach the target power level
         * @param maxDelta   the maximum change in power level per second
         */
        @Override
        public DcMotor setRampingParameters(Measure<Time> smoothTime, double maxDelta) {
            v.setRampingParameters(smoothTime, maxDelta);
            return this;
        }

        /**
         * Set the time it takes for the motor to reach the target power level.
         *
         * @param smoothTime the time it takes for the motor to reach the target power level
         */
        @Override
        public DcMotor setRampingTime(Measure<Time> smoothTime) {
            v.setRampingTime(smoothTime);
            return this;
        }

        /**
         * Set the maximum change in power level per second.
         *
         * @param maxDelta the maximum change in power level per second
         */
        @Override
        public DcMotor setMaxRampingDelta(double maxDelta) {
            v.setMaxRampingDelta(maxDelta);
            return this;
        }

        /**
         * Set the power level of the motor, which will be passed through a SmoothDamp function defined by the motor's ramping parameters.
         * <b>This function must be called periodically</b> (e.g. constantly during an activeLoop) to update the motor's power level,
         * as it relies on the time since the last call to calculate the new power level.
         *
         * @param power the new power level of the motor, a value in the interval [-1.0, 1.0]
         */
        @Override
        public void setPower(double power) {
            super.setPower(v.get(power));
        }

        /**
         * Instantly set the power level of the motor, bypassing the SmoothDamp function.
         */
        public void setPowerInstant(double power) {
            super.setPower(power);
        }

        /**
         * Instantly stop the motor, bypassing the SmoothDamp function.
         */
        public void stop() {
            setPowerInstant(0);
        }
    }
}
