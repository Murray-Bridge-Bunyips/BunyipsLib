package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;

import androidx.annotation.NonNull;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;

/**
 * Drive model parameters for inches per tick and track width configuration.
 *
 * @author Lucas Bubner, 2024
 * @since 6.0.0
 */
public class DriveModel {
    /**
     * The ratio of inches per encoder tick for the drive.
     */
    public double inPerTick;
    /**
     * The ratio of inches per encoder tick for lateral movement.
     * Not required for a differential drive.
     */
    public double lateralInPerTick;
    /**
     * The track width in encoder ticks.
     */
    public double trackWidthTicks;

    @NonNull
    @Override
    public String toString() {
        return "DriveModel{" +
                "inPerTick=" + inPerTick +
                ", lateralInPerTick=" + lateralInPerTick +
                ", trackWidthTicks=" + trackWidthTicks +
                '}';
    }

    /**
     * Builder class to assist in creating a DriveModel.
     */
    public static class Builder {
        private final DriveModel model = new DriveModel();

        /**
         * Set the inches per tick for the drive as a result of a tuned measurement ratio.
         *
         * @param inPerTick the inches per tick for the drive calculated as (real dist in inches / ticks reported)
         * @return the builder
         */
        @NonNull
        public Builder setInPerTick(double inPerTick) {
            model.inPerTick = inPerTick;
            if (model.lateralInPerTick == 0) model.lateralInPerTick = inPerTick;
            return this;
        }

        // TODO: averages for multiple tests?
        /**
         * Set the inches per tick for the drive as a result of movement and encoder ticks.
         *
         * @param measuredDistance the measured distance of the movement
         * @param reportedTicks    the reported ticks from the encoder when the distance was measured
         * @return the builder
         */
        @NonNull
        public Builder setDistPerTick(@NonNull Measure<Distance> measuredDistance, double reportedTicks) {
            setInPerTick(measuredDistance.in(Inches) / reportedTicks);
            return this;
        }

        /**
         * Set the inches per tick for the drive as a result of a precomputed ratio.
         *
         * @param ticksPerRevolution the ticks per revolution of the encoder
         * @param gearRatio          the gear ratio of the drive
         * @param wheelDiameter      the diameter of the wheel
         * @return the builder
         */
        @NonNull
        public Builder setComputedInPerTick(double ticksPerRevolution, double gearRatio, @NonNull Measure<Distance> wheelDiameter) {
            setInPerTick((wheelDiameter.in(Inches) * Math.PI) / (ticksPerRevolution * gearRatio));
            return this;
        }

        /**
         * Set the inches per tick for lateral movement as a result of a tuned measurement ratio.
         * Not required for a differential drive. By default, this is set to the same value as inches per tick.
         *
         * @param lateralInPerTick the inches per tick for lateral movement
         * @return the builder
         */
        @NonNull
        public Builder setLateralInPerTick(double lateralInPerTick) {
            model.lateralInPerTick = lateralInPerTick;
            return this;
        }

        /**
         * Set the inches per tick for lateral movement as a result of movement and encoder ticks.
         * Not required for a differential drive. By default, this is set to the same value as inches per tick.
         *
         * @param measuredDistance the measured distance of the movement
         * @param reportedTicks    the reported ticks from the encoder when the distance was measured
         * @return the builder
         */
        @NonNull
        public Builder setLateralDistPerTick(@NonNull Measure<Distance> measuredDistance, double reportedTicks) {
            model.lateralInPerTick = measuredDistance.in(Inches) / reportedTicks;
            return this;
        }

        /**
         * Set the inches per tick for lateral movement as a result of a precomputed ratio.
         * Not required for a differential drive. By default, this is set to the same value as inches per tick.
         *
         * @param ticksPerRevolution the ticks per revolution of the encoder
         * @param gearRatio          the gear ratio of the drive
         * @param wheelDiameter      the diameter of the wheel
         * @return the builder
         */
        @NonNull
        public Builder setComputedLateralInPerTick(double ticksPerRevolution, double gearRatio, @NonNull Measure<Distance> wheelDiameter) {
            model.lateralInPerTick = (wheelDiameter.in(Inches) * Math.PI) / (ticksPerRevolution * gearRatio);
            return this;
        }

        /**
         * Set the track width in encoder ticks.
         *
         * @param trackWidthTicks the track width in encoder ticks as a result of a tuned measurement
         * @return the builder
         */
        @NonNull
        public Builder setTrackWidthTicks(double trackWidthTicks) {
            model.trackWidthTicks = trackWidthTicks;
            return this;
        }

        /**
         * Finalise the configuration and build the DriveModel.
         *
         * @return the DriveModel
         */
        @NonNull
        public DriveModel build() {
            return model;
        }
    }
}
