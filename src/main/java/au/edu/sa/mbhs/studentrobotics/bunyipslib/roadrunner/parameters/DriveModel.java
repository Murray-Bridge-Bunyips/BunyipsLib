package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.parameters;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Inches;

import androidx.annotation.NonNull;

import java.util.ArrayList;

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
        private final ArrayList<Double> axialMeasurements = new ArrayList<>();
        private final ArrayList<Double> lateralMeasurements = new ArrayList<>();

        /**
         * Set the inches per tick for the drive as a result of a tuned measurement ratio.
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
         *
         * @param inPerTick the inches per tick for the drive calculated as (real dist in inches / ticks reported)
         * @return the builder
         */
        @NonNull
        public Builder setInPerTick(double inPerTick) {
            axialMeasurements.add(inPerTick);
            return this;
        }

        /**
         * Set the inches per tick for the drive as a result of movement and encoder ticks.
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
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
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
         * Do also note precomputed ratios are not recommended for determining the averages, empirical values are far more accurate.
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
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the lateral inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
         * <p>
         * Do also note that empirically tuned values will have far greater accuracy and should not be used if making averages.
         *
         * @param lateralInPerTick the inches per tick for lateral movement
         * @return the builder
         */
        @NonNull
        public Builder setLateralInPerTick(double lateralInPerTick) {
            lateralMeasurements.add(lateralInPerTick);
            return this;
        }

        /**
         * Set the inches per tick for lateral movement as a result of movement and encoder ticks.
         * Not required for a differential drive. By default, this is set to the same value as inches per tick.
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the lateral inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
         * Using an average for lateral inches per tick is only really recommended if using the LateralPushTest.
         *
         * @param measuredDistance the measured distance of the movement
         * @param reportedTicks    the reported ticks from the encoder when the distance was measured
         * @return the builder
         */
        @NonNull
        public Builder setLateralDistPerTick(@NonNull Measure<Distance> measuredDistance, double reportedTicks) {
            setLateralInPerTick(measuredDistance.in(Inches) / reportedTicks);
            return this;
        }

        /**
         * Set the inches per tick for lateral movement as a result of a precomputed ratio.
         * Not required for a differential drive. By default, this is set to the same value as inches per tick.
         * <p>
         * <b>Note</b>: Calling this function multiple times will take the average between other calls that have set
         * the lateral inches per tick. This allows for multiple tuning session results to average in on the most accurate value.
         * Using an average for lateral inches per tick is only really recommended if using the LateralPushTest.
         * <p>
         * Do also note that empirically tuned values will have far greater accuracy and should not be used if making averages.
         *
         * @param ticksPerRevolution the ticks per revolution of the encoder
         * @param gearRatio          the gear ratio of the drive
         * @param wheelDiameter      the diameter of the wheel
         * @return the builder
         */
        @NonNull
        public Builder setComputedLateralInPerTick(double ticksPerRevolution, double gearRatio, @NonNull Measure<Distance> wheelDiameter) {
            setLateralInPerTick((wheelDiameter.in(Inches) * Math.PI) / (ticksPerRevolution * gearRatio));
            return this;
        }

        /**
         * Set the track width in encoder ticks.
         * <p>
         * Multiple calls of this function will not take an average between the previous calls
         * (as the track width tuning relies on accurate values of kS and kV, rather than observed push tests)
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
            model.inPerTick = !axialMeasurements.isEmpty()
                    ? axialMeasurements.stream().reduce(0.0, Double::sum) / axialMeasurements.size()
                    : 0.0;
            model.lateralInPerTick = !lateralMeasurements.isEmpty()
                    ? lateralMeasurements.stream().reduce(0.0, Double::sum) / lateralMeasurements.size()
                    : model.inPerTick;
            return model;
        }
    }
}
