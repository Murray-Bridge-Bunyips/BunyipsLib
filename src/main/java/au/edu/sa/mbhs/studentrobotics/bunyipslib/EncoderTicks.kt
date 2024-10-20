package au.edu.sa.mbhs.studentrobotics.bunyipslib

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Angle
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Distance
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Revolutions
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Second
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Velocity
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Motor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import kotlin.math.roundToInt

/**
 * Utility class for converting between SI units and encoder ticks.
 *
 * @author Lucas Bubner, 2024
 * @since 3.0.0
 */
object EncoderTicks {
    /**
     * Converts encoder ticks to inches for use in calculation with a Localizer.
     *
     * @param ticks the encoder ticks
     * @param wheelRadiusInches the wheel radius in inches
     * @param gearRatio the gear ratio as a division ratio
     * @param ticksPerRevolution the ticks per revolution of the encoder
     * @return inches travelled as per circumference
     */
    @JvmStatic
    fun toInches(ticks: Number, wheelRadiusInches: Double, gearRatio: Double, ticksPerRevolution: Number): Double {
        return wheelRadiusInches * 2 * Math.PI * gearRatio * ticks.toDouble() / ticksPerRevolution.toDouble()
    }

    /**
     * Convert encoder ticks to an angle.
     *
     * @param ticks The number of encoder ticks.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The angle.
     */
    @JvmStatic
    fun toAngle(ticks: Number, ticksPerRevolution: Number, reduction: Double): Measure<Angle> {
        return Revolutions.of(ticks.toDouble() / ticksPerRevolution.toDouble()).times(reduction)
    }

    /**
     * Convert an angle to encoder ticks.
     *
     * @param angle The angle.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The number of encoder ticks.
     */
    @JvmStatic
    fun fromAngle(angle: Measure<Angle>, ticksPerRevolution: Number, reduction: Double): Int {
        // Equation: angle (in revolutions) * ticksPerRevolution * reduction = ticks
        return (angle.`in`(Revolutions) * ticksPerRevolution.toDouble() * reduction).roundToInt()
    }

    /**
     * Convert encoder ticks to a distance.
     *
     * @param ticks The number of encoder ticks.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param wheelDiameter The diameter of the wheel.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The distance.
     */
    @JvmStatic
    fun toDistance(
        ticks: Number,
        ticksPerRevolution: Number,
        wheelDiameter: Measure<Distance>,
        reduction: Double
    ): Measure<Distance> {
        // Equation: circumference (2*pi*r_m) * (encoder ticks / ticksPerRevolution) = distance
        return wheelDiameter
            .times(Math.PI)
            .times(ticks.toDouble() / ticksPerRevolution.toDouble())
            .times(reduction)
    }

    /**
     * Convert a distance to encoder ticks.
     *
     * @param distance The distance.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param wheelDiameter The diameter of the wheel.
     * @param reduction The gear reduction (input speed / output speed).
     * @return The number of encoder ticks.
     */
    @JvmStatic
    fun fromDistance(
        distance: Measure<Distance>,
        ticksPerRevolution: Number,
        wheelDiameter: Measure<Distance>,
        reduction: Double
    ): Int {
        // Equation: distance * ticksPerRevolution / circumference (2*pi*r_m) = ticks
        return distance
            .times(ticksPerRevolution.toDouble())
            .divide(wheelDiameter.times(Math.PI))
            .times(reduction)
            .magnitude()
            .roundToInt()
    }

    /**
     * Create a dynamic generator for encoder tick conversions based on a motor.
     * This will try to automatically determine the number of encoder ticks per revolution, if this is not provided,
     * use the other constructor.
     *
     * @param motor The motor.
     * @param reduction The gear reduction (input speed / output speed). If not provided, the reduction will be 1.
     * @param wheelDiameter The diameter of the wheel. If not provided, distance conversions will not be available.
     * @return The encoder tick generator.
     */
    @JvmStatic
    @JvmOverloads
    fun createGenerator(
        motor: DcMotor,
        reduction: Double = 1.0,
        wheelDiameter: Measure<Distance>? = null
    ): Generator {
        return Generator(motor, motor.motorType.ticksPerRev, reduction, wheelDiameter)
    }

    /**
     * Create a dynamic generator for encoder tick conversions based on a motor.
     *
     * @param motor The motor.
     * @param ticksPerRevolution The number of encoder ticks per revolution.
     * @param reduction The gear reduction (input speed / output speed). If not provided, the reduction will be 1.
     * @param wheelDiameter The diameter of the wheel. If not provided, distance conversions will not be available.
     * @return The encoder tick generator.
     */
    @JvmStatic
    @JvmOverloads
    fun createGenerator(
        motor: DcMotor,
        ticksPerRevolution: Number,
        reduction: Double = 1.0,
        wheelDiameter: Measure<Distance>? = null
    ): Generator {
        return Generator(motor, ticksPerRevolution, reduction, wheelDiameter)
    }

    /**
     * Dynamic generator for encoder tick conversions based on a motor.
     */
    class Generator(
        motor: DcMotor,
        private val ticksPerRevolution: Number,
        private val reduction: Double,
        private val wheelDiameter: Measure<Distance>?
    ) {
        private val motorEx: DcMotorEx = motor as DcMotorEx

        /**
         * Get the current encoder ticks in an angle.
         */
        fun getAngle(): Measure<Angle> {
            return toAngle(motorEx.currentPosition, ticksPerRevolution, reduction)
        }

        /**
         * Get the angular velocity of the encoder.
         */
        fun getAngularVelocity(): Measure<Velocity<Angle>> {
            return toAngle(motorEx.velocity, ticksPerRevolution, reduction).per(Second)
        }

        /**
         * Get the angular acceleration of the encoder.
         * Note that the motor attached to this generator *must* be an instance of [Motor], or an exception will be thrown.
         */
        fun getAngularAcceleration(): Measure<Velocity<Velocity<Angle>>> {
            if (motorEx !is Motor) {
                throw IllegalStateException("Motor attached to this generator is not a Motor instance. Acceleration information is not available.")
            }
            return toAngle(motorEx.acceleration, ticksPerRevolution, reduction).per(Second).per(Second)
        }

        /**
         * Get the current encoder ticks in a distance.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun getDistance(): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(motorEx.currentPosition, ticksPerRevolution, it, reduction) }
        }

        /**
         * Get the current encoder velocity in a distance.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun getVelocity(): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(motorEx.velocity, ticksPerRevolution, it, reduction) }
        }

        /**
         * Get the current encoder acceleration in a distance.
         * Note that the motor attached to this generator *must* be an instance of [Motor], or an exception will be thrown.
         */
        fun getAcceleration(): Measure<Velocity<Velocity<Distance>>>? {
            if (motorEx !is Motor) {
                throw IllegalStateException("Motor attached to this generator is not a Motor instance. Acceleration information is not available.")
            }
            return wheelDiameter?.let {
                toDistance(motorEx.acceleration, ticksPerRevolution, it, reduction)
                    .per(Second).per(Second)
            }
        }

        /**
         * Get an angle from the provided encoder ticks.
         */
        fun angle(ticks: Int): Measure<Angle> {
            return toAngle(ticks, ticksPerRevolution, reduction)
        }

        /**
         * Get a distance from the provided encoder ticks.
         * @return the distance, else null if the wheel diameter is not provided.
         */
        fun distance(ticks: Int): Measure<Distance>? {
            return wheelDiameter?.let { toDistance(ticks, ticksPerRevolution, it, reduction) }
        }

        /**
         * Get encoder ticks from the provided angle.
         */
        fun ticks(angle: Measure<Angle>): Int {
            return fromAngle(angle, ticksPerRevolution, reduction)
        }

        /**
         * Get encoder ticks from the provided distance.
         * @return the number of encoder ticks, else null if the wheel diameter is not provided.
         */
        fun ticks(distance: Measure<Distance>): Int? {
            return wheelDiameter?.let { fromDistance(distance, ticksPerRevolution, it, reduction) }
        }
    }
}