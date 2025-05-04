package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.function.DoubleSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Time;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Controls a generic {@link DcMotorSimple} to rotate.
 *
 * @author Lucas Bubner, 2025
 * @see HoldableActuator
 * @since 7.2.0
 */
public class Actuator extends BunyipsSubsystem {
    /**
     * Tasks for Actuator.
     */
    public final Tasks tasks = new Tasks();

    private final DcMotorSimple actuator;
    private volatile double power;

    /**
     * Create a new Actuator.
     *
     * @param actuator the {@link DcMotorSimple} (e.g. {@link CRServo}, {@link DcMotor} without encoder) to control
     */
    public Actuator(DcMotorSimple actuator) {
        assertParamsNotNull(actuator);
        this.actuator = actuator;
    }

    /**
     * Set the power output for the actuator.
     *
     * @param power power level in domain [-1.0, 1.0], will be clamped
     * @return this
     */
    public Actuator setPower(double power) {
        this.power = power;
        return this;
    }

    @Override
    protected void onDisable() {
        power = 0;
        actuator.setPower(0);
    }

    @Override
    protected void periodic() {
        actuator.setPower(power);
    }

    /**
     * Tasks for Actuator, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Controls the actuator with a continuous supplier of power. Should be allocated as a default task if used.
         *
         * @param powerSupplier the power value supplier
         * @return a task to move the actuator
         */
        @NonNull
        public Task control(@NonNull DoubleSupplier powerSupplier) {
            return Task.task().periodic(() -> Actuator.this.setPower(powerSupplier.getAsDouble()))
                    .onFinish(() -> power = 0)
                    .on(Actuator.this, false)
                    .named(forThisSubsystem("Power Target Control"));
        }

        /**
         * Instantly set the power of the actuator.
         *
         * @param pwr the power to set
         * @return a one-shot task to set the power instantly then end
         */
        @NonNull
        public Task setPower(double pwr) {
            return new Lambda(() -> power = pwr)
                    .on(Actuator.this, false)
                    .named(forThisSubsystem("Set Power to " + pwr));
        }

        /**
         * Run the actuator for a certain amount of time.
         *
         * @param time the time to run for
         * @param pwr  the power to run at
         * @return a task to run the actuator
         */
        @NonNull
        public Task runFor(@NonNull Measure<Time> time, double pwr) {
            return Task.task().periodic(() -> power = pwr)
                    .onFinish(() -> power = 0)
                    .on(Actuator.this, false)
                    .timeout(time)
                    .named(forThisSubsystem("Run For " + time));
        }
    }
}
