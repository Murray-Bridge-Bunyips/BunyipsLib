package au.edu.sa.mbhs.studentrobotics.bunyipslib.subsystems;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Servo;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsSubsystem;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.DualTelemetry;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.Mathf;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Measure;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.ServoEx;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Lambda;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Controls a set of two servos together to one of two static setpoints for each servo.
 *
 * @author Lucas Bubner, 2023
 * @since 1.0.0-pre
 */
public class DualServos extends BunyipsSubsystem {
    /**
     * Tasks for DualServos.
     */
    public final Tasks tasks = new Tasks();
    private final double leftClosed;
    private final double leftOpen;
    private final double rightClosed;
    private final double rightOpen;
    private Servo left;
    private Servo right;
    private double leftServoPosition;
    private double rightServoPosition;

    /**
     * Create a new DualServos.
     *
     * @param left        the left servo
     * @param right       the right servo
     * @param leftClosed  the left servo closed position
     * @param leftOpen    the left servo open position
     * @param rightClosed the right servo closed position
     * @param rightOpen   the right servo open position
     */
    public DualServos(@Nullable Servo left, @Nullable Servo right, double leftClosed, double leftOpen, double rightClosed, double rightOpen) {
        if (leftClosed == leftOpen || rightClosed == rightOpen)
            throw new IllegalArgumentException("Open and close positions for either servo cannot be the same");

        this.leftClosed = Mathf.clamp(leftClosed, 0, 1);
        this.leftOpen = Mathf.clamp(leftOpen, 0, 1);
        this.rightClosed = Mathf.clamp(rightClosed, 0, 1);
        this.rightOpen = Mathf.clamp(rightOpen, 0, 1);

        if (!assertParamsNotNull(left, right)) return;
        this.left = left;
        this.right = right;

        // Always close on init
        // Note: Updating must be done manually
        leftServoPosition = leftClosed;
        rightServoPosition = rightClosed;
    }

    /**
     * Create a new DualServos with default open and close positions.
     * This relies on that your servos are configured at the hardware level to move in the correct directions.
     *
     * @param left  the left servo
     * @param right the right servo
     */
    public DualServos(@Nullable Servo left, @Nullable Servo right) {
        this(left, right, 0, 1, 0, 1);
    }

    /**
     * Toggle the state of the servos.
     *
     * @param servo the servo to toggle
     * @return this
     */
    @NonNull
    public DualServos toggle(@NonNull ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = (leftServoPosition == leftOpen) ? leftClosed : leftOpen;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = (rightServoPosition == rightOpen) ? rightClosed : rightOpen;
            return this;
        }
        leftServoPosition = (leftServoPosition == leftOpen) ? leftClosed : leftOpen;
        rightServoPosition = (rightServoPosition == rightOpen) ? rightClosed : rightOpen;
        return this;
    }

    /**
     * Open the servos.
     *
     * @param servo the servo to open
     * @return this
     */
    @NonNull
    public DualServos open(@NonNull ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = leftOpen;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = rightOpen;
            return this;
        }
        leftServoPosition = leftOpen;
        rightServoPosition = rightOpen;
        return this;
    }


    /**
     * Close the servos.
     *
     * @param servo the servo to close
     * @return this
     */
    @NonNull
    public DualServos close(@NonNull ServoSide servo) {
        if (servo == ServoSide.LEFT) {
            leftServoPosition = leftClosed;
            return this;
        }
        if (servo == ServoSide.RIGHT) {
            rightServoPosition = rightClosed;
            return this;
        }
        leftServoPosition = leftClosed;
        rightServoPosition = rightClosed;
        return this;
    }

    /**
     * Query if a servo is open.
     *
     * @param servo the servo to query
     * @return whether the servo side is open
     */
    public boolean isOpen(@NonNull ServoSide servo) {
        return switch (servo) {
            case LEFT -> left.getPosition() == leftOpen;
            case RIGHT -> right.getPosition() == rightOpen;
            case BOTH -> left.getPosition() == leftOpen && right.getPosition() == rightOpen;
        };
    }

    /**
     * Push stateful changes to the servos.
     */
    @Override
    protected void periodic() {
        left.setPosition(leftServoPosition);
        right.setPosition(rightServoPosition);
        DualTelemetry.smartAdd(toString(), "Left->% Right->%",
                leftServoPosition == leftOpen ? "<font color='yellow'>OPEN</font>" : "<font color='green'>CLOSE</font>",
                rightServoPosition == rightOpen ? "<font color='yellow'>OPEN</font>" : "<font color='green'>CLOSE</font>");
    }

    /**
     * Represents the side of the servos to control.
     */
    public enum ServoSide {
        /**
         * The left servo.
         */
        LEFT,
        /**
         * The right servo.
         */
        RIGHT,
        /**
         * Both servos.
         */
        BOTH
    }

    /**
     * DualServo tasks, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Create a task to open the left servo.
         *
         * @return the task
         */
        @NonNull
        public Task openLeft() {
            return new Lambda((t) -> {
                t.timeout = ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftOpen);
                open(ServoSide.LEFT);
            }).on(DualServos.this, true)
                    .timeout(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftOpen)) // preliminary
                    .named(name + ":Open Left");
        }

        /**
         * Create a task to open the right servo.
         *
         * @return the task
         */
        @NonNull
        public Task openRight() {
            return new Lambda((t) -> {
                t.timeout = ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightOpen);
                open(ServoSide.RIGHT);
            })
                    .on(DualServos.this, true)
                    .timeout(ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightOpen)) // preliminary
                    .named(name + ":Open Right");
        }

        /**
         * Create a task to open both servos.
         *
         * @return the task
         */
        @NonNull
        public Task openBoth() {
            return new Lambda((t) -> {
                t.timeout(Measure.max(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftOpen),
                        ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightOpen)));
                open(ServoSide.BOTH);
            })
                    .on(DualServos.this, true)
                    .timeout(Measure.max(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftOpen),
                            ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightOpen))) // preliminary
                    .named(name + ":Open Both");
        }

        /**
         * Create a task to close the left servo.
         *
         * @return the task
         */
        @NonNull
        public Task closeLeft() {
            return new Lambda((t) -> {
                t.timeout = ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftClosed);
                close(ServoSide.LEFT);
            }).on(DualServos.this, true)
                    .timeout(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftClosed)) // preliminary
                    .named(name + ":Close Left");
        }

        /**
         * Create a task to close the right servo.
         *
         * @return the task
         */
        @NonNull
        public Task closeRight() {
            return new Lambda((t) -> {
                t.timeout = ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightClosed);
                close(ServoSide.RIGHT);
            })
                    .on(DualServos.this, true)
                    .timeout(ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightClosed)) // preliminary
                    .named(name + ":Close Right");
        }

        /**
         * Create a task to close both servos.
         *
         * @return the task
         */
        @NonNull
        public Task closeBoth() {
            return new Lambda((t) -> {
                t.timeout(Measure.max(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftClosed),
                        ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightClosed)));
                close(ServoSide.BOTH);
            })
                    .on(DualServos.this, true)
                    .timeout(Measure.max(ServoEx.tryGetEndToEndTime(left, leftServoPosition, leftClosed),
                            ServoEx.tryGetEndToEndTime(right, rightServoPosition, rightClosed))) // preliminary
                    .named(name + ":Close Both");
        }

        // TODO
        /**
         * Create a task to toggle the left servo.
         *
         * @return the task
         */
        @NonNull
        public Task toggleLeft() {
            return new Lambda(() -> toggle(ServoSide.LEFT))
                    .on(DualServos.this, true)
                    .named(name + ":Toggle Left");
        }

        /**
         * Create a task to toggle the right servo.
         *
         * @return the task
         */
        @NonNull
        public Task toggleRight() {
            return new Lambda(() -> toggle(ServoSide.RIGHT))
                    .on(DualServos.this, true)
                    .named(name + ":Toggle Right");
        }

        /**
         * Create a task to toggle both servos.
         *
         * @return the task
         */
        @NonNull
        public Task toggleBoth() {
            return new Lambda(() -> toggle(ServoSide.BOTH))
                    .on(DualServos.this, true)
                    .named(name + ":Toggle Both");
        }
    }
}
