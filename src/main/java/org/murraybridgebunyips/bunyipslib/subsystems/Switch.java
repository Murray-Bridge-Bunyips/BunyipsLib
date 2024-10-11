package org.murraybridgebunyips.bunyipslib.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.external.Mathf;
import org.murraybridgebunyips.bunyipslib.tasks.ContinuousTask;
import org.murraybridgebunyips.bunyipslib.tasks.RunTask;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.DoubleSupplier;

/**
 * A generic servo controller subsystem that may be used to hold two positions and to control movements in between
 * these setpoints. Includes a built-in delta step configuration that may be used to control servo speed.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class Switch extends BunyipsSubsystem {
    /**
     * Tasks for Cannon.
     */
    public final Tasks tasks = new Tasks();

    private final Servo servo;
    private double target;
    private double openPosition = 1;
    private double closePosition = 0;

    /**
     * Constructs a new Switch.
     *
     * @param servo         the servo to use
     * @param openPosition  the position to set the servo to when open
     * @param closePosition the position to set the servo to when closed
     */
    public Switch(Servo servo, double openPosition, double closePosition) {
        this.servo = servo;
        // Auto-close servos
        target = closePosition;
        withName("Switch");
        setBounds(openPosition, closePosition);
    }

    /**
     * Constructs a new Switch with default open and close positions.
     *
     * @param servo the servo to use
     */
    public Switch(Servo servo) {
        this(servo, 1, 0);
    }

    /**
     * Set the new bounds this servo should respect for opened and closed positions.
     *
     * @param openPosition  the open position to respect
     * @param closePosition the closed position to respect
     */
    public void setBounds(double openPosition, double closePosition) {
        this.openPosition = Mathf.clamp(openPosition, 0, 1);
        this.closePosition = Mathf.clamp(closePosition, 0, 1);
    }

    /**
     * Set the new open position.
     *
     * @param openPosition the open position to respect
     */
    public void setOpenBound(double openPosition) {
        this.openPosition = Mathf.clamp(openPosition, 0, 1);
    }

    /**
     * Set the new closed position.
     *
     * @param closePosition the closed position to respect
     */
    public void setClosedBound(double closePosition) {
        this.closePosition = Mathf.clamp(closePosition, 0, 1);
    }

    /**
     * Open the switch.
     */
    public void open() {
        target = openPosition;
    }

    /**
     * Close the switch.
     */
    public void close() {
        target = closePosition;
    }

    /**
     * Toggles the switch, moving it to whichever is closest between open and close.
     */
    public void toggle() {
        double pos = servo.getPosition();
        double distanceToOpen = Math.abs(pos - openPosition);
        double distanceToClose = Math.abs(pos - closePosition);

        if (distanceToOpen < distanceToClose) {
            open();
        } else {
            close();
        }
    }

    /**
     * Set a custom position that is clipped between the closed and open bounds.
     *
     * @param position the position to set to, clipped between open and close
     */
    public void setPositionClipped(double position) {
        target = Mathf.clamp(position, Math.min(openPosition, closePosition), Math.max(openPosition, closePosition));
    }

    /**
     * Set a custom position that is not affected by the closed and open bounds.
     *
     * @param position the raw position to send to the servo
     */
    public void setPosition(double position) {
        target = Mathf.clamp(position, 0, 1);
    }

    /**
     * @return the current servo target
     */
    public double getTarget() {
        return target;
    }

    /**
     * Query whether the switch is open.
     *
     * @return whether the switch is open
     */
    public boolean isOpen() {
        return servo.getPosition() == openPosition;
    }

    /**
     * Query whether the switch is closed.
     *
     * @return whether the switch is closed
     */
    public boolean isClosed() {
        return servo.getPosition() == closePosition;
    }

    @Override
    protected void periodic() {
        opMode(o -> o.telemetry.add("%: %", name, target == openPosition ? "<font color='orange'><b>OPEN</b></font>" : target == closePosition ? "<font color='green'>CLOSE</font>" : "<font color='white'>" + target + "/1.0</font>"));
        servo.setPosition(target);
    }

    /**
     * Tasks for Switch, access with {@link #tasks}.
     */
    public class Tasks {
        /**
         * Move the servo with a supplier of position. Should be a default task.
         *
         * @param positionSupplier the position value supplier
         * @return a task to continually move the switch to the position supply
         */
        public Task controlPosition(DoubleSupplier positionSupplier) {
            return new ContinuousTask(() -> setPosition(positionSupplier.getAsDouble()))
                    .onSubsystem(Switch.this, false)
                    .withName("Supplier Position Control");
        }

        /**
         * Move the servo with a supplier of power/delta. Should be a default task.
         *
         * @param powerSupplier the power/delta supplier
         * @return a task to continually move the switch in accordance with the delta step
         */
        public Task controlDelta(DoubleSupplier powerSupplier) {
            return new ContinuousTask(() -> setPosition(servo.getPosition() + powerSupplier.getAsDouble()))
                    .onSubsystem(Switch.this, false)
                    .withName("Supplier Delta Control");
        }

        /**
         * Open the switch.
         *
         * @return Open switch task.
         */
        public Task open() {
            return new RunTask(Switch.this::open)
                    .onSubsystem(Switch.this, true)
                    .withName("Open");
        }

        /**
         * Close the switch.
         *
         * @return Close switch task.
         */
        public Task close() {
            return new RunTask(Switch.this::close)
                    .onSubsystem(Switch.this, true)
                    .withName("Close");
        }

        /**
         * Toggle the switch
         *
         * @return Toggle switch task
         */
        public Task toggle() {
            return new RunTask(Switch.this::toggle)
                    .onSubsystem(Switch.this, true)
                    .withName("Toggle");
        }

        /**
         * Set a custom position that is clipped between the closed and open bounds.
         *
         * @param position the position to set to, clipped between open and close
         * @return a task to perform this action
         */
        public Task setClipped(double position) {
            return new RunTask(() -> setPositionClipped(position))
                    .onSubsystem(Switch.this, true)
                    .withName("Go To " + Mathf.clamp(position, Math.min(closePosition, openPosition), Math.max(closePosition, openPosition)) + "/1.0");
        }

        /**
         * Set a custom position that is not affected by the closed and open bounds.
         *
         * @param position the raw position to send to the servo
         * @return a task to perform this action
         */
        public Task setTo(double position) {
            return new RunTask(() -> setPosition(position))
                    .onSubsystem(Switch.this, true)
                    .withName("Go To " + Mathf.clamp(position, Math.min(closePosition, openPosition), Math.max(closePosition, openPosition)) + "/1.0");
        }

        /**
         * Delta the current servo position. This task is clamped between the closed and open positions.
         *
         * @param delta the amount to delta the servo by
         * @return a task to perform this action
         */
        public Task deltaClipped(double delta) {
            return new RunTask(() -> setPositionClipped(servo.getPosition() + delta))
                    .onSubsystem(Switch.this, true)
                    .withName("Delta By " + delta);
        }

        /**
         * Delta the current servo position. This task is not affected by the closed and open bounds.
         *
         * @param delta the amount to delta the servo by, unclamped between the closed and open bounds
         * @return a task to perform this action
         */
        public Task delta(double delta) {
            return new RunTask(() -> setPosition(servo.getPosition() + delta))
                    .onSubsystem(Switch.this, true)
                    .withName("Delta By " + delta);
        }
    }
}
