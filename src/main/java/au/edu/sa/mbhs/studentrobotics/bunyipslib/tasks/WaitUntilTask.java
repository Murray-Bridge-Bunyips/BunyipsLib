package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import androidx.annotation.NonNull;

import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * A task that waits until a condition is true before finishing.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class WaitUntilTask extends Task {
    private final BooleanSupplier condition;

    /**
     * Create a new WaitUntilTask with the given condition.
     *
     * @param condition the condition to wait for true
     */
    public WaitUntilTask(@NonNull BooleanSupplier condition) {
        this.condition = condition;
        withName("Wait Until");
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        return condition.getAsBoolean();
    }
}
