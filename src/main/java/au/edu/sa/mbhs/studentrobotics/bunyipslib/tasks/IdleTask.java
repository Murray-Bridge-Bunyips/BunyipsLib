package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Task that runs forever but does nothing.
 * This is used as a default task on subsystems that don't have a default task.
 * IdleTask is ignored from subsystem report telemetry via a string match to "IdleTask".
 *
 * @since 1.0.0-pre
 */
public class IdleTask extends Task {
    @Override
    protected void init() {
        // Ensure the name remains the same for telemetry ignoring
        withName("IdleTask");
    }

    @Override
    protected void periodic() {
        // no-op
    }

    @Override
    protected boolean isTaskFinished() {
        return false;
    }
}
