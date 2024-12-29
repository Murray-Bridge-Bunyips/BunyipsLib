package au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.tasks.bases.Task;

/**
 * Task that runs forever but does nothing.
 * This is used as a default task on subsystems that don't have a default task.
 * IdleTask instances are matched by the name "Idle".
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class IdleTask extends Task {
    // no-op
}
