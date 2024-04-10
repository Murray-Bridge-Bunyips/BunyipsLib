package org.murraybridgebunyips.bunyipslib.tasks.bases;

import static org.murraybridgebunyips.bunyipslib.external.units.Units.Seconds;

import androidx.annotation.NonNull;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;

/**
 * A task that runs with no timeout, finish condition must be controlled by the user.
 */
public abstract class NoTimeoutTask extends Task {
    protected NoTimeoutTask() {
        super(Seconds.zero());
    }

    protected NoTimeoutTask(@NonNull BunyipsSubsystem dependencySubsystem, boolean override) {
        super(Seconds.zero(), dependencySubsystem, override);
    }
}
