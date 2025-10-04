package au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.decode;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.external.units.Units.Milliseconds;

import java.util.ArrayList;
import java.util.concurrent.Callable;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.BunyipsOpMode;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.executables.Periodic;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Threads;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.data.AprilTagData;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.vision.processors.AprilTag;

/**
 * Consistently runs a detection to find the Motif AprilTag. Once found, this task ends
 * and the result is returned.
 * <p>
 * This finder is designed to be used by the {@link Threads} system, and results returned
 * via the {@link Threads.Result} interface. Will log to Logcat and BOM telemetry logs on completion if available.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public class MotifFinder implements Callable<Motif> {
    /**
     * The rate at which the thread should check for updates in the AprilTag data list.
     * Adjusting this value can assist in filtering out brief appearances of the Obelisk tags to ensure a stable result.
     * <p>
     * Changing this value requires the Motif Finder to be reconstructed/restarted.
     */
    public static int POLL_MS = 100;

    private final AprilTag processor;
    private final Periodic poll;

    private Motif motif;

    /**
     * Creates a new MotifFinder.
     *
     * @param processor the AprilTag processor to use. Must be attached to a Vision instance (initialised).
     */
    public MotifFinder(AprilTag processor) {
        if (processor == null || !processor.isAttached())
            throw new IllegalStateException("AprilTag processor not attached! Attach the processor through a Vision instance first.");
        this.processor = processor;
        poll = new Periodic(Milliseconds.of(POLL_MS), () -> {
            ArrayList<AprilTagData> tags = this.processor.getData();
            for (AprilTagData tag : tags) {
                for (Motif motif : Motif.values()) {
                    if (motif.aprilTagId == tag.getId()) {
                        this.motif = motif;
                        return;
                    }
                }
            }
        });
    }

    @Override
    public Motif call() {
        if (poll == null) return null;
        while (true) {
            poll.run();
            if (motif != null) {
                Dbg.log(MotifFinder.class, "found Motif: %", motif);
                BunyipsOpMode.ifRunning(o -> o.telemetry.log(MotifFinder.class, "found Motif: %", motif));
                return motif;
            }
        }
    }
}
