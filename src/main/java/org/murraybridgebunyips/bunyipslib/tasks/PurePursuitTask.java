package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.roadrunner.path.Path;

import org.murraybridgebunyips.bunyipslib.PurePursuit;
import org.murraybridgebunyips.bunyipslib.Text;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.Supplier;

/**
 * Task for running Pure Pursuit paths using the BunyipsOpMode Task system.
 * This task is internally used by the {@link PurePursuit.PathMaker} to run the path.
 *
 * @author Lucas Bubner, 2024
 * @since 5.1.0
 */
public class PurePursuitTask extends Task {
    private final PurePursuit runner;
    private final Supplier<Path> pathBuilder;

    private Path path;

    /**
     * Create a new Pure Pursuit task to follow the given constructed path.
     *
     * @param runner the Pure Pursuit runner to use
     * @param path the path to follow
     */
    public PurePursuitTask(PurePursuit runner, Path path) {
        this.runner = runner;
        pathBuilder = () -> path;
        withName(Text.format("Pure Pursuit Path %::%", path.start(), path.end()));
    }

    /**
     * Create a new Pure Pursuit task that will be constructed when this task is run.
     *
     * @param runner the Pure Pursuit runner to use
     * @param pathMaker the path maker to construct the path when this task is run
     */
    public PurePursuitTask(PurePursuit runner, PurePursuit.PathMaker pathMaker) {
        this.runner = runner;
        pathBuilder = pathMaker::buildPathNow;
        withName("Pure Pursuit Path");
    }

    @Override
    protected void init() {
        path = pathBuilder.get();
        runner.followPath(path);
    }

    @Override
    protected void periodic() {
        // Shouldn't update twice in one loop if we're attached to a BOM loop
        if (!runner.ATTACHED_TO_BOM_LOOP)
            runner.run();
    }

    @Override
    protected boolean isTaskFinished() {
        return !runner.isBusy() || runner.getCurrentPath() != path;
    }
}
