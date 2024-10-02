package org.murraybridgebunyips.bunyipslib.tasks;

import com.acmerobotics.roadrunner.path.Path;

import org.murraybridgebunyips.bunyipslib.BunyipsSubsystem;
import org.murraybridgebunyips.bunyipslib.PurePursuit;
import org.murraybridgebunyips.bunyipslib.Text;
import org.murraybridgebunyips.bunyipslib.tasks.bases.Task;

import java.util.function.Supplier;

/**
 * Task for running Pure Pursuit paths using the BunyipsOpMode Task system.
 * This task is internally used by the {@link PurePursuit.PathMaker} to run the path.
 * <p>
 * <b>Note!</b> Unlike the other drive tasks, this task does not automatically attach itself to a {@link BunyipsSubsystem}
 * on construction, and needs to be done manually via the {@code onSubsystem} method, or can be omitted if there are
 * no default tasks for the drive during execution (e.g. only action in Autonomous).
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
     * @param path   the path to follow
     */
    public PurePursuitTask(PurePursuit runner, Path path) {
        this.runner = runner;
        pathBuilder = () -> path;
        withName(Text.format("Pure Pursuit Path %::%", path.start(), path.end()));
        // Detach for all future automatic runtime of the runner if we're using a task system
        opMode(o -> o.detachActiveLoopRunnables(runner));
    }

    /**
     * Create a new deferred Pure Pursuit task with a path to be constructed when this task is run.
     *
     * @param runner    the Pure Pursuit runner to use
     * @param pathMaker the path maker to construct the path when this task is run
     */
    public PurePursuitTask(PurePursuit runner, PurePursuit.PathMaker pathMaker) {
        this.runner = runner;
        pathBuilder = pathMaker::buildPath;
        withName("Pure Pursuit Path");
        opMode(o -> o.detachActiveLoopRunnables(runner));
    }

    @Override
    protected void init() {
        path = pathBuilder.get();
        runner.followPath(path);
    }

    @Override
    protected void periodic() {
        runner.run();
    }

    @Override
    protected boolean isTaskFinished() {
        return !runner.isBusy() || runner.getCurrentPath() != path;
    }
}
