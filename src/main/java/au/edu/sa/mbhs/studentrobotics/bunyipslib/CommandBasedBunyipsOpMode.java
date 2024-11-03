package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.hardware.Controller;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.NullSafety;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Command-based structure for a {@link BunyipsOpMode} utilising the Scheduler.
 * This can be used for seamless/zero-step integration with the Scheduler in TeleOp, for Autonomous it is
 * recommended to use the {@link AutonomousBunyipsOpMode} classes as Tasks there are used in a different context.
 *
 * @author Lucas Bubner, 2024
 * @see BunyipsOpMode
 * @since 1.0.0-pre
 */
public abstract class CommandBasedBunyipsOpMode extends BunyipsOpMode {
    /**
     * The Scheduler instance manages all operations and commands for the CommandBasedBunyipsOpMode.
     * <p>
     * Tasks may be scheduled and managed through this instance, with setup and subsystem management
     * being handled automatically. Common scheduling that are used in the Scheduler are available here as a proxy.
     */
    @NonNull
    public final Scheduler scheduler = new Scheduler();
    @NonNull
    private HashSet<BunyipsSubsystem> managedSubsystems = new HashSet<>();

    /**
     * Create a new controller button trigger creator.
     * <p>
     * For Kotlin users, calling this method can be done with the notation {@code `when`}
     * (see <a href="https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin">here</a>),
     * or by calling the alias {@code on}.
     *
     * @param user The Controller instance to use.
     * @return The controller button trigger creator.
     */
    @NonNull
    @SuppressLint("NoHardKeywords")
    public Scheduler.ControllerTriggerCreator when(@NonNull Controller user) {
        return scheduler.when(user);
    }

    /**
     * Create a new controller button trigger creator.
     *
     * @param user The Controller instance to use.
     * @return The controller button trigger creator.
     */
    @NonNull
    public Scheduler.ControllerTriggerCreator on(@NonNull Controller user) {
        return scheduler.when(user);
    }

    /**
     * Create a new controller button trigger creator for the driver.
     *
     * @return The controller button trigger creator.
     */
    @NonNull
    public Scheduler.ControllerTriggerCreator driver() {
        return scheduler.driver();
    }

    /**
     * Create a new controller button trigger creator for the operator.
     *
     * @return The controller button trigger creator.
     */
    @NonNull
    public Scheduler.ControllerTriggerCreator operator() {
        return scheduler.operator();
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     * <p>
     * For Kotlin users, calling this method can be done with the notation {@code `when`}
     * (see <a href="https://kotlinlang.org/docs/java-interop.html#escaping-for-java-identifiers-that-are-keywords-in-kotlin">here</a>),
     * or by calling the alias {@code on}.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @NonNull
    @SuppressLint("NoHardKeywords")
    public Scheduler.ScheduledTask when(@NonNull BooleanSupplier condition) {
        return scheduler.when(condition);
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated continuously.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @NonNull
    public Scheduler.ScheduledTask on(@NonNull BooleanSupplier condition) {
        return scheduler.when(condition);
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a rising-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @NonNull
    public Scheduler.ScheduledTask whenRising(@NonNull BooleanSupplier condition) {
        return scheduler.whenRising(condition);
    }

    /**
     * Run a task when a condition is met.
     * This condition will be evaluated according to a falling-edge detection.
     *
     * @param condition Supplier to provide a boolean value of when the task should be run.
     * @return Task scheduling builder
     */
    @NonNull
    public Scheduler.ScheduledTask whenFalling(@NonNull BooleanSupplier condition) {
        return scheduler.whenFalling(condition);
    }

    /**
     * Run a task always. This is the same as calling {@code .when(() -> true)}.
     *
     * @return Task scheduling builder
     */
    @NonNull
    public Scheduler.ScheduledTask always() {
        return scheduler.always();
    }

    /**
     * Call to manually add the subsystems that should be managed by the scheduler. Using this method will override
     * the automatic collection of {@link BunyipsSubsystem}s, and allows you to determine which subsystems will be managed for
     * this OpMode.
     * <p>
     * For most cases, using this method is not required and all you need to do is construct your subsystems and they
     * will be managed automatically. This method is for advanced cases where you don't want this behaviour to happen.
     *
     * @param subsystems the restrictive list of subsystems to be managed and updated by the scheduler
     */
    public void use(@NonNull BunyipsSubsystem... subsystems) {
        if (!NullSafety.assertNotNull(Arrays.stream(subsystems).toArray())) {
            throw new RuntimeException("Null subsystems were added in the use() method!");
        }
        Collections.addAll(managedSubsystems, subsystems);
    }

    @Override
    protected final void onInit() {
        Exceptions.runUserMethod(this::onInitialise, this);

        if (managedSubsystems.isEmpty()) {
            managedSubsystems = BunyipsSubsystem.getInstances();
        }
        scheduler.addSubsystems(managedSubsystems.toArray(new BunyipsSubsystem[0]));

        Exceptions.runUserMethod(this::assignCommands, this);

        Scheduler.ScheduledTask[] tasks = scheduler.getAllocatedTasks();
        BunyipsSubsystem[] subsystems = scheduler.getManagedSubsystems();
        Text.Builder out = Text.builder();
        // Task count will account for tasks on subsystems that are not IdleTasks
        int taskCount = (int) (tasks.length + subsystems.length - Arrays.stream(subsystems).filter(BunyipsSubsystem::isIdle).count());
        out.append("[CommandBasedBunyipsOpMode] assignCommands() called | Managing % subsystem(s) | % task(s) scheduled (% subsystem, % command)\n",
                subsystems.length,
                taskCount,
                Arrays.stream(tasks).filter(task -> task.getTaskToRun().hasDependency()).count() + taskCount - tasks.length,
                Arrays.stream(tasks).filter(task -> !task.getTaskToRun().hasDependency()).count()
        );
        for (BunyipsSubsystem subsystem : subsystems) {
            out.append("  | %\n", subsystem.toVerboseString());
            for (Scheduler.ScheduledTask task : tasks) {
                Optional<BunyipsSubsystem> dep = task.getTaskToRun().getDependency();
                if (!dep.isPresent() || !dep.get().equals(subsystem)) continue;
                out.append("    -> %\n", task);
            }
        }
        for (Scheduler.ScheduledTask task : tasks) {
            if (task.getTaskToRun().getDependency().isPresent()) continue;
            out.append("  | %\n", task);
        }
        Dbg.logd(out.toString());
        // Ensure to always run assignCommands() even if no subsystems are made, since it may be used for other purposes
        if (managedSubsystems.isEmpty()) {
            throw new RuntimeException("No BunyipsSubsystems were constructed!");
        }
    }

    @Override
    protected final void activeLoop() {
        Exceptions.runUserMethod(this::periodic, this);
        scheduler.run();
    }

    // Access to the other BunyipsOpMode methods (onInitLoop() etc.) are handled by the user of this class, as
    // they may wish to do something else. This class does not limit the user from a standard BunyipsOpMode, but
    // removes the need to ensure the Scheduler is set up properly.

    /**
     * Runs upon the pressing of the INIT button on the Driver Station.
     * This is where you should initialise your hardware and other components.
     */
    protected abstract void onInitialise();

    /**
     * Assign your scheduler commands here by accessing the {@link #scheduler} and controllers {@link #driver()} and {@link #operator()}.
     */
    protected abstract void assignCommands();

    /**
     * Override this method to run any additional activeLoop() code before the Scheduler runs.
     */
    protected void periodic() {
    }
}
