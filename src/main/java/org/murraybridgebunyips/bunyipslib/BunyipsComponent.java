package org.murraybridgebunyips.bunyipslib;

/**
 * Base class for components used with a BunyipsOpMode.
 * This allows injection of the OpMode into the component, and provides a common base for all components.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public abstract class BunyipsComponent {
    // Pre-4.1.0, this property would cause a critical crash if a BunyipsComponent was instantiated as a member
    // field of the derived BOM class, however, a constructor hook is now called to supply the instance field, albeit
    // partially constructed. This partial construction won't particularly matter to the user unless they decide
    // to class-cast for some reason, in which case casting will be available on the instance during the re-assignment on runtime.
    // This makes it so a BunyipsOpMode has a brief 'partially constructed' phase that can allow components that extend
    // from this class to work properly, reducing the number of gotchas when writing OpModes.
    // This partial construction still doesn't allow conventional subsystems to be instantiated early (as hardwareMap is
    // not available), however, simple tasks that operate on functional interfaces benefit from this behaviour.
    // TODO: consider nature of BunyipsComponent in all contexts - should Task really crash if its not in a BOM? (should be null instead?)
    protected final BunyipsOpMode opMode = BunyipsOpMode.getInstance();
}
