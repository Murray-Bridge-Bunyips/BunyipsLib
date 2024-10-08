package org.murraybridgebunyips.bunyipslib.external;

import org.murraybridgebunyips.bunyipslib.external.pid.PIDFController;

/**
 * Proportional Integral Derivative Feedforward system controller.
 * Used for class casts that support PID algorithms, such as composite controllers that internally use PID.
 *
 * @since 4.0.0
 */
public interface PIDF extends SystemController {
    PIDFController getPIDFController();
}
