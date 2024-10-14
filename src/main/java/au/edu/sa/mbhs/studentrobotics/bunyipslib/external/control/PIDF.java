package au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.external.control.pid.PIDFController;

/**
 * Proportional Integral Derivative Feedforward system controller.
 * Used for class casts that support PID algorithms, such as composite controllers that internally use PID.
 *
 * @since 4.0.0
 */
public interface PIDF extends SystemController {
    PIDFController getPIDFController();
}
