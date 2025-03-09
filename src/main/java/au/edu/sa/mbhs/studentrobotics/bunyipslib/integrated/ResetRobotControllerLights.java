package au.edu.sa.mbhs.studentrobotics.bunyipslib.integrated;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Dbg;

/**
 * Reset OpMode to clear any BunyipsLib-set robot controller lights.
 * <p>
 * Registered as a System OpMode and executed automagically on the stopping of any OpMode.
 *
 * @author Lucas Bubner, 2024
 * @since 3.4.0
 */
public final class ResetRobotControllerLights extends LinearOpMode {
    @Override
    public void runOpMode() {
        List<LynxModule> rcs = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < rcs.size(); i++) {
            LynxModule module = rcs.get(i);
            Dbg.logv(getClass(), "Resetting Robot Controller (#%) lights from % ...", i + 1, module.getPattern());
            module.setPattern(LynxModule.blinkerPolicy.getIdlePattern(module));
        }
        terminateOpModeNow();
    }
}
