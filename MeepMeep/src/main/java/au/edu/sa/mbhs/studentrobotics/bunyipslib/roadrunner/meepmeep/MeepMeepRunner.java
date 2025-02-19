package au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep;

import static au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.internal.units.Units.*;

import com.acmerobotics.roadrunner.*;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.BunyipsLibBotBuilder;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.roadrunner.meepmeep.shims.MeepMeepInternal;

/**
 * Runner file for MeepMeep. This is where you will create your MeepMeep path and run it.
 * <p>
 * You may wish to ignore this file from Git using `git update-index --skip-worktree MeepMeepRunner.java`
 * in the current directory from the command line.
 *
 * @since 6.0.0
 */
@SuppressWarnings("UnknownNullness")
public final class MeepMeepRunner extends MeepMeepInternal {
    /**
     * Entry point for MeepMeep.
     * Configure and run all your MeepMeep paths here. You can run MeepMeep by running this file in Android Studio
     * (Right-click file > Run MeepMeepRunner.main()).
     *
     * @param args cli args
     */
    public static void main(String[] args) {
        final boolean HARDWARE_ACCELERATION = false;
        final int WINDOW_SIZE_PX = 800;
        final int MAX_FPS = 60;

        System.setProperty("sun.java2d.opengl", String.valueOf(HARDWARE_ACCELERATION));
        MeepMeep meepMeep = new MeepMeep(WINDOW_SIZE_PX, MAX_FPS);

        RoadRunnerBotEntity bot = new BunyipsLibBotBuilder(meepMeep)
                .setMaxVel(60, InchesPerSecond)
                .setMaxAccel(60, InchesPerSecondPerSecond)
                .setMaxAngVel(180, DegreesPerSecond)
                .setMaxAngAccel(180, DegreesPerSecondPerSecond)
                .setTrackWidth(15, Inches)
                .build();

        drive.makeTrajectory(new Pose2d(0, 0, 0))
                .addTask();

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.8f)
                .addEntity(bot)
                .start();
    }
}