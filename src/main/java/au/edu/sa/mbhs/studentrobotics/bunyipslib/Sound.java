package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import androidx.annotation.RawRes;

import com.qualcomm.ftccommon.SoundPlayer;

import org.apache.commons.math3.exception.NumberIsTooLargeException;
import org.apache.commons.math3.exception.NumberIsTooSmallException;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

/**
 * Class that interfaces with the {@link SoundPlayer} to create and broadcast sound files on the Driver Station.
 * <p>
 * All sounds played through this class are proxied via the {@link SoundPlayer} class, accessed on the {@link #play}
 * call. Sound files can be stored in the {@code res/raw} folder of the project, and can be retrieved
 * using your {@code R} class.
 *
 * @author Lucas Bubner, 2024
 * @see SoundPlayer
 * @since 6.1.0
 */
public class Sound {
    private final SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams(false);
    private final Object res;
    private Consumer<Integer> startAction = (i) -> {
    };
    private Runnable endAction = () -> {
    };

    /**
     * Create a new Sound file with the given file.
     *
     * @param file the file to play
     */
    public Sound(File file) {
        res = file;
        SoundPlayer.getInstance().preload(AppUtil.getDefContext(), file);
    }

    /**
     * Create a new Sound file with the given resource ID.
     *
     * @param resourceId the raw resource ID to play (e.g {@code R.raw.ss_r2d2_up})
     */
    public Sound(@RawRes int resourceId) {
        res = resourceId;
        SoundPlayer.getInstance().preload(AppUtil.getDefContext(), resourceId);
    }

    /**
     * Set the volume of the sound.
     *
     * @param volume the volume scaling to set the sound to, default 1.0
     * @return this Sound object
     */
    public Sound setVolume(double volume) {
        params.volume = (float) volume;
        return this;
    }

    /**
     * Set the playback rate of the sound.
     *
     * @param rate the rate to set the sound to, must be between 0.5 and 2, default 1.0
     * @return this Sound object
     */
    public Sound setRate(double rate) {
        if (rate < 0.5) {
            throw new NumberIsTooSmallException(rate, 0.5, true);
        }
        if (rate > 2) {
            throw new NumberIsTooLargeException(rate, 2, true);
        }
        params.rate = (float) rate;
        return this;
    }

    /**
     * Whether to wait for currently playing non-looping sounds to finish before playing this sound.
     *
     * @param wait whether to wait for non-looping sounds to finish, default is false, where sounds can overlap
     *             and play simultaneously
     * @return this Sound object
     */
    public Sound setYielding(boolean wait) {
        params.waitForNonLoopingSoundsToFinish = wait;
        return this;
    }

    /**
     * Set the action to run when the sound starts.
     *
     * @param action the action to run when the sound starts
     * @return this Sound object
     */
    public Sound onStart(java.util.function.Consumer<Integer> action) {
        startAction = action::accept;
        return this;
    }

    /**
     * Set the action to run when the sound ends.
     *
     * @param action the action to run when the sound ends
     * @return this Sound object
     */
    public Sound onEnd(Runnable action) {
        endAction = action;
        return this;
    }

    /**
     * Sends this sound to the Driver Station to be queued and played with the set parameters now.
     *
     * @param times the number of times to play the sound, <= 0 for infinite
     */
    public void play(int times) {
        params.loopControl = times <= 0 ? -1 : times - 1;
        if (res instanceof File) {
            SoundPlayer.getInstance().startPlaying(AppUtil.getDefContext(), (File) res, params, startAction, endAction);
        } else {
            SoundPlayer.getInstance().startPlaying(AppUtil.getDefContext(), (int) res, params, startAction, endAction);
        }
    }

    /**
     * Sends this sound to the Driver Station to be queued and played with the set parameters once.
     */
    public void play() {
        play(1);
    }

    /**
     * Sends this sound to the Driver Station to be queued and played with the set parameters infinitely.
     */
    public void playForever() {
        play(-1);
    }
}
