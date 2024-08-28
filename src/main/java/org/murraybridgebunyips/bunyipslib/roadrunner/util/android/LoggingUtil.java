package org.murraybridgebunyips.bunyipslib.roadrunner.util.android;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Objects;

/**
 * Utility functions for log files.
 *
 * @since 1.0.0-pre
 */
public final class LoggingUtil {
    /**
     * The folder where RoadRunner logs are stored.
     */
    public static final File ROAD_RUNNER_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/");
    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB log quota for now

    private LoggingUtil() {
    }

    private static void buildLogList(List<? super File> logFiles, File dir) {
        for (File file : Objects.requireNonNull(dir.listFiles())) {
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                logFiles.add(file);
            }
        }
    }

    private static void pruneLogsIfNecessary() {
        List<File> logFiles = new ArrayList<>();
        buildLogList(logFiles, ROAD_RUNNER_FOLDER);
        logFiles.sort(Comparator.comparingLong(File::lastModified));

        long dirSize = 0;
        for (File file : logFiles) {
            dirSize += file.length();
        }

        while (dirSize > LOG_QUOTA) {
            if (logFiles.isEmpty()) break;
            File fileToRemove = logFiles.remove(0);
            dirSize -= fileToRemove.length();
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    /**
     * Obtain a log file with the provided name
     *
     * @param name the name of the log file
     * @return the log file
     */
    public static File getLogFile(String name) {
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        pruneLogsIfNecessary();

        return new File(ROAD_RUNNER_FOLDER, name);
    }
}
