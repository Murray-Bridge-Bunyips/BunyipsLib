package au.edu.sa.mbhs.studentrobotics.bunyipslib.util;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.Pose2d;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonDeserializationContext;
import com.google.gson.JsonDeserializer;
import com.google.gson.JsonElement;
import com.google.gson.JsonParseException;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSerializationContext;
import com.google.gson.JsonSerializer;
import com.google.gson.JsonSyntaxException;
import com.google.gson.reflect.TypeToken;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.Closeable;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.HashMap;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.RobotConfig;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.executables.UserSelection;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.localization.Localizer;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.StartingConfiguration;

/**
 * Global filesystem and volatile storage utilities for robot operation.
 *
 * @author Lucas Bubner, 2024
 * @since 3.2.0
 */
public final class Storage {
    private static Memory memory = null;
    private static Filesystem filesystem = null;

    private Storage() {
        throw new AssertionError("This is a utility class");
    }

    /**
     * Get the global volatile (cleared after restart) memory storage for the robot.
     *
     * @return Instance for volatile memory storage
     */
    @NonNull
    public static Memory memory() {
        if (memory == null)
            memory = new Memory();
        return memory;
    }

    /**
     * Get the global persistent (saved after restart) filesystem storage for the robot.
     * Storage information is saved in the robot controller internal storage.
     *
     * @return Instance for persistent storage
     */
    @NonNull
    public static Filesystem filesystem() {
        if (filesystem == null)
            filesystem = new Filesystem();
        return filesystem;
    }

    /**
     * Represents in-memory storage for the robot.
     * <p>
     * The volatile memory HashMap, previously in this class, is now stored in the OpMode {@code blackboard},
     * introduced in SDK v10.3.
     */
    public static class Memory {
        /**
         * Static array of hardware errors stored via hardware name.
         *
         * @see RobotConfig
         */
        public final ArrayList<String> hardwareErrors = new ArrayList<>();
        /**
         * The last known/selected {@link StartingConfiguration.Position} object, which can be used to access
         * the last selected alliance, starting position, or other flags as defined by this object selected.
         * <p>
         * This object is automatically filled by {@link UserSelection} immediately when a selection is made, which
         * can fire a maximum of once per selection.
         *
         * @see StartingConfiguration
         * @since 7.1.0
         */
        @Nullable
        public StartingConfiguration.Position lastKnownStartingConfiguration = null;
        /**
         * The last known position of the robot from odometry localization.
         * Defaults to the origin.
         *
         * @see Localizer
         */
        @NonNull
        public Pose2d lastKnownPosition = Geometry.zeroPose();

        private Memory() {
        }

        /**
         * Clear all volatile memory related to the robot, including the HashMap.
         */
        public void clear() {
            OpMode.blackboard.clear();
            hardwareErrors.clear();
            lastKnownStartingConfiguration = null;
            lastKnownPosition = Geometry.zeroPose();
        }

        /**
         * Clear the volatile HashMap. This is linked to the OpMode {@code blackboard} member introduced in SDK v10.3.
         */
        public void clearVolatile() {
            OpMode.blackboard.clear();
        }

        /**
         * Get a volatile value from memory stored by key in {@link #setVolatile(String, Object)}.
         * <p>
         * This is linked to the OpMode {@code blackboard} member introduced in SDK v10.3.
         *
         * @param key the key to search for
         * @return the value associated with the key
         * @throws IllegalArgumentException if key not found
         */
        @Nullable
        public Object getVolatile(@NonNull String key) throws IllegalArgumentException {
            if (!OpMode.blackboard.containsKey(key))
                throw new IllegalArgumentException("Key not found in memory: " + key);
            return OpMode.blackboard.get(key);
        }

        /**
         * Set a volatile value in memory stored by key.
         * <p>
         * This is linked to the OpMode {@code blackboard} member introduced in SDK v10.3.
         *
         * @param key   the key to store the value under
         * @param value the value to store
         */
        public void setVolatile(@NonNull String key, @NonNull Object value) {
            OpMode.blackboard.put(key, value);
        }
    }

    /**
     * Represents persistent, file-saved storage for the robot.
     */
    public static class Filesystem implements Closeable {
        private final HashMap<String, Object> store = new HashMap<>();
        private final File file = new File(AppUtil.ROBOT_DATA_DIR, "bunyipslib_storage.json");
        private final Gson gson = new GsonBuilder()
                .registerTypeAdapter(Class.class, new ClassTypeAdapter())
                .setPrettyPrinting()
                .create();
        private final Type storeType = new TypeToken<HashMap<String, Object>>() {
        }.getType();

        private Filesystem() {
            File dir = file.getParentFile();
            AppUtil.getInstance().ensureDirectoryExists(dir);

            if (!file.exists())
                return;

            try (FileReader reader = new FileReader(file)) {
                store.putAll(gson.fromJson(reader, storeType));
            } catch (IOException | JsonSyntaxException e) {
                Dbg.error("Failed to load storage file: " + e.getMessage());
            }
        }

        /**
         * Access the HashMap of all stored values in the filesystem.
         * <p>
         * When this resource is closed, the values are saved to the file, so ensure to use
         * a try-with-resources block or call {@link #close()} when done.
         * <p>
         * <b>Warning</b>: Trying to add objects to the HashMap that are not serializable by Gson will throw an exception on write,
         * and may cause in the corruption of the storage file (other valid objects may be lost)
         *
         * @return the stored values
         */
        @NonNull
        public HashMap<String, Object> access() {
            return store;
        }

        /**
         * Delete all persistent storage related to the robot.
         * This will close the stream and save the empty store to the file.
         *
         * @throws RuntimeException if the storage file cannot be deleted
         */
        public void delete() {
            store.clear();
            close();
            filesystem = null;
        }

        @Override
        public void close() {
            try (FileWriter writer = new FileWriter(file)) {
                gson.toJson(store, writer);
            } catch (IOException e) {
                Dbg.error("Failed to save storage file: " + e.getMessage());
            }
        }

        private static class ClassTypeAdapter implements JsonSerializer<Class<?>>, JsonDeserializer<Class<?>> {
            @Override
            public JsonElement serialize(Class<?> src, Type typeOfSrc, JsonSerializationContext context) {
                return new JsonPrimitive(src.getName());
            }

            @Override
            public Class<?> deserialize(JsonElement json, Type typeOfT, JsonDeserializationContext context)
                    throws JsonParseException {
                try {
                    return Class.forName(json.getAsString());
                } catch (ClassNotFoundException e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
