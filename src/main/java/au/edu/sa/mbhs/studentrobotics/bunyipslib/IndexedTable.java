package au.edu.sa.mbhs.studentrobotics.bunyipslib;

import androidx.annotation.NonNull;

/**
 * Multipurpose index-based table with increment and decrement functionality.
 * Exposed as a runnable component for telemetry updates.
 *
 * @author Lucas Bubner, 2024
 * @since 1.0.0-pre
 */
public class IndexedTable implements Runnable {
    private final double[] tableValues;
    private String name = "IndexedTable";
    private int index = 0;

    /**
     * Create a new IndexedTable.
     *
     * @param tableValues the values to use, when the index is selected the corresponding value is used
     */
    public IndexedTable(@NonNull double... tableValues) {
        this.tableValues = tableValues;
    }

    /**
     * Set the default index of the indexed table. Implicit default of zero.
     *
     * @param defaultIndex the index to set
     * @return this
     */
    @NonNull
    public IndexedTable withDefaultIndex(int defaultIndex) {
        if (defaultIndex < 0 || defaultIndex >= tableValues.length)
            throw new EmergencyStop("Default index out of bounds");
        index = defaultIndex;
        return this;
    }

    /**
     * Set the name of the indexed table.
     *
     * @param name the name to set
     * @return this
     */
    @NonNull
    public IndexedTable withName(@NonNull String name) {
        this.name = name;
        return this;
    }

    /**
     * Increment the table index.
     */
    public void increment() {
        if (index >= tableValues.length - 1) return;
        index++;
    }

    /**
     * Decrement the table index.
     */
    public void decrement() {
        if (index <= 0) return;
        index--;
    }

    /**
     * Set the table index.
     *
     * @param index the index to set
     */
    public void set(int index) {
        if (index < 0) {
            this.index = 0;
            return;
        }
        if (index >= tableValues.length) {
            this.index = tableValues.length - 1;
            return;
        }
        this.index = index;
    }

    /**
     * Get the applied value from the index.
     *
     * @return the table value
     */
    public double get() {
        return tableValues[index];
    }

    /**
     * Optionally updates telemetry with the current value and index.
     */
    @Override
    public void run() {
        DualTelemetry.smartAdd(
                name,
                "% <font color='gray'>(%/%)</font>",
                tableValues[index],
                index + 1,
                tableValues.length
        );
    }
}
