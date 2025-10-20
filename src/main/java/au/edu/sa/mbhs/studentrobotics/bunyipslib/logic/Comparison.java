package au.edu.sa.mbhs.studentrobotics.bunyipslib.logic;

/**
 * Represents a generic boolean comparison between two numeric values.
 *
 * @author Lucas Bubner, 2025
 * @since 8.0.0
 */
public enum Comparison {
    /**
     * a < b
     */
    LESS_THAN("<"),
    /**
     * a > b
     */
    GREATER_THAN(">"),
    /**
     * a <= b
     */
    LESS_THAN_OR_EQUAL("<="),
    /**
     * a >= b
     */
    GREATER_THAN_OR_EQUAL(">="),
    /**
     * a == b
     */
    EQUAL("=="),
    /**
     * a != b
     */
    NOT_EQUAL("!="),
    /**
     * Math.abs(a) > b
     */
    MAGNITUDE_GREATER_THAN("abs(>)");

    /**
     * Standard symbol representation for this comparison.
     */
    public final String symbol;

    Comparison(String symbol) {
        this.symbol = symbol;
    }
}
