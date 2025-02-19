/*
 * Copyright (c) 2023 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
package au.edu.sa.mbhs.studentrobotics.bunyipslib.external;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Locale;
import java.util.Stack;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import au.edu.sa.mbhs.studentrobotics.bunyipslib.transforms.Controls;
import au.edu.sa.mbhs.studentrobotics.bunyipslib.util.Text;

/**
 * Utility for creating interactive menus through {@link Telemetry}.
 * <a href="https://github.com/OpenFTC/Extracted-RC/blob/b585e5988f9e32a7a80df65f510f5e354903bb94/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/UtilityOctoQuadConfigMenu.java#L296">Source (extracted from FtcRobotController v9.2)</a>
 *
 * @author OpenFTC, 2023
 * @since 4.0.0
 */
public class TelemetryMenu {
    private final MenuElement root;
    private final Telemetry telemetry;
    private final Stack<Integer> selectedIdxStack = new Stack<>();
    private MenuElement currentLevel;
    private boolean dpadUpPrev;
    private boolean dpadDnPrev;
    private boolean dpadRightPrev;
    private boolean dpadLeftPrev;
    private boolean aPrev;
    private boolean bPrev;
    private int selectedIdx = 0;

    /**
     * TelemetryMenu constructor.
     *
     * @param telemetry pass in 'telemetry' from your OpMode
     * @param root      the root menu element
     */
    public TelemetryMenu(@NonNull Telemetry telemetry, @NonNull MenuElement root) {
        this.root = root;
        currentLevel = root;
        this.telemetry = telemetry;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(50);
    }

    /**
     * Call this from inside your loop to put the current menu state into
     * telemetry, and process gamepad inputs for navigating the menu
     *
     * @param gamepad the gamepad you want to use to navigate the menu
     */
    public void loop(@NonNull Gamepad gamepad) {
        // Capture current state of the gamepad buttons we care about;
        // We can only look once, or we risk a race condition
        boolean dpadUp = gamepad.dpad_up;
        boolean dpadDn = gamepad.dpad_down;
        boolean dpadRight = gamepad.dpad_right;
        boolean dpadLeft = gamepad.dpad_left;
        // Use Controls util to prevent Start+A/B presses
        boolean a = Controls.isSelected(gamepad, Controls.A);
        boolean b = Controls.isSelected(gamepad, Controls.B);

        // Figure out who our children our at this level
        // and figure out which item is currently highlighted
        // with the selection pointer
        ArrayList<Element> children = currentLevel.children();
        Element currentSelection = children.get(selectedIdx);

        // Left and right are inputs to the selected item (if it's an Option)
        if (currentSelection instanceof OptionElement) {
            if (dpadRight && !dpadRightPrev) // rising edge
            {
                ((OptionElement) currentSelection).onRightInput();
            } else if (dpadLeft && !dpadLeftPrev) // rising edge
            {
                ((OptionElement) currentSelection).onLeftInput();
            }
        }

        // Up and down navigate the current selection pointer
        if (dpadUp && !dpadUpPrev) // rising edge
        {
            selectedIdx--; // Move selection pointer up
        } else if (dpadDn && !dpadDnPrev) // rising edge
        {
            selectedIdx++; // Move selection pointer down
        }

        // Make selected index sane (don't let it go out of bounds) :eyes:
        if (selectedIdx >= children.size()) {
            selectedIdx = children.size() - 1;
        } else if (selectedIdx < 0) {
            selectedIdx = 0;
        }

        // Select: either enter submenu or input to option
        else if (a && !aPrev) // rising edge
        {
            // Select up element
            if (currentSelection instanceof SpecialUpElement) {
                // We can only go up if we're not at the root level
                if (currentLevel != root) {
                    // Restore selection pointer to where it was before
                    selectedIdx = selectedIdxStack.pop();

                    // Change to the parent level
                    currentLevel = currentLevel.parent();
                }
            }
            // Input to option
            else if (currentSelection instanceof OptionElement) {
                ((OptionElement) currentSelection).onClick();
            }
            // Enter submenu
            else if (currentSelection instanceof MenuElement) {
                // Save our current selection pointer so we can restore it
                // later if the user navigates back up a level
                selectedIdxStack.push(selectedIdx);

                // We have no idea what's in the submenu :monkey: so best to
                // just set the selection pointer to the first element
                selectedIdx = 0;

                // Now the current level becomes the submenu that the selection
                // pointer was on
                currentLevel = (MenuElement) currentSelection;
            }
        }

        // Go up a level
        else if (b && !bPrev) {
            // We can only go up if we're not at the root level
            if (currentLevel != root) {
                // Restore selection pointer to where it was before
                selectedIdx = selectedIdxStack.pop();

                // Change to the parent level
                currentLevel = currentLevel.parent();
            }
        }

        // Save the current button states so that we can look for
        // the rising edge the next time around the loop :)
        dpadUpPrev = dpadUp;
        dpadDnPrev = dpadDn;
        dpadRightPrev = dpadRight;
        dpadLeftPrev = dpadLeft;
        aPrev = a;
        bPrev = b;

        // Start building the text display.
        //noinspection ExtractMethodRecommender
        StringBuilder builder = new StringBuilder();
        // First, we add the static directions for gamepad operation
        builder.append("<font color='#119af5' face=monospace>");
        builder.append("Navigate items.....dpad up/down\n")
                .append("Up one level.......B\n")
                .append("Edit option........dpad left/right\n")
                .append("Select.............A\n");
        builder.append("</font>");
        builder.append("\n");

        // Now actually add the menu options. We start by adding the name of the current menu level.
        builder.append("<font face=monospace>");
        builder.append("Menu: ").append(currentLevel.name).append("\n");

        // Now we loop through all the child elements of this level and add them
        for (int i = 0; i < children.size(); i++) {
            // If the selection pointer is at this index, put a green dot in the box :)
            if (selectedIdx == i) {
                builder.append("[<font color=green face=monospace>•</font>]: ");
            }
            // Otherwise, just put an empty box
            else {
                builder.append("[ ]: ");
            }

            // Figure out who the selection pointer is pointing at :eyes:
            Element e = children.get(i);

            // If it's pointing at a submenu, indicate that it's a submenu to the user
            // by prefixing "> " to the name.
            if (e instanceof MenuElement) {
                builder.append("> ");
            }

            // Finally, add the element's name
            builder.append(e.getDisplayText());

            // We musn't forget the newline
            builder.append("\n");
        }

        // Don't forget to close the font tag either
        builder.append("</font>");

        // Build the string!!!! :nerd:
        // and add it to telemetry
        telemetry.addLine(builder.toString());
    }

    /**
     * An element of {@link TelemetryMenu}, representing a "page" in the overall menu display.
     */
    public static class MenuElement extends Element {
        private final String name;
        private final ArrayList<Element> children = new ArrayList<>();

        /**
         * Create a new MenuElement; may either be the root menu, or a submenu (set isRoot accordingly)
         *
         * @param name   the name for this menu
         * @param isRoot whether this is a root menu, or a submenu
         */
        public MenuElement(@NonNull String name, boolean isRoot) {
            this.name = name;

            // If it's not the root menu, we add the up one level option as the first element
            if (!isRoot) {
                children.add(new SpecialUpElement());
            }
        }

        /**
         * Add a child element to this menu (may either be an Option or another menu)
         *
         * @param child the child element to add
         */
        public void addChild(@NonNull Element child) {
            child.setParent(this);
            children.add(child);
        }

        /**
         * Add multiple child elements to this menu (may either be option, or another menu)
         *
         * @param childrenElems the children to add
         */
        public void addChildren(@NonNull Element... childrenElems) {
            for (Element e : childrenElems) {
                e.setParent(this);
                children.add(e);
            }
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return name;
        }

        private ArrayList<Element> children() {
            return children;
        }
    }

    /**
     * An option that has interactivity with the user.
     */
    public abstract static class OptionElement extends Element {
        /**
         * Override this to get notified when the element is clicked
         */
        protected void onClick() {
        }

        /**
         * Override this to get notified when the element gets a "left edit" input
         */
        protected void onLeftInput() {
        }

        /**
         * Override this to get notified when the element gets a "right edit" input
         */
        protected void onRightInput() {
        }
    }

    /**
     * A set of menu items that is backed by an Enum.
     */
    @SuppressWarnings("rawtypes")
    public static class EnumOption extends OptionElement {
        protected int idx = 0;
        @NonNull
        protected Enum[] e;
        @NonNull
        protected String name;

        /**
         * Create a new EnumOption.
         *
         * @param name The name of this option
         * @param e    The enum to back these options
         */
        public EnumOption(@NonNull String name, @NonNull Enum[] e) {
            this.e = e;
            this.name = name;
        }

        /**
         * Create a new EnumOption.
         *
         * @param name The name of this option
         * @param e    The enum to back these options
         * @param def  Default value
         */
        public EnumOption(@NonNull String name, @NonNull Enum[] e, @NonNull Enum def) {
            this(name, e);
            idx = def.ordinal();
        }

        @Override
        public void onLeftInput() {
            idx++;

            if (idx > e.length - 1) {
                idx = 0;
            }
        }

        @Override
        public void onRightInput() {
            idx--;

            if (idx < 0) {
                idx = e.length - 1;
            }
        }

        @Override
        public void onClick() {
            onRightInput();
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, e[idx].name());
        }

        @NonNull
        public Enum getValue() {
            return e[idx];
        }
    }

    /**
     * A menu item backed by a range of ints.
     */
    public static class IntegerOption extends OptionElement {
        protected int i;
        protected int min;
        protected int max;
        @NonNull
        protected String name;

        /**
         * Create a new IntegerOption.
         *
         * @param name the name of this option
         * @param min  minimum integer range
         * @param max  maximum integer range
         * @param def  default option
         */
        public IntegerOption(@NonNull String name, int min, int max, int def) {
            this.name = name;
            this.min = min;
            this.max = max;
            i = def;
        }

        @Override
        public void onLeftInput() {
            i--;

            if (i < min) {
                i = max;
            }
        }

        @Override
        public void onRightInput() {
            i++;

            if (i > max) {
                i = min;
            }
        }

        @Override
        public void onClick() {
            onRightInput();
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return String.format(Locale.getDefault(), "%s: <font color='#e37c07' face=monospace>%d</font>", name, i);
        }

        public int getValue() {
            return i;
        }
    }

    /**
     * A menu item backed by a boolean that can be adjusted by the user.
     */
    public static class BooleanOption extends OptionElement implements BooleanSupplier {
        private final String name;
        private boolean val;

        private String customTrue;
        private String customFalse;

        /**
         * Create a new BooleanOption.
         *
         * @param name the name of this option
         * @param def  default value
         */
        public BooleanOption(@NonNull String name, boolean def) {
            this.name = name;
            val = def;
        }

        /**
         * Create a new BooleanOption.
         *
         * @param name        the name of this option
         * @param def         default value
         * @param customTrue  the value instead of "true" to display when this option is selected
         * @param customFalse the value instead of "false" to display when this option is selected
         */
        public BooleanOption(@NonNull String name, boolean def, @NonNull String customTrue, @NonNull String customFalse) {
            this(name, def);
            this.customTrue = customTrue;
            this.customFalse = customFalse;
        }

        @Override
        public void onLeftInput() {
            val = !val;
        }

        @Override
        public void onRightInput() {
            val = !val;
        }

        @Override
        public void onClick() {
            val = !val;
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            String valStr;

            if (customTrue != null && customFalse != null) {
                valStr = val ? customTrue : customFalse;
            } else {
                valStr = val ? "true" : "false";
            }

            return String.format("%s: <font color='#e37c07' face=monospace>%s</font>", name, valStr);
        }

        /**
         * Gets the state of this boolean menu item.
         *
         * @return a result whether this item is selected
         */
        @Override
        public boolean getAsBoolean() {
            return val;
        }
    }

    /**
     * A menu item backed by a simple string.
     */
    public static class StaticItem extends OptionElement {
        private final Object name;

        /**
         * Create a new StaticItem.
         *
         * @param name the name of this option
         */
        public StaticItem(@NonNull Object name) {
            this.name = name;
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return name.toString();
        }
    }

    /**
     * A menu item that may be clicked by the user.
     */
    public static class StaticClickableOption extends OptionElement {
        private final String name;
        private Runnable onClick;

        protected StaticClickableOption(@NonNull String name) {
            this.name = name;
        }

        /**
         * Create a new StaticClickableOption.
         *
         * @param name    the name of this option
         * @param onClick the action to perform when this option is clicked
         */
        public StaticClickableOption(@NonNull String name, @NonNull Runnable onClick) {
            this.name = name;
            this.onClick = onClick;
        }

        protected void onClick() {
            if (onClick != null)
                onClick.run();
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return name;
        }
    }

    /**
     * A menu item backed by a string that can be adjusted by the user.
     *
     * @author Lucas Bubner, 2024
     * @since 6.0.0
     */
    public static class DynamicItem extends Element {
        private final String title;
        private final Supplier<?> displayText;

        /**
         * Create a new DynamicItem.
         *
         * @param title       the static name of this option
         * @param displayText a function that returns the display value for this option
         */
        public DynamicItem(@NonNull String title, @NonNull Supplier<?> displayText) {
            this.title = title;
            this.displayText = displayText;
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            return Text.format("%: %", title, displayText.get());
        }
    }

    /**
     * A menu element that has an attached functional interface hook for the current selected state.
     *
     * @author Lucas Bubner, 2024
     * @since 6.0.0
     */
    public static class InteractiveToggle extends OptionElement {
        private final String title;
        private final Function<Boolean, ?> executor;
        private final boolean def;
        private BooleanSupplier reset;
        private BooleanSupplier latch;
        private String activeColour = "green";
        private String inactiveColour = "white";
        private boolean active;

        /**
         * Create a new InteractiveOption.
         *
         * @param title    the static name of this option
         * @param executor a function that returns the display text for this option, with the supplied parameter being the current state;
         *                 this parameter allows you to perform operations continuously while the option is active/inactive.
         * @param def      the default state of this option
         */
        public InteractiveToggle(@NonNull String title, boolean def, @NonNull Function<Boolean, ?> executor) {
            this.title = title;
            this.executor = executor;
            this.def = def;
            active = def;
        }

        /**
         * Sets a supplier that determines whether this option should be returned to default state.
         * Useful for options that should not be interacted with under certain conditions.
         *
         * @param defState the supplier that determines whether this option should be reset to default state
         * @return this
         */
        @NonNull
        public InteractiveToggle resetIf(@NonNull BooleanSupplier defState) {
            reset = defState;
            return this;
        }

        /**
         * The opposite of {@link #resetIf(BooleanSupplier)}, which will instead set the state to the opposite
         * on this boolean supplier activating.
         *
         * @param notDefState the supplier that determines whether this option should be set to the opposite of the def. state
         * @return this
         */
        @NonNull
        public InteractiveToggle latchIf(@NonNull BooleanSupplier notDefState) {
            latch = notDefState;
            return this;
        }

        /**
         * Set the colours for the active and inactive states of this option.
         * Defaults to white and green for inactive and active states, respectively.
         *
         * @param activeColour   the colour to display when this option is active
         * @param inactiveColour the colour to display when this option is inactive
         * @return this
         */
        @NonNull
        public InteractiveToggle withColours(@NonNull String activeColour, @NonNull String inactiveColour) {
            this.activeColour = activeColour;
            this.inactiveColour = inactiveColour;
            return this;
        }

        @Override
        protected void onClick() {
            active = !active;
        }

        @Override
        protected void onLeftInput() {
            active = false;
        }

        @Override
        protected void onRightInput() {
            active = true;
        }

        @Override
        @NonNull
        protected String getDisplayText() {
            if (reset != null && reset.getAsBoolean())
                active = def;
            if (latch != null && latch.getAsBoolean())
                active = !def;
            return Text.format("<font color='%' face='monospace'><b>%</b>: %</font>", active ? activeColour : inactiveColour, title, executor.apply(active));
        }
    }

    /**
     * A generic child component that has a parent and a backing string.
     */
    public abstract static class Element {
        private MenuElement parent;

        protected void setParent(@NonNull MenuElement parent) {
            this.parent = parent;
        }

        @NonNull
        protected MenuElement parent() {
            return parent;
        }

        @NonNull
        protected abstract String getDisplayText();
    }

    private static class SpecialUpElement extends Element {
        @Override
        @NonNull
        protected String getDisplayText() {
            return "<font color='#119af5' face=monospace>.. ↰ Up One Level</font>";
        }
    }
}