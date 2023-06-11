package org.firstinspires.ftc.teamcode.archive.PowerPlay.Subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

public class ButtonToggleSubsystem {
    /*
    private BooleanSupplier buttonState;
    private boolean currState;
    //Shows if the toggle has been checked or not
    private boolean isUpdated = true;
    private static List<ButtonToggleSubsystem> listOfButtons = null;

    public ButtonToggleSubsystem(BooleanSupplier tempButton) {
        this.buttonState = tempButton;
        listOfButtons.add(this);
    }

    **
     * Clears the list of buttons to check. Run during init
     *
    public static void clearList() {
        listOfButtons.clear();
    }

    **
     * Updates button values. Run at start of loop
     *
     /*
    public static void updateButtons() {
        for (ButtonToggleSubsystem button : listOfButtons) {
            if(button.buttonState.getAsBoolean() != button.currState) {
                button.currState = button.buttonState.getAsBoolean();
                button.isUpdated = false;
            }
        }
    }

    public boolean isOn() {
        return this.currState;
    }

    public boolean isOff() {
        return !this.currState;
    }

    public boolean isToggledOn() {
        if(!this.isUpdated && this.currState) {
            return true;
        }

        return false;
    }

    public boolean isToggledOff() {
        if(!this.isUpdated && !this.currState) {
            return true;
        }

        return false;
    }


    */
}
