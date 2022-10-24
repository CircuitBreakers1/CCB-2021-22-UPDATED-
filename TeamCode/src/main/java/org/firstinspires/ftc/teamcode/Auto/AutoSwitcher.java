package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Auto.switchType.*;

/**
 * This class provides the framework to allow opposite alliance autos to be created in one program
 * and simply switch certain values depending on if the opmode is run from blue or red side.
 */
public interface AutoSwitcher {

     static double switchable(double value, switchType SwitchMethod, boolean shouldSwitch) {
        if(SwitchMethod == INVERT_VALUE) {
            return -value;
        } else if (SwitchMethod == MIRROR_ANGLE) {
            return -value;
        }

        return 0;
    }
}

/**
 * Mirror angle inverts the angle across 0, Invert value negates the value
 */
enum switchType {
    MIRROR_ANGLE, INVERT_VALUE
}

