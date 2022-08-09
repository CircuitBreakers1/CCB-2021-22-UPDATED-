package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.switchMethod.*;

/**
 * This class provides the framework to allow opposite alliance autos to be created in one program
 * and simply switch certain values depending on if the opmode is run from blue or red side.
 */
public class autoSwitcher {
    public autoSwitcher() {}

    public static float switchable(float value, switchMethod SwitchMethod) {
        if(SwitchMethod == INVERT_VALUE) {
            return -value;
        } else if (SwitchMethod == MIRROR_ANGLE) {

        }

        return 0f;
    }
}

enum switchMethod {
    MIRROR_ANGLE, INVERT_VALUE
}

