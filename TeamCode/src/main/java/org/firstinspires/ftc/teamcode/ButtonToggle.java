package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.List;


/**
 * Yes, this class is more complex than it needs to be, but java no like pass by reference so here
 * we are
 */
public class ButtonToggle implements Runnable {

    private boolean isOn = false;
    private boolean isDown = false;
    private boolean toggleProcessed = true;
    private boolean thisButtonState = false;
    private int gamepadRef;

    private Button button;

    public static OpMode opMode;

    //Create a list to show the buttons that need to be updated
    private static List<ButtonToggle> buttons = new ArrayList<>();

    public ButtonToggle(int gamepadNumber, Button buttonName) {
        this.button = buttonName;
        this.gamepadRef = gamepadNumber;
        buttons.add(this);
    }

    public void onToggleOn(Runnable action) {
        if(this.toggleProcessed || !this.isOn) return;
        run();
        this.toggleProcessed = true;
    }

    public void onToggleOff(Runnable action) {
        if(this.toggleProcessed || this.isOn) return;
        run();
        this.toggleProcessed = true;
    }

    public void whileOn(Runnable action) {
        if(!isOn) return;
        run();
    }

    @Override
    public void run() {

    }

    /**
     * Updates the states of the buttons. Run at the start of the loop.
     */
    public static void updateButtons() {
        for(ButtonToggle btn: buttons) {
            //Updates the button state that matches each object
            switch (btn.button) {
                case a:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.a;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.a;
                    }
                    break;
                case b:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.b;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.b;
                    }
                    break;
                case back:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.back;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.back;
                    }
                    break;
                case dpad_down:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.dpad_down;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.dpad_down;
                    }
                    break;
                case dpad_left:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.dpad_left;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.dpad_left;
                    }
                    break;
                case dpad_right:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.dpad_right;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.dpad_right;
                    }
                    break;
                case dpad_up:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.dpad_up;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.dpad_up;
                    }
                    break;
                case guide:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.guide;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.guide;
                    }
                    break;
                case left_bumper:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.left_bumper;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.left_bumper;
                    }
                    break;
                case left_stick_button:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.left_stick_button;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.left_stick_button;
                    }
                    break;
                case right_bumper:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.right_bumper;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.right_bumper;
                    }
                    break;
                case right_stick_button:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.right_stick_button;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.right_stick_button;
                    }
                    break;
                case start:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.start;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.start;
                    }
                    break;
                case touchpad:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.touchpad;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.touchpad;
                    }
                    break;
                case touchpad_finger_1:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.touchpad_finger_1;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.touchpad_finger_1;
                    }
                    break;
                case touchpad_finger_2:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.touchpad_finger_2;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.touchpad_finger_2;
                    }
                    break;
                case x:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.x;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.x;
                    }
                    break;
                case y:
                    if (btn.gamepadRef == 1) {
                        btn.thisButtonState = opMode.gamepad1.y;
                    } else {
                        btn.thisButtonState = opMode.gamepad2.y;
                    }
                    break;
            }

            if(btn.thisButtonState && !btn.isDown) {
                btn.isDown = true;
                btn.isOn = !btn.isOn;
                btn.toggleProcessed = false;
            } else if(!btn.thisButtonState) btn.isDown = false;
        }
    }

}

enum Button {
    a,
    b,
    back,
    dpad_down,
    dpad_left,
    dpad_right,
    dpad_up,
    guide,
    left_bumper,
    left_stick_button,
    right_bumper,
    right_stick_button,
    start,
    touchpad,
    touchpad_finger_1,
    touchpad_finger_2,
    x,
    y,
}
