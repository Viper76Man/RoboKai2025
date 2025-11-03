package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadV1 {
    public Gamepad gamepad;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public double delay = 0.3;

    public double left_stick_x = 0;
    public double left_stick_y = 0;
    public double right_stick_x = 0;
    public double right_stick_y = 0;
    public double left_trigger = 0;
    public double right_trigger = 0;
    public boolean left_bumper = false;
    public boolean right_bumper = false;
    public boolean left_stick_button = false;
    public boolean right_stick_button = false;
    public boolean dpad_up = false;
    public boolean dpad_down = false;
    public boolean dpad_left = false;
    public boolean dpad_right = false;
    public boolean x = false;
    public boolean y = false;
    public boolean a = false;
    public boolean b = false;
    public boolean circle = false;
    public boolean square = false;
    public boolean triangle = false;
    public boolean cross = false;
    public boolean ps = false;

    public void init(Gamepad gamepad, double delaySeconds) {
        this.gamepad = gamepad;
        resetTimer();
        delay = delaySeconds;
    }

    public void resetTimer() {
        buttonTimer.reset();
    }

    public double getTimerSeconds() {
        return buttonTimer.seconds();
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    public boolean isGamepadReady() {
        return buttonTimer.seconds() > delay;
    }


    public void update() {
        left_stick_x = gamepad.left_stick_x;
        right_stick_x = gamepad.right_stick_x;
        left_stick_y = gamepad.left_stick_y;
        right_stick_y = gamepad.right_stick_y;
        left_trigger = gamepad.left_trigger;
        right_trigger = gamepad.right_trigger;
        circle = gamepad.circle;
        square = gamepad.square;
        triangle = gamepad.triangle;
        cross = gamepad.cross;
        left_bumper = gamepad.left_bumper;
        right_bumper = gamepad.right_bumper;
        left_stick_button = gamepad.left_stick_button;
        right_stick_button = gamepad.right_stick_button;
        dpad_down = gamepad.dpad_down;
        dpad_up = gamepad.dpad_up;
        dpad_left = gamepad.dpad_left;
        dpad_right = gamepad.dpad_right;
        ps = gamepad.ps;
        x = gamepad.x;
        y = gamepad.y;
        a = gamepad.a;
        b = gamepad.b;
    }
}
