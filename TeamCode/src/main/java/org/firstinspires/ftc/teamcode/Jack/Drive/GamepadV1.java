package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class GamepadV1 {
    public Gamepad gamepad;
    public ElapsedTime buttonTimer = new ElapsedTime();
    public double delay = 0.3;
    public void init(Gamepad gamepad, double delaySeconds){
        this.gamepad = gamepad;
        resetTimer();
        delay = delaySeconds;
    }

    public void resetTimer(){
        buttonTimer.reset();
    }

    public boolean isGamepadReady(){
        return buttonTimer.seconds() > delay;
    }
}
