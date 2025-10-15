package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public enum modeSelected {
        SELECTED,
        NOT_SELECTED
    }
    public modeSelected isModeSelected = modeSelected.NOT_SELECTED;

    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        if(isModeSelected == modeSelected.NOT_SELECTED){
            if(gamepad.isGamepadReady()){

            }
        }
    }

    @Override
    public void loop() {

    }
}
