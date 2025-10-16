package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();


    public enum modeSelected {
        SELECTED,
        NOT_SELECTED
    }

    public enum modes {
        DRIVE,
        SHOOT,
        INTAKE,
        CAMERA
    }

    public modeSelected isModeSelected = modeSelected.NOT_SELECTED;
    public modes mode = modes.DRIVE;

    public int selectedMode = 0;

    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        mecDrive.init(hardwareMap, gamepad);

    }

    @Override
    public void loop() {
        if(isModeSelected == modeSelected.NOT_SELECTED) {
            menuSelectUpdate();
        }
        else {
            tuningUpdate();
        }
    }

    public void menuSelectUpdate(){
        telemetry.addData("Selected tuning mode: ", mode.name());
        if(gamepad.isGamepadReady()) {
            if (gamepad.gamepad.dpad_up) {
                mode = modes.values()[Math.max(0, (selectedMode - 1))];
            } else if (gamepad.gamepad.dpad_down) {
                mode = modes.values()[Math.min((selectedMode - 1), modes.values().length - 1)];
            }
            if (gamepad.gamepad.circle) {
                isModeSelected = modeSelected.SELECTED;
            }
        }
    }

    public void tuningUpdate(){
        switch (mode){
            case DRIVE:
                mecDrive.drive();
                mecDrive.log(telemetry);
                break;
            case SHOOT:
                break;
        }
    }
}
