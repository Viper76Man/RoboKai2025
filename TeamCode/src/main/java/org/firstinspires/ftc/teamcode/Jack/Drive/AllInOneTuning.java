package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;

@TeleOp
public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();

    public boolean firstIteration = true;

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
        arcShooter.init(hardwareMap);
        intake.init(hardwareMap);

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
        gamepad.update();
        telemetry.addData("Selected tuning mode: ", mode.name());
        if(gamepad.isGamepadReady()) {
            if (gamepad.dpad_up) {
                selectedMode -= 1;
                mode = modes.values()[Math.max(0, (selectedMode))];
                gamepad.resetTimer();
            } else if (gamepad.dpad_down) {
                selectedMode += 1;
                mode = modes.values()[Math.min((selectedMode), modes.values().length - 1)];
                gamepad.resetTimer();
            }
            if (gamepad.circle) {
                isModeSelected = modeSelected.SELECTED;
                gamepad.resetTimer();
            }
        }
    }

    public void tuningUpdate(){
        gamepad.update();
        switch (mode){
            case DRIVE:
                mecDrive.drive();
                mecDrive.log(telemetry);
                break;
            case SHOOT:
                //Set initial velocity
                if(firstIteration){
                    arcShooter.setTargetVelocity(RobotConstantsV1.defaultShooterVelocity);
                    firstIteration = false;
                }

                if(gamepad.dpad_up && gamepad.isGamepadReady()){
                    arcShooter.setTargetVelocity(arcShooter.getTargetVelocity() + RobotConstantsV1.velocityUpStep);
                    gamepad.resetTimer();
                }
                else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
                    arcShooter.setTargetVelocity(arcShooter.getTargetVelocity() - RobotConstantsV1.velocityDownStep);
                    gamepad.resetTimer();
                }
                arcShooter.log(telemetry);
                break;
            case INTAKE:
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                if(gamepad.circle && gamepad.isGamepadReady()){
                    intake.switchDirection();
                    gamepad.resetTimer();
                }
                intake.log(telemetry);
                break;
        }
    }
}
