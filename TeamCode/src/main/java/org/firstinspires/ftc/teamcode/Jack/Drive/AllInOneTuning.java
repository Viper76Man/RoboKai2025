package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeDualMotorsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

@TeleOp
public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();
    public IntakeDualMotorsV1 dualIntake = new IntakeDualMotorsV1();

    public boolean firstIteration = true;
    public PanelsTelemetry panels = PanelsTelemetry.INSTANCE;
    public MultipleTelemetry multipleTelemetry;

    public StorageServoV1 storageServo = new StorageServoV1();
    public enum modeSelected {
        SELECTED,
        NOT_SELECTED
    }

    public enum modes {
        DRIVE,
        SHOOT,
        INTAKE,
        STORAGE,
        PINPOINT,
        CAMERA
    }

    public modeSelected isModeSelected = modeSelected.NOT_SELECTED;
    public modes mode = modes.DRIVE;

    public int selectedMode = 0;

    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        mecDrive.init(hardwareMap, gamepad);
        arcShooter.init(hardwareMap, 0.2, 0, 0);
        //intake.init(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry, panels);
        storageServo.init(hardwareMap);
        dualIntake.init(hardwareMap);
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
        multipleTelemetry.addData("Selected tuning mode: ", mode.name());
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
                mecDrive.log(multipleTelemetry);
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
                multipleTelemetry.addData("Power for velocity: ", arcShooter.runToVelocity(arcShooter.getVelocity(), 6000));
                multipleTelemetry.addData("RPM: ", (arcShooter.getVelocity()));
                arcShooter.log(multipleTelemetry);
                arcShooter.graph(multipleTelemetry);
                break;
            case INTAKE:
                dualIntake.setPowers(RobotConstantsV1.INTAKE_POWER);
                if(gamepad.isGamepadReady() && gamepad.a){
                    dualIntake.switchDirections();
                    gamepad.resetTimer();
                }
                break;
            case STORAGE:
                if(firstIteration){
                    storageServo.setPosition(RobotConstantsV1.STORAGE_BALL_1);
                    firstIteration = false;
                }
                if(gamepad.dpad_up && gamepad.isGamepadReady()){
                    storageServo.setPosition(storageServo.getPosition() + RobotConstantsV1.storageServoStep);
                    gamepad.resetTimer();
                }
                else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
                    storageServo.setPosition(storageServo.getPosition() - RobotConstantsV1.storageServoStep);
                    gamepad.resetTimer();
                }
                storageServo.log(telemetry);
                break;
        }

    }
}
