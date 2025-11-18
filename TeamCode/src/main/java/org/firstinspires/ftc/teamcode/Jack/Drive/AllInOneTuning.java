package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeDualMotorsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.TagIDToAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

import java.util.List;

@TeleOp
public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();
    public IntakeDualMotorsV1 dualIntake = new IntakeDualMotorsV1();

    public LimelightV1 limelight = new LimelightV1();
    public TagIDToAprilTag tagIDToAprilTag = new TagIDToAprilTag();
    public boolean firstIteration = true;
    public int loopUpdates = 0;

    public TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
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
        CAMERA,
        GRAPH
    }

    public modeSelected isModeSelected = modeSelected.NOT_SELECTED;
    public modes mode = modes.DRIVE;

    public int selectedMode = 0;

    @Override
    public void init() {
        gamepad.init(gamepad1, 0.3);
        mecDrive.init(hardwareMap, gamepad);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
        //intake.init(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry, telemetryManager);
        storageServo.init(hardwareMap);
        //limelight.init(hardwareMap, telemetry);
        dualIntake.init(hardwareMap);
    }

    @Override
    public void init_loop(){
        arcShooter.graph(multipleTelemetry);
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
        multipleTelemetry.update();
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
                arcShooter.run();
                if(gamepad.dpad_up && gamepad.isGamepadReady()){
                    arcShooter.setTargetRPM(arcShooter.getTargetRPM() + RobotConstantsV1.velocityUpStep);
                    gamepad.resetTimer();
                }
                else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
                    arcShooter.setTargetRPM(arcShooter.getTargetVelocity() - RobotConstantsV1.velocityDownStep);
                    gamepad.resetTimer();
                }
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
            case GRAPH:
                loopUpdates = loopUpdates + 1;
                arcShooter.graph(multipleTelemetry);
                break;
            case CAMERA:
                List<LLResultTypes.FiducialResult> latest = limelight.getFiducialResults();
                telemetry.addData("Latest: ", tagIDToAprilTag.getTag(latest.get(latest.toArray().length).getFiducialId()));
                break;
        }

    }
}
