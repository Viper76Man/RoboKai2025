package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.os.Environment;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.TagIDToAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;
import org.slf4j.LoggerFactory;

import java.util.List;

@TeleOp
public class AllInOneTuning extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();

    public LimelightV1 limelight = new LimelightV1();
    public TagIDToAprilTag tagIDToAprilTag = new TagIDToAprilTag();
    public FlickerServoV1 flicker = new FlickerServoV1();
    public boolean firstIteration = true;
    public int loopUpdates = 0;

    public TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    public MultipleTelemetry multipleTelemetry;
    public LoggerV1 logger = new LoggerV1();

    public StorageServoV1 storageServo = new StorageServoV1();
    public enum modeSelected {
        SELECTED,
        NOT_SELECTED
    }

    public enum modes {
        DRIVE,
        SHOOT,
        INTAKE,
        FLICKER,
        STORAGE,
        PINPOINT,
        CAMERA,
        LOG,
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
        flicker.init(hardwareMap);
        intake.init(hardwareMap);
        logger.init(telemetry);
        limelight.init(hardwareMap, telemetry);
        limelight.limelight.pipelineSwitch(0);
    }

    @Override
    public void init_loop(){
        arcShooter.graph(multipleTelemetry);
    }

    @Override
    public void start(){
        limelight.startStreaming();
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
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                if(gamepad.isGamepadReady() && gamepad.a){
                    intake.switchDirection();
                    gamepad.resetTimer();
                }
                break;
            case FLICKER:
                if(firstIteration){
                    flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
                    firstIteration = false;
                }
                if(gamepad.isGamepadReady() && gamepad.circle){
                    flicker.switchPositions();
                }
                flicker.log(multipleTelemetry);
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
            case LOG:
                if(firstIteration) {
                    String path = Environment.getExternalStorageDirectory().getAbsolutePath();
                    String file = path + "/hi.txt";
                    if (logger.exists(file)) {
                        int status = logger.openFile(file, false);
                        logger.info("INIT");
                        multipleTelemetry.addLine("File open status: " + status);
                    } else {
                        int status = logger.openFile(file, true);
                        logger.info("HELLOWORLD");
                        multipleTelemetry.addLine("File open status: " + status);
                    }
                    multipleTelemetry.update();
                    for (String line : logger.readLines(file)) {
                        multipleTelemetry.addLine(line);
                        multipleTelemetry.update();
                    }
                    firstIteration = false;
                }
                break;
            case CAMERA:
                List<LLResultTypes.FiducialResult> latest = limelight.getFiducialResults();
                if(!latest.isEmpty()) {
                    LLResultTypes.FiducialResult latest_result = latest.get(latest.toArray().length - 1);
                    int latestID = latest_result.getFiducialId();
                    multipleTelemetry.addData("Latest: ", tagIDToAprilTag.getTag(latestID));
                    multipleTelemetry.addData("ID:", latestID);
                    multipleTelemetry.addData("X: ", latest_result.getTargetXDegrees());
                    multipleTelemetry.addData("Y: ", latest_result.getTargetYDegrees());
                    multipleTelemetry.update();
                }
                break;
        }

    }
}
