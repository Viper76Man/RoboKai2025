package org.firstinspires.ftc.teamcode.Jack.Drive;

import android.os.Environment;
import android.provider.Settings;

import com.bylazar.field.CanvasRotation;
import com.bylazar.field.FieldPluginConfig;
import com.bylazar.field.FieldPresetParams;
import com.bylazar.field.PanelsField;
import com.bylazar.gamepad.PanelsGamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.andymark.AndyMarkColorSensor;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.BlueAutoPathsV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.RedAutoPathsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.TagIDToAprilTag;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.nio.channels.Pipe;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@TeleOp
public class AllInOneTuning extends OpMode {
    private static final Logger log = LoggerFactory.getLogger(AllInOneTuning.class);
    public GamepadV1 gamepad = new GamepadV1();
    public MecanumDriveOnly mecDrive = new MecanumDriveOnly();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();

    public boolean usePanelsGamepad = false;

    public LimelightV1 limelight = new LimelightV1();
    public TagIDToAprilTag tagIDToAprilTag = new TagIDToAprilTag();
    public FlickerServoV1 flicker = new FlickerServoV1();
    public boolean firstIteration = true;
    public int loopUpdates = 0;
    public int pipelineIndex = 0;

    public TelemetryManager telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    public MultipleTelemetry multipleTelemetry;
    public LoggerV1 logger = new LoggerV1();
    public GamepadV1 gamepad2 = new GamepadV1();

    public StorageServoV1 storageServo = new StorageServoV1();
    public SlotColorSensorV1 slot1Sensor = new SlotColorSensorV1();
    public enum modeSelected {
        SELECTED,
        NOT_SELECTED
    }

    public enum modes {
        DRIVE,
        AUTO_ALIGN_BLUE,
        AUTO_ALIGN_RED,
        SHOOT,
        INTAKE,
        FLICKER,
        STORAGE,
        COLOR_SENSORS,
        PINPOINT,
        CAMERA,
        LOG,
        GRAPH
    }

    public modeSelected isModeSelected = modeSelected.NOT_SELECTED;
    public modes mode = modes.DRIVE;
    public Follower follower;

    public int selectedMode = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
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
        slot1Sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
        Gamepad gamepad3 = PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1);
        gamepad2.init(gamepad3, 0.3);
        FieldPresetParams params = new FieldPresetParams();
        FieldPluginConfig config = new FieldPluginConfig();
        config.setExtraPresets(Collections.singletonList(params));
        PanelsField.INSTANCE.getField().setConfig(config);
        PanelsField.INSTANCE.getField().update();
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
            menuSelectUpdate(gamepad);
            menuSelectUpdate(gamepad2);
        }
        else {
            if(usePanelsGamepad) {
                tuningUpdate(gamepad2);
            }
            else {
                tuningUpdate(gamepad);
            }
        }
    }

    public void menuSelectUpdate(GamepadV1 gamepad_) {
        gamepad_.update();
        multipleTelemetry.addData("Selected tuning mode: ", mode.name());
        multipleTelemetry.update();
        if (gamepad_.isGamepadReady()) {
            if (gamepad_.dpad_up) {
                selectedMode -= 1;
                mode = modes.values()[Math.max(0, (selectedMode))];
                gamepad_.resetTimer();
            } else if (gamepad_.dpad_down) {
                selectedMode += 1;
                mode = modes.values()[Math.min((selectedMode), modes.values().length - 1)];
                gamepad_.resetTimer();
            }
            if (gamepad_.circle) {
                if(gamepad_.gamepad.id == gamepad2.gamepad.id){
                    usePanelsGamepad = true;
                }
                isModeSelected = modeSelected.SELECTED;
                gamepad_.resetTimer();
            }
        }
    }

    public void tuningUpdate(GamepadV1 gamepad){
        gamepad.update();
        follower.update();
        switch (mode){
            case DRIVE:
                mecDrive.drive();
                mecDrive.log(multipleTelemetry);
                break;
            case AUTO_ALIGN_BLUE:
                if(firstIteration) {
                    follower.setPose(BlueAutoPathsV1.startPose);
                    limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
                    limelight.startStreaming();
                    firstIteration = false;
                }
                localizer.drawToPanels(follower);
                multipleTelemetry.addData("Pose: ", follower.getPose());
                mecDrive.log(multipleTelemetry);
                mecDrive.driveWithRotationLock(Robot.Alliance.BLUE, follower.getPose(), telemetry, true);
                break;
            case AUTO_ALIGN_RED:
                if(firstIteration) {
                    follower.setPose(RedAutoPathsV1.startPose);
                    limelight.setPipeline(LimelightV1.Pipeline.RED_GOAL);
                    limelight.startStreaming();
                    firstIteration = false;
                }
                localizer.drawToPanels(follower);
                multipleTelemetry.addData("Pose: ", follower.getPose());
                mecDrive.log(multipleTelemetry);
                mecDrive.driveWithRotationLock(Robot.Alliance.RED, follower.getPose(), telemetry, true);
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
            case COLOR_SENSORS:
                telemetry.addData("RGB:", "("+ slot1Sensor.getRGB().asText() + ")");
                telemetry.addData("IS GREEN: ", slot1Sensor.isGreen());
                telemetry.addData("IS PURPLE: ", slot1Sensor.isPurple());
                break;
            case GRAPH:
                loopUpdates = loopUpdates + 1;
                arcShooter.graph(multipleTelemetry);
                break;
            case LOG:
                if(firstIteration) {
                    logger.open("robokailogs.txt");
                    logger.info("This was written from AllInOneTuning.java.");
                }
                break;
            case CAMERA:
                List <LimelightV1.Pipeline> pipelines = Arrays.asList(LimelightV1.Pipeline.OBELISK, LimelightV1.Pipeline.BLUE_GOAL, LimelightV1.Pipeline.RED_GOAL);
                if(gamepad.isGamepadReady() && gamepad.circle){
                    pipelineIndex += 1;
                    if(pipelineIndex > 2){
                        pipelineIndex = 0;
                    }
                    limelight.setPipeline(pipelines.get(pipelineIndex));
                    gamepad.resetTimer();
                }
                multipleTelemetry.addData("SELECTED PIPELINE: " + pipelineIndex, " (" + pipelines.get(pipelineIndex) + ")");
                LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
                if(latest_result != null) {
                    int latestID = latest_result.getFiducialId();
                    multipleTelemetry.addData("Latest: ", tagIDToAprilTag.getTag(latestID));
                    multipleTelemetry.addData("ID:", latestID);
                    multipleTelemetry.addData("X: ", latest_result.getTargetXDegrees());
                    multipleTelemetry.addData("Y: ", latest_result.getTargetYDegrees());
                    multipleTelemetry.addData("TARGET DISTANCE: ", limelight.getTargetDistance());
                    multipleTelemetry.update();
                }
                break;
        }

    }
}
