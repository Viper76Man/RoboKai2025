package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.DeliverySubsystem;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;

import java.util.Objects;

public class RobotV4 {
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public FlickerServoV2 flicker = new FlickerServoV2();
    public TurretServoCR turret = new TurretServoCR();
    public LimelightV1 limelight = new LimelightV1();
    public Follower follower;
    public PIDController controller;
    public IntakeV1 intake = new IntakeV1();
    public BallManager ballManager = new BallManager();
    public DeliverySubsystem delivery = new DeliverySubsystem();

    public boolean ballSet = false;
    //----------------------------------------------------------------------------------------------
    public HardwareMap hardwareMap;
    public Robot.Alliance alliance;
    public GamepadV1 gamepad;
    public enum State {
        INTAKE_BALL_1,
        INTAKE_BALL_2,
        INTAKE_BALL_3,
        SHOOT_BALL_1,
        SHOOT_BALL_2,
        SHOOT_BALL_3
    }

    public enum Mode {
        FIRE_SINGLE,
        FIRE_TRIPLE,
        INTAKE
    }

    public enum IntakeMode {
        FORWARD,
        REVERSE,
        IDLE
    }

    public State state = State.INTAKE_BALL_1;
    public Mode mode = Mode.INTAKE;
    public IntakeMode intakeMode = IntakeMode.FORWARD;

    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1, Robot.Mode mode, Robot.Alliance alliance){
        this.gamepad = gamepadV1;
        this.alliance = alliance;
        this.hardwareMap = hardwareMap;
        ballManager.setCurrentBall(1);
        delivery.init(flicker, arcShooter, spindexer, ballManager);
        initHardware();
        switch (mode){
            case TELEOP:
                break;
            case AUTONOMOUS:
                spindexer.resetEncoder();
                break;
        }
    }
    public void initHardware(){
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        limelight.init(hardwareMap);
        drive.init(hardwareMap, gamepad);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        spindexer.setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
        controller = new PIDController(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        sensor.sensor.setGain(10);
        switch (alliance){
            case RED:
                limelight.setPipeline(LimelightV1.Pipeline.RED_GOAL);
                break;
            case BLUE:
            case TEST:
                limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
                break;
        }
        limelight.startStreaming();
    }

    public void systemStatesUpdate(){
        intakeUpdate();
        gamepad.update();
        drive.drive();
        spindexerUpdate();
        delivery.update(gamepad);
        flicker.update(spindexer.isSpindexerReady());
        sensor.update(spindexer.state, spindexer.isSpindexerReady());
        setStateBasedOnBallManager();
        if(entireRobotFull() && mode == Mode.INTAKE){
            mode = Mode.FIRE_TRIPLE;
            ballManager.setCurrentBall(1);
        }
        else if(!entireRobotFull() && mode == Mode.INTAKE && gamepad.isGamepadReady() && gamepad.circle){
            gamepad.resetTimer();
            mode = Mode.FIRE_SINGLE;
            ballManager.setCurrentBall(1);
        }

        switch (state){
            case INTAKE_BALL_1:
                delivery.firedAlready = false;
                ball1Update();
                break;
            case INTAKE_BALL_2:
                ball2Update();
                break;
            case INTAKE_BALL_3:
                ball3Update();
                break;
            case SHOOT_BALL_1:
            case SHOOT_BALL_2:
            case SHOOT_BALL_3:
                arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                if(gamepad.right_trigger > 0.15 && gamepad.isGamepadReady() && (delivery.state == DeliverySubsystem.State.IDLE || delivery.state == DeliverySubsystem.State.SPIN_UP) && !delivery.firedAlready){
                    gamepad.resetTimer();
                    switch (mode){
                        case INTAKE:
                        case FIRE_TRIPLE:
                            mode = Mode.FIRE_TRIPLE;
                            delivery.fireTriple();
                            break;
                        case FIRE_SINGLE:
                            delivery.fireSingle();
                            break;
                    }
                }
                if(delivery.state == DeliverySubsystem.State.IDLE && delivery.firedAlready){
                    mode = Mode.INTAKE;
                    setSystemState(State.INTAKE_BALL_1);
                }
                break;
        }
    }

    public void spindexerUpdate(){
        spindexer.run();
        switch (mode){
            case INTAKE:
                switch (ballManager.getCurrentBall()){
                    case 1:
                        spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                        break;
                    case 2:
                        spindexer.setState(SpindexerMotorV1.State.BALL_2_INTAKE);
                        break;
                    case 3:
                        spindexer.setState(SpindexerMotorV1.State.BALL_3_INTAKE);
                        break;
                }
                break;
            case FIRE_TRIPLE:
            case FIRE_SINGLE:
                switch (ballManager.getCurrentBall()){
                    case 1:
                        spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                        break;
                    case 2:
                        spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                        break;
                    case 3:
                        spindexer.setState(SpindexerMotorV1.State.BALL_3_SHOOT);
                        break;
                }
            break;
        }
    }

    public void setStateBasedOnBallManager(){
        switch (mode) {
            case INTAKE:
                switch (ballManager.getCurrentBall()){
                    case 1:
                        setSystemState(State.INTAKE_BALL_1);
                        break;
                    case 2:
                        setSystemState(State.INTAKE_BALL_2);
                        break;
                    case 3:
                        setSystemState(State.INTAKE_BALL_3);
                        break;
                }
                break;
            case FIRE_SINGLE:
            case FIRE_TRIPLE:
                switch (ballManager.getCurrentBall()){
                    case 1:
                        setSystemState(State.SHOOT_BALL_1);
                        break;
                    case 2:
                        setSystemState(State.SHOOT_BALL_2);
                        break;
                    case 3:
                        setSystemState(State.SHOOT_BALL_3);
                        break;
                }
                if(entireRobotEmpty()){
                    mode = Mode.INTAKE;
                    ballManager.setCurrentBall(1);
                    setSystemState(State.INTAKE_BALL_1);
                }
                break;
        }
    }
    public void setBall(int ball){
        if(!ballSet) {
            ballManager.setCurrentBall(1);
        }
    }

    public void setSystemState(State newState){
        state = newState;
    }

    public boolean isColorSensorGreen(){
        return sensor.getCurrent() == ArtifactColor.GREEN && sensor.getNormalizedRGB().green > 0.03 && spindexer.isSpindexerReady(); // && sensor.getNormalizedRGB().green > 0.03
    }

    public boolean isColorSensorPurple(){
        return sensor.getCurrent() == ArtifactColor.PURPLE && sensor.getNormalizedRGB().green > 0.03 && spindexer.isSpindexerReady();
    }

    //----------------------------------------------------------------------------------------------
    public void ball1Update(){
        delivery.state = DeliverySubsystem.State.IDLE;
        intake.setState(IntakeV1.IntakeState.FORWARD);
        if(isColorSensorGreen()){
            ballManager.setBall1(ArtifactColor.GREEN);
            sensor.clear();
            ballManager.next();
        }
        else if(isColorSensorPurple()){
            ballManager.setBall1(ArtifactColor.PURPLE);
            sensor.clear();
            ballManager.next();
        }
    }

    public void ball2Update(){
        delivery.state = DeliverySubsystem.State.IDLE;
        intake.setState(IntakeV1.IntakeState.FORWARD);
        if(isColorSensorGreen()){
            ballManager.setBall2(ArtifactColor.GREEN);
            sensor.clear();
            ballManager.next();
        }
        else if(isColorSensorPurple()){
            ballManager.setBall2(ArtifactColor.PURPLE);
            sensor.clear();
            ballManager.next();
        }
    }

    public void ball3Update(){
        delivery.state = DeliverySubsystem.State.IDLE;
        intake.setState(IntakeV1.IntakeState.FORWARD);
        if(isColorSensorGreen()){
            ballManager.setBall3(ArtifactColor.GREEN);
            sensor.clear();
            ballManager.next();
        }
        else if(isColorSensorPurple()){
            ballManager.setBall3(ArtifactColor.PURPLE);
            sensor.clear();
            ballManager.next();
        }
    }

    public boolean entireRobotEmpty(){
        return ballManager.isEmpty(1) && ballManager.isEmpty(2) && ballManager.isEmpty(3);
    }

    public boolean entireRobotFull(){
        return !ballManager.isEmpty(1) && !ballManager.isEmpty(2) && !ballManager.isEmpty(3);
    }

    public void intakeUpdate(){
        switch (intakeMode){
            case IDLE:
                intake.setPower(0.4);
                intake.setDirection(RobotConstantsV1.intakeDirection);
                break;
            case FORWARD:
                intake.setDirection(RobotConstantsV1.intakeDirection);
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
                break;
            case REVERSE:
                intake.setDirection(inverse(RobotConstantsV1.intakeDirection));
                intake.setPower(0.85);
                break;
        }
    }

    public DcMotorSimple.Direction inverse(DcMotorSimple.Direction original){
        if (Objects.requireNonNull(original) == DcMotorSimple.Direction.FORWARD) {
            return DcMotorSimple.Direction.REVERSE;
        }
        return DcMotorSimple.Direction.FORWARD;
    }

    public void log(Telemetry telemetry){
        telemetry.addData("State: " , state.name());
        telemetry.addLine("Ball 1: " + ballManager.getSlot1().name());
        telemetry.addLine("Ball 2: " + ballManager.getSlot2().name());
        telemetry.addLine("Ball 3: " + ballManager.getSlot3().name());
        flicker.log(telemetry);
        telemetry.addData("Delivery State: " , delivery.state.name());
        telemetry.addData("Fired already? : " , delivery.firedAlready);
    }

}
