package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactSlot;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;

import kotlin.time.TestTimeSource;

public class RobotV3 {
    //HARDWARE--------------------------------------------------------------------------------------
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public FlickerServoV2 flicker = new FlickerServoV2();
    public TurretServoCR turret = new TurretServoCR();
    public LimelightV1 limelight = new LimelightV1();
    public PIDController controller;
    public IntakeV1 intake = new IntakeV1();
    //VARIABLES-------------------------------------------------------------------------------------
    public HardwareMap hardwareMap;
    public TelemetryManager telemetryM;
    public GamepadV1 gamepad;
    public Robot.Alliance alliance = Robot.Alliance.TEST;
    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();
    public boolean firedAlready = false;
    public boolean fire = false;
    public int lastBallFound = 0;
    public int currentBall = 1;
    public double cameraTx = 0;
    public double latestTagID = -1;
    public double TURRET_OFFSET_MULTIPLIER = 1;
    //TIMERS----------------------------------------------------------------------------------------
    public ElapsedTime stateTimer = new ElapsedTime();
    public ElapsedTime noResultTimer = new ElapsedTime();
    public ElapsedTime buttonHoldTimer = new ElapsedTime();
    //----------------------------------------------------------------------------------------------

    public enum State {
        INTAKE_BALL_1,
        INTAKE_BALL_2,
        INTAKE_BALL_3,
        SHOOT_BALL_1,
        SHOOT_BALL_2,
        SHOOT_BALL_3
    }

    public enum ArcState {
        FRONT,
        BACK,
        IDLE
    }

    public enum MODE {
        SHOOT,
        INTAKE
    }

    public State state = State.INTAKE_BALL_1;
    public ArcState arcState = ArcState.IDLE;
    public boolean intakeReversed = false;
    public MODE mode = MODE.INTAKE;

    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1, Robot.Mode mode, Robot.Alliance alliance){
        this.gamepad = gamepadV1;
        this.hardwareMap = hardwareMap;
        this.alliance = alliance;
        initHardware();
        initArtifactSlots();
        switch (mode){
            case TELEOP:
            case AUTONOMOUS:
                break;
        }
        if(RobotConstantsV1.panelsEnabled){
            telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        }
    }

    public void initHardware(){
        drive.init(hardwareMap, gamepad);
        intake.init(hardwareMap);
        turret.init(hardwareMap);
        limelight.init(hardwareMap);
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
                TURRET_OFFSET_MULTIPLIER *= -1;
                break;
            case BLUE:
            case TEST:
                limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
                break;
        }
        limelight.startStreaming();
    }

    public void initArtifactSlots(){
        setEmpty(1);
        setEmpty(2);
        setEmpty(3);
    }


    public void systemStatesUpdate(){
        drive.drive();
        gamepad.update();
        turretUpdate();
        flicker.update(spindexer.isSpindexerReady());
        sensor.update(spindexer.state, spindexer.isSpindexerReady());
        intakeUpdate();
        arcShooter.run();
        arcShooter.updatePIDsFromConstants();
        spindexerRun();
        switch (state){
            case INTAKE_BALL_1:
                spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                mode = MODE.INTAKE;
                currentBall = 1;
                break;
            case INTAKE_BALL_2:
                spindexer.setState(SpindexerMotorV1.State.BALL_2_INTAKE);
                mode = MODE.INTAKE;
                currentBall = 2;
                break;
            case INTAKE_BALL_3:
                spindexer.setState(SpindexerMotorV1.State.BALL_3_INTAKE);
                mode = MODE.INTAKE;
                currentBall = 3;
                break;
            case SHOOT_BALL_1:
                spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                mode = MODE.SHOOT;
                currentBall = 1;
                break;
            case SHOOT_BALL_2:
                spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                mode = MODE.SHOOT;
                currentBall = 2;
                break;
            case SHOOT_BALL_3:
                spindexer.setState(SpindexerMotorV1.State.BALL_3_SHOOT);
                mode = MODE.SHOOT;
                currentBall = 3;
                break;
        }

        //INTAKE-SET-------------------------------------------------------------------------------------
        if(gamepad.left_trigger > 0.15 && gamepad.isGamepadReady()){
            intakeReversed = !intakeReversed;
            gamepad.resetTimer();
        }
        if(mode == MODE.INTAKE) {
            if(intakeReversed){
                setEmpty(currentBall);
            }
            if(isRightTriggerPressed() && buttonHoldTimer.seconds() > 1 && gamepad.isGamepadReady() && stateTimer.seconds() > 0.5) {
                mode = MODE.SHOOT;
                currentBall = getNextBall();
                setSystemState(getState(currentBall, mode));
                sensor.clear();
                gamepad.resetTimer();
            }
            if(!isRightTriggerPressed()) {
                buttonHoldTimer.reset();
            }
            if (isFlickerMoving() && !isFlickerTravelingDown()) {
                flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
            }
            setIntakeOn();
            setShooterIdle();
            if (isCurrentBallGreen() && lastBallFound != currentBall && isEmpty(currentBall) && (sensor.sensor.getNormalizedColors().green / sensor.sensor.getNormalizedColors().alpha) < 0.15 && sensor.sensor.getNormalizedColors().green > 0.03) {
                setGreen(currentBall);
                sensor.clear();
                setSystemState(getNextState());
                lastBallFound = currentBall;
            } else if (isCurrentBallPurple() && lastBallFound != currentBall && isEmpty(currentBall) && (sensor.sensor.getNormalizedColors().green / sensor.sensor.getNormalizedColors().alpha) < 0.15 && sensor.sensor.getNormalizedColors().green > 0.03) {
                setPurple(currentBall);
                sensor.clear();
                setSystemState(getNextState());
                lastBallFound = currentBall;
            }
            firedAlready = false;
        }
        //SHOOT-SET---------------------------------------------------------------------------------------
        else {
            setIntakeIdle();
            if(isRightTriggerPressed() && !fire){
                fire = true;
            }
            if(fire) {
                if (isFlickerDown() && !firedAlready) {
                    setFlickerUp();
                    firedAlready = true;
                }
                if (isFlickerDown() && firedAlready) {
                    setEmpty(currentBall);
                    setSystemState(getNextState());
                    firedAlready = false;
                    fire = false;
                }
            }
            setShooterActiveBack();
        }
    }

    public void setSystemState(State state_){
        state = state_;
        stateTimer.reset();
    }

    public boolean isRightTriggerPressed(){
        return gamepad.right_trigger > 0.15;
    }


    //FUNCTIONS-------------------------------------------------------------------------------------

    public boolean isSpindexerReady(){
        return spindexer.isSpindexerReady();
    }

    //COLOR-SENSOR----------------------------------------------------------------------------------
    public boolean isCurrentBallGreen() {
        return sensor.getCurrent() == ArtifactColor.GREEN;
    }

    public boolean isCurrentBallPurple() {
        return sensor.getCurrent() == ArtifactColor.PURPLE ;
    }

    public boolean isCurrentBallEmpty() {
        return sensor.getCurrent() == ArtifactColor.NONE;
    }

    //BALL-CONTROL----------------------------------------------------------------------------------

    public void setEmpty(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.NONE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.NONE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.NONE);
                break;
        }
    }

    public void setGreen(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.GREEN);
                break;
            case 2:
                slot2.setColor(ArtifactColor.GREEN);
                break;
            case 3:
                slot3.setColor(ArtifactColor.GREEN);
                break;
        }
    }
    public void setPurple(int ball){
        switch (ball){
            case 1:
                slot1.setColor(ArtifactColor.PURPLE);
                break;
            case 2:
                slot2.setColor(ArtifactColor.PURPLE);
                break;
            case 3:
                slot3.setColor(ArtifactColor.PURPLE);
                break;
        }
    }
    public boolean isEmpty(int ball){
        switch (ball){
            case 1:
                return slot1.getColor() == ArtifactColor.NONE;
            case 2:
                return slot2.getColor() == ArtifactColor.NONE;
            case 3:
                return slot3.getColor() == ArtifactColor.NONE;
        }
        return true;
    }

    public int getNextBall(){
        switch (mode){
            case INTAKE:
                if(isEmpty(1)){
                    return 1;
                }
                else if(isEmpty(2)){
                    return 2;
                }
                else if(isEmpty(3)){
                    return 3;
                }
                return 1;
            case SHOOT:
                if(!isEmpty(1)){
                    return 1;
                }
                else if(!isEmpty(2)){
                    return 2;
                }
                else if(!isEmpty(3)){
                    return 3;
                }
                return 1;
            default:
                return 1;
            }
        }

    //SPINDEXER-------------------------------------------------------------------------------------
    public void spindexerRun(){
        switch (mode) {
            case SHOOT:
                switch (currentBall) {
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
            case INTAKE:
                switch (currentBall) {
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
        }
        spindexer.run();
    }
    //ARC-SHOOTER-----------------------------------------------------------------------------------
    public void setShooterIdle(){
        arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
        arcState = ArcState.IDLE;
    }
    public void setShooterActiveBack(){
        arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
        arcState = ArcState.BACK;
    }

    public void setShooterActiveFront(){
        arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
        arcState = ArcState.FRONT;
    }

    //INTAKE----------------------------------------------------------------------------------------
    public void setIntakeOn() {
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
    }

    public void setIntakeIdle(){
        if(!intakeReversed) {
            intake.setPower(0.2);
        }
        else {
            intake.setPower(0.7);
        }
    }

    public void intakeUpdate(){
        if(intakeReversed){
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
    }

    //FLICKER---------------------------------------------------------------------------------------
    public boolean isFlickerMoving(){
       return flicker.getState() != FlickerServoV2.State.IDLE;
    }

    public boolean isFlickerTravelingDown(){
        return flicker.getState() == FlickerServoV2.State.TRAVEL_DOWN;
    }

    public boolean isFlickerDown(){
        return flicker.getState() == FlickerServoV2.State.IDLE;
    }

    public void setFlickerUp(){
        flicker.setState(FlickerServoV2.State.DOWN);
    }




    //----------------------------------------------------------------------------------------------

    public State getState(int ball, MODE mode){
        switch (mode){
            case INTAKE:
                if(ball == 1){
                    return State.INTAKE_BALL_1;
                }
                else if(ball == 2){
                    return State.INTAKE_BALL_2;
                }
                else if(ball == 3){
                    return State.INTAKE_BALL_3;
                }
                return State.INTAKE_BALL_1;
            case SHOOT:
                if(ball == 1){
                    return State.SHOOT_BALL_1;
                }
                else if(ball == 2){
                    return State.SHOOT_BALL_2;
                }
                else if(ball == 3){
                    return State.SHOOT_BALL_3;
                }
                return State.SHOOT_BALL_1;
            default:
                return State.INTAKE_BALL_1;
        }
    }
    public State getNextState(){
        switch (state){
            case INTAKE_BALL_1:
                if(isEmpty(2)){
                    return State.INTAKE_BALL_2;
                }
                else if(isEmpty(3)){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_2:
                if(isEmpty(1)){
                    return State.INTAKE_BALL_1;
                }
                else if(isEmpty(3)){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_3:
                if(isEmpty(1)){
                    return State.INTAKE_BALL_1;
                }
                else if(isEmpty(2)){
                    return State.INTAKE_BALL_2;
                }
                return State.SHOOT_BALL_1;
            case SHOOT_BALL_1:
            case SHOOT_BALL_2:
            case SHOOT_BALL_3:
                if(!isEmpty(1)){
                    return State.SHOOT_BALL_1;
                }
                else if(!isEmpty(2)){
                    return State.SHOOT_BALL_2;
                }
                else if(!isEmpty(3)){
                    return State.SHOOT_BALL_3;
                }
                return State.INTAKE_BALL_1;
            default:
                return State.INTAKE_BALL_1;
        }
    }
    //----------------------------------------------------------------------------------------------
    public void log(TelemetryManager telemetryM, Telemetry telemetry){
        if(RobotConstantsV1.panelsEnabled) {
            telemetryM.addLine("STATE: " + state.name());
            telemetryM.addLine("Slot 1: " + slot1.getColor().name());
            telemetryM.addLine("Slot 2: " + slot2.getColor().name());
            telemetryM.addLine("Slot 3: " + slot3.getColor().name());
            telemetryM.addLine("Voltage (shooter right): " + arcShooter.shooter.getCurrent(CurrentUnit.AMPS));
            telemetryM.addLine("Voltage (shooter left): " + arcShooter.shooter2.getCurrent(CurrentUnit.AMPS));
            telemetryM.addLine("Voltage (intake): " + intake.getCurrent());
            telemetryM.addLine("Flicker pos: " + flicker.position);
            telemetryM.addLine("Flicker state: " + flicker.getState().name());
            telemetryM.addLine("Flicker timer (seconds): " + flicker.getStateTimerSeconds());
            telemetryM.addLine("Camera Tx: " + cameraTx);
            arcShooter.graph(telemetryM);
            drive.log(telemetry);

        }
        else {
            telemetryM.addLine("STATE: " + state.name());
            telemetry.addLine("Slot 1: " + slot1.getColor().name());
            telemetry.addLine("Slot 2: " + slot2.getColor().name());
            telemetry.addLine("Slot 3: " + slot3.getColor().name());
            telemetry.addLine("Voltage (shooter motors): " + (arcShooter.shooter.getCurrent(CurrentUnit.AMPS) + arcShooter.shooter2.getCurrent(CurrentUnit.AMPS)));
            telemetry.addLine("Voltage (intake): " + intake.getCurrent());
            arcShooter.graph(telemetry);
            drive.log(telemetry);
        }
    }
    //TURRET----------------------------------------------------------------------------------------
    public void turretUpdate(){
        double power;
        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if(latest_result != null) {
            latestTagID = latest_result.getFiducialId();
            cameraTx = latest_result.getTargetXDegreesNoCrosshair();
            noResultTimer.reset();
            power = -controller.getOutput(cameraTx + (RobotConstantsV1.TURRET_OFFSET_ANGLE * TURRET_OFFSET_MULTIPLIER));
        }
        else {
            cameraTx = 0;
            power = -controller.getOutput((int)turret.getEncoderPos(), 236);
        }
        if(turret.getEncoderPos() >= RobotConstantsV1.TURRET_MAX_ENCODER_VALUE && power < 0){
            power = 0;
        }
        if(Math.abs((cameraTx + (RobotConstantsV1.TURRET_OFFSET_ANGLE * TURRET_OFFSET_MULTIPLIER))) < RobotConstantsV1.degreeToleranceCamera){
            power = power / 3;
        }
        //if(noResultTimer.seconds() > 1){
            //turret.setPower(0);

        //}

        controller.updatePIDsFromConstants(RobotConstantsV1.turretPIDs);
        turret.setPower(power);
    }
}
