package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactSlot;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;

public class RobotV3 {
    //HARDWARE--------------------------------------------------------------------------------------
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public FlickerServoV2 flicker = new FlickerServoV2();
    public IntakeV1 intake = new IntakeV1();
    //VARIABLES-------------------------------------------------------------------------------------
    public HardwareMap hardwareMap;
    public TelemetryManager telemetryM;
    public GamepadV1 gamepad;
    public ArtifactSlot slot1 = new ArtifactSlot();
    public ArtifactSlot slot2 = new ArtifactSlot();
    public ArtifactSlot slot3 = new ArtifactSlot();
    public boolean firedAlready = false;
    public boolean fire = false;
    public int currentBall = 1;
    //TIMERS----------------------------------------------------------------------------------------
    public ElapsedTime stateTimer = new ElapsedTime();
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
    public MODE mode = MODE.INTAKE;

    public void init(HardwareMap hardwareMap, GamepadV1 gamepadV1, Robot.Mode mode, Robot.Alliance alliance){
        this.gamepad = gamepadV1;
        this.hardwareMap = hardwareMap;
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
        intake.setDirection(RobotConstantsV1.intakeDirection);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        spindexer.setTargetPos(RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }

    public void initArtifactSlots(){
        setEmpty(1);
        setEmpty(2);
        setEmpty(3);
    }


    public void systemStatesUpdate(){
        drive.drive();
        gamepad.update();
        flicker.update(spindexer.isSpindexerReady());
        sensor.update(spindexer.state, spindexer.isSpindexerReady());
        arcShooter.run();
        spindexerRun();
        if(gamepad.right_trigger > 0.15){
            fire = true;
        }
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
        if(mode == MODE.INTAKE) {
            if(isRightTriggerPressed() && gamepad.isGamepadReady()){
                mode = MODE.SHOOT;
                currentBall = getNextBall();
                sensor.clear();
                gamepad.resetTimer();
            }
            if (isFlickerMoving() && !isFlickerTravelingDown()) {
                flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
            }
            setIntakeOn();
            setShooterIdle();
            if (isCurrentBallGreen()) {
                setGreen(currentBall);
                sensor.clear();
                setSystemState(getNextState());
            } else if (isCurrentBallPurple()) {
                setPurple(currentBall);
                sensor.clear();
                setSystemState(getNextState());
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
                } else if (isFlickerDown() && firedAlready) {
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
        return false;
    }

    public int getNextBall(){
        switch (mode){
            case INTAKE:
                if(slot1.getColor() == ArtifactColor.NONE){
                    return 1;
                }
                else if(slot2.getColor() == ArtifactColor.NONE){
                    return 2;
                }
                else if(slot3.getColor() == ArtifactColor.NONE){
                    return 3;
                }
                return 1;
            case SHOOT:
                if(slot1.getColor() != ArtifactColor.NONE){
                    return 1;
                }
                else if(slot2.getColor() != ArtifactColor.NONE){
                    return 2;
                }
                else if(slot3.getColor() != ArtifactColor.NONE){
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
        intake.setPower(0.25);
    }

    //FLICKER---------------------------------------------------------------------------------------
    public boolean isFlickerMoving(){
       return flicker.getState() != FlickerServoV2.State.DOWN;
    }

    public boolean isFlickerTravelingDown(){
        return flicker.getState() == FlickerServoV2.State.TRAVEL_DOWN;
    }

    public boolean isFlickerDown(){
        return flicker.getState() == FlickerServoV2.State.DOWN && flicker.getStateTimerSeconds() > 0.5;
    }

    public void setFlickerUp(){
        flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_UP);
    }




    //----------------------------------------------------------------------------------------------
    public State getEmptyBall(){
        switch (state){
            case INTAKE_BALL_1:
            case INTAKE_BALL_2:
            case INTAKE_BALL_3:
                if(slot1.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_1;
                }
                else if(slot2.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_2;
                }
                else if(slot3.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case SHOOT_BALL_1:
            case SHOOT_BALL_2:
            case SHOOT_BALL_3:
                if(slot1.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_1;
                }
                else if(slot2.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_2;
                }
                else if(slot3.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_3;
                }
                return State.INTAKE_BALL_1;
            default:
                return State.INTAKE_BALL_1;
        }
    }

    public State getNextState(){
        switch (state){
            case INTAKE_BALL_1:
                if(slot2.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_2;
                }
                else if(slot3.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_2:
                if(slot1.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_1;
                }
                else if(slot3.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_3;
                }
                return State.SHOOT_BALL_1;
            case INTAKE_BALL_3:
                if(slot1.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_1;
                }
                else if(slot2.getColor() == ArtifactColor.NONE){
                    return State.INTAKE_BALL_2;
                }
                return State.SHOOT_BALL_1;
            case SHOOT_BALL_1:
            case SHOOT_BALL_2:
            case SHOOT_BALL_3:
                if(slot1.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_1;
                }
                else if(slot2.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_2;
                }
                else if(slot3.getColor() != ArtifactColor.NONE){
                    return State.INTAKE_BALL_3;
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
}
