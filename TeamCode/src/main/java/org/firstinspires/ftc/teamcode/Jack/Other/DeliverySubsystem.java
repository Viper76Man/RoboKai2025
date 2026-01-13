package org.firstinspires.ftc.teamcode.Jack.Other;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV3;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV2;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

import java.util.List;
import java.util.Objects;

public class DeliverySubsystem {
    public ArcShooterV1 arc;
    public FlickerServoV2 flicker;

    public boolean readyToShoot = false;
    public boolean doneShooting = true;


    public boolean firedAlready = false;

    public enum State {
        IDLE,
        SPIN_UP,
        READY_TO_SHOOT,
        FIRING,
        FIRED
    }

    public enum ShootingZone {
        FRONT,
        BACK
    }

    public enum FireMode {
        SINGLE,
        TRIPLE
    }

    public ShootingZone zone = ShootingZone.BACK;
    public State state = State.IDLE;
    public FireMode fireMode = FireMode.TRIPLE;
    public SpindexerMotorV1 spindexer;
    public BallManager ballManager;

    public void init(FlickerServoV2 flicker, ArcShooterV1 arcShooter, SpindexerMotorV1 spindexer, BallManager ballManager){
        this.flicker = flicker;
        this.arc = arcShooter;
        this.spindexer = spindexer;
        this.ballManager = ballManager;
    }

    public void update(){
        arc.run();
        switch (state){
            case IDLE:
                arc.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
                break;
            case SPIN_UP:
                firedAlready = false;
                switch (zone){
                    case FRONT:
                        arc.setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
                        break;
                    case BACK:
                        arc.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                        break;
                }
                if(arc.isInRange(20)){
                    setState(State.READY_TO_SHOOT);
                }
                break;
            case READY_TO_SHOOT:
                readyToShoot = true;
                if(flicker.getState() == FlickerServoV2.State.IDLE && !firedAlready){
                    flicker.setState(FlickerServoV2.State.DOWN);
                    setState(State.FIRING);
                }
                break;
            case FIRING:
                if (flicker.getState() == FlickerServoV2.State.IDLE && firedAlready) {
                    ballManager.setEmpty(ballManager.getCurrentBall());
                    setState(State.FIRED);
                }
                break;
            case FIRED:
                switch (fireMode){
                    case TRIPLE:
                        if(allSlotsAreEmpty()){
                            setState(State.IDLE);
                        }
                        else {
                            firedAlready = true;
                        }
                        break;
                    case SINGLE:
                        doneShooting = true;
                        firedAlready = true;
                }
        }
    }

    public void fireTriple(){
        doneShooting = false;
        firedAlready = false;
        readyToShoot = false;
        fireMode = FireMode.TRIPLE;
        setState(State.SPIN_UP);
    }

    public void fireSingle(){
        doneShooting = false;
        firedAlready = false;
        readyToShoot = false;
        fireMode = FireMode.SINGLE;
        setState(State.SPIN_UP);
    }

    private void setState(State state){
        this.state = state;
    }

    public void setZone(ShootingZone zone){
        this.zone = zone;
    }

    public boolean allSlotsAreEmpty(){
        if(Objects.equals(getSlot1(), null)){
            return true;
        }
        else {
            return isEmpty(1) && isEmpty(2) && isEmpty(3);
        }
    }

    public ArtifactColor getSlot1(){
        return ballManager.getSlot1();
    }

    public ArtifactColor getSlot2(){
        return ballManager.getSlot2();
    }

    public ArtifactColor getSlot3(){
        return ballManager.getSlot3();
    }

    public boolean isEmpty(int ball){
        switch (ball) {
            case 1:
                return Objects.equals(getSlot1(), ArtifactColor.NONE);
            case 2:
                return Objects.equals(getSlot2(), ArtifactColor.NONE);
            case 3:
                return Objects.equals(getSlot3(), ArtifactColor.NONE);
        }
        return true;
    }

    public RobotV3.State getNextState(){
        switch (spindexer.state){
            case BALL_1_INTAKE:
                if(isEmpty(2)){
                    return RobotV3.State.INTAKE_BALL_2;
                }
                else if(isEmpty(3)){
                    return RobotV3.State.INTAKE_BALL_3;
                }
                return RobotV3.State.INTAKE_BALL_1;
            case BALL_2_INTAKE:
                if(isEmpty(1)){
                    return RobotV3.State.INTAKE_BALL_1;
                }
                else if(isEmpty(3)){
                    return RobotV3.State.INTAKE_BALL_3;
                }
                return RobotV3.State.INTAKE_BALL_2;
            case BALL_3_INTAKE:
                if(isEmpty(1)){
                    return RobotV3.State.INTAKE_BALL_1;
                }
                else if(isEmpty(2)){
                    return RobotV3.State.INTAKE_BALL_2;
                }
                return RobotV3.State.INTAKE_BALL_3;
            case BALL_1_SHOOT:
            case BALL_2_SHOOT:
            case BALL_3_SHOOT:
                if(!isEmpty(1)){
                    return RobotV3.State.SHOOT_BALL_1;
                }
                else if(!isEmpty(2)){
                    return RobotV3.State.SHOOT_BALL_2;
                }
                else if(!isEmpty(3)){
                    return RobotV3.State.SHOOT_BALL_3;
                }
                return RobotV3.State.SHOOT_BALL_1;
            default:
                return RobotV3.State.INTAKE_BALL_1;
        }
    }




}
