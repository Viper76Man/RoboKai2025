package org.firstinspires.ftc.teamcode.Jack.Motors;


import androidx.core.content.PermissionChecker;

import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class SpindexerMotorV2 implements Subsystem {
    public static final SpindexerMotorV2 INSTANCE = new SpindexerMotorV2();
    private final double firstPos = 0;  //0 degrees (INTAKE_BALL_1)
    private final double secondPos = 250.6; //-120 degrees (INTAKE_BALL_2)
    private final double thirdPos = 501.2;  //-240 degrees (INTAKE_BALL_3)
    private final double fourthPos = 751.8;  //360 degrees (SHOOT_WITH_1_BALL)
    private final double fifthPos = 1002.4;  //480 degrees
    private final double sixthPos = 1253; //600 degrees

    public Command lastFireCommand;

    private SpindexerMotorV2() {

    }

    public void init(BallManager manager){
        this.manager = manager;
    }



    public enum SpindexerState {
        INTAKE_BALL_1,
        INTAKE_BALL_2,
        INTAKE_BALL_3,
        SHOOT_WITH_1_BALL,
        SHOOT_WITH_2_BALLS,
        SHOOT_WITH_3_BALLS
    }
    public SpindexerState currentSpindexerState = SpindexerState.INTAKE_BALL_1;

    private MotorEx motor;
    public BallManager manager;



    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(RobotConstantsV1.spindexerPIDs.p, 0, RobotConstantsV1.spindexerPIDs.d)
            .build();

    public Command toFirstPos = new RunToPosition(controlSystem, firstPos).requires(this).then(setCurrentState(SpindexerState.INTAKE_BALL_1));
    public Command toSecondPos = new RunToPosition(controlSystem, secondPos).requires(this).then(setCurrentState(SpindexerState.INTAKE_BALL_2));
    public Command toThirdPos = new RunToPosition(controlSystem, thirdPos).requires(this).then(setCurrentState(SpindexerState.INTAKE_BALL_3));
    public Command toFourthPos = new RunToPosition(controlSystem, fourthPos).requires(this).then(setCurrentState(SpindexerState.SHOOT_WITH_1_BALL));
    public Command toFifthPos = new RunToPosition(controlSystem, fifthPos).requires(this).then(setCurrentState(SpindexerState.SHOOT_WITH_2_BALLS));
    public Command toSixthPos = new RunToPosition(controlSystem, sixthPos).requires(this).then(setCurrentState(SpindexerState.SHOOT_WITH_3_BALLS));


    @Override
    public void initialize() {
        motor = new MotorEx("spindexer").reversed();
        //reset encoder
    }

    public void resetMotor(){
        motor.zero();
    }

    @Override
    public void periodic() {
        motor.setPower(controlSystem.calculate(motor.getState()));
    }

    public void setState(SpindexerState state){
        this.currentSpindexerState = state;
    }

    private Command setCurrentState(SpindexerState state){
        return new Command() {
            boolean done = false;

            @Override
            public void update(){
                currentSpindexerState = state;
                done = true;
            }

            @Override
            public boolean isDone() {
                return done;
            }
        };
    }

    public void next(){
        switch (currentSpindexerState){
            case INTAKE_BALL_1:
                toSecondPos.schedule();
                manager.setCurrentBall(2);
                break;
            case INTAKE_BALL_2:
                toThirdPos.schedule();
                manager.setCurrentBall(3);
                break;
            case INTAKE_BALL_3:
                lastFireCommand = Fire();
                lastFireCommand.schedule();
                manager.setCurrentBall(4);
                break;
            case SHOOT_WITH_1_BALL:
            case SHOOT_WITH_2_BALLS:
            case SHOOT_WITH_3_BALLS:
            default:
                toFirstPos.schedule();
            break;
        }
    }

   public Command spindexerUpdate(){
        return new Command() {
            public void update() {
                spin();
            }

            @Override
            public boolean isDone() {
                return false;
            }
        };
   }

   public void spin() {
        switch (manager.mode) {
            case INTAKE:
                switch (manager.getCurrentBall()) {
                    case 1:
                        currentSpindexerState = SpindexerState.INTAKE_BALL_1;
                        break;
                    case 2:
                        currentSpindexerState = SpindexerState.INTAKE_BALL_2;
                        break;
                    case 3:
                        currentSpindexerState = SpindexerState.INTAKE_BALL_3;
                        break;
                }
                break;
        }
   }


    public boolean isActive(){
        switch (currentSpindexerState){
            case INTAKE_BALL_1:
                return toSecondPos.isScheduled() && !toSecondPos.isDone();
            case INTAKE_BALL_2:
                return toThirdPos.isScheduled() && !toThirdPos.isDone();
            case INTAKE_BALL_3:
                if(!Objects.equals(lastFireCommand, null)) {
                    return lastFireCommand.isScheduled() && !lastFireCommand.isDone();
                }
                else {
                    return false;
                }
            case SHOOT_WITH_1_BALL:
            case SHOOT_WITH_2_BALLS:
            case SHOOT_WITH_3_BALLS:
            default:
                toFirstPos.schedule();
                break;
        }
        return false;
    }

    public Command Fire(){
        switch (currentSpindexerState) {
            case INTAKE_BALL_1:
                    new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        toFourthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        toFirstPos
                    ).requires(this);

            case INTAKE_BALL_2:
                    new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        toFifthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        toFirstPos
                    ).requires(this);
            case INTAKE_BALL_3:
                return new ParallelRaceGroup(
                    new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        toSixthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        toFirstPos
                    )
                ).requires(this);
        }
        return null;
    }

}
