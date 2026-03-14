package org.firstinspires.ftc.teamcode.Jack.Motors;


import androidx.core.content.PermissionChecker;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;

import java.util.Objects;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.impl.MotorEx;

public class SpindexerMotorV2 implements Subsystem {
    public static final SpindexerMotorV2 INSTANCE = new SpindexerMotorV2(Robot.Mode.TELEOP);
    public static final SpindexerMotorV2 INSTANCE_AUTO = new SpindexerMotorV2(Robot.Mode.AUTONOMOUS);
    private final double firstPos = 0;  //0 degrees (INTAKE_BALL_1)
    private final double secondPos = 250.6; //-120 degrees (INTAKE_BALL_2)
    private final double thirdPos = 501.2;  //-240 degrees (INTAKE_BALL_3)
    private final double fourthPos = 751.8;  //360 degrees (SHOOT_WITH_1_BALL)
    private final double fifthPos = 1002.4;  //480 degrees
    private final double sixthPos = 1253; //600 degrees

    public Command lastFireCommand;
    public Robot.Mode mode;

    public SpindexerMotorV2(Robot.Mode mode) {
        this.mode = mode;
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

    public MotorEx motor;
    public BallManager manager;
    public boolean firstLoop = true;



    private final ControlSystem controlSystem = ControlSystem.builder()
            .posPid(0.005, 0, 0)
            .build();

    public Command toFirstPos = new RunToPosition(controlSystem, firstPos).requires(this);
    public Command toSecondPos = new RunToPosition(controlSystem, secondPos).requires(this);
    public Command toThirdPos = new RunToPosition(controlSystem, thirdPos).requires(this);
    public Command toFourthPos = new RunToPosition(controlSystem, fourthPos).requires(this);
    public Command toFifthPos = new RunToPosition(controlSystem, fifthPos).requires(this);
    public Command toSixthPos = new RunToPosition(controlSystem, sixthPos).requires(this);


    @Override
    public void initialize() {
        motor = new MotorEx("spindexer").reversed().brakeMode();
        if (mode == Robot.Mode.AUTONOMOUS) {
            motor.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            controlSystem.reset();
            motor.atPosition(0);
        }
    }


    public void runMotorToZero(){
        motor.zero();
    }

    @Override
    public void periodic() {
        if(firstLoop && mode == Robot.Mode.TELEOP) {
            motor.atPosition(motor.getMotor().getCurrentPosition());
            motor.zero();

        }
        if(firstLoop && new Range(0, 20).isInRange(motor.getState().getPosition())){
            firstLoop = false;
        }
        if(!firstLoop) {
            motor.setPower(controlSystem.calculate(motor.getState()));
        }
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

    public boolean isActive2(){
       switch (currentSpindexerState){
           case INTAKE_BALL_1:
               return ! new Range(firstPos, 20).isInRange(motor.getState().getPosition());
           case INTAKE_BALL_2:
               return ! new Range(secondPos, 20).isInRange(motor.getState().getPosition());
           case INTAKE_BALL_3:
               return ! new Range(thirdPos, 20).isInRange(motor.getState().getPosition());
           case SHOOT_WITH_1_BALL:
               return ! new Range(fourthPos, 20).isInRange(motor.getState().getPosition());
           case SHOOT_WITH_2_BALLS:
               return ! new Range(fifthPos, 20).isInRange(motor.getState().getPosition());
           case SHOOT_WITH_3_BALLS:
               return ! new Range(sixthPos, 20).isInRange(motor.getState().getPosition());
           default:
               return false;
       }
    }

    public Command Fire(){
        switch (currentSpindexerState) {
            case INTAKE_BALL_1:
                return new ParallelRaceGroup(
                    new SequentialGroup(
                            Ramp.INSTANCE.rampUp,
                            new Delay(.4),
                            toFourthPos,
                            Ramp.INSTANCE.rampDown,
                            new Delay(.4),
                            toFirstPos
                    )
                    , new Delay(5)
             ).requires(this);
            case INTAKE_BALL_2:
                return new ParallelRaceGroup(
                        new SequentialGroup(
                                Ramp.INSTANCE.rampUp,
                                new Delay(.4),
                                toFifthPos,
                                Ramp.INSTANCE.rampDown,
                                new Delay(.4),
                                toFirstPos
                        )
                        , new Delay(5)
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
                    , new Delay(5)
                ).requires(this);
        }
        return null;
    }

    public double getCurrentPos(){
        return motor.getCurrentPosition();
    }

    public double getStatePos(){
        return motor.getState().getPosition();
    }

}
