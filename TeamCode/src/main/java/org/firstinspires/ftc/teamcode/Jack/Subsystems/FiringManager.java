package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

import java.util.Objects;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class FiringManager implements Subsystem {
    public BallManager manager;
    public FlickerSubsystem.FlickUp command, command2, command3;
    public FlickerSubsystem flicker;
    public SpindexerMotorV1 spindexer;
    public static boolean done = false;
    public LimelightSubsystem ll;

    public void init(BallManager manager, FlickerSubsystem flicker, SpindexerMotorV1 spindexer, LimelightSubsystem limelightSub) {
        this.flicker = flicker;
        this.command = flicker.fire();
        this.manager = manager;
        this.spindexer = spindexer;
        this.command2 = flicker.fire();
        this.command3 = flicker.fire();
        this.ll = limelightSub;
    }

    public FireTriple fireTriple(Robot.Mode mode, ArcMotorsV2 arcMotorsV2) {
        return new FireTriple(true, mode, arcMotorsV2);
    }

    public FireTriple fireSingle(ArcMotorsV2 arc) {
        return new FireTriple(false, Robot.Mode.TELEOP, arc);
    }

    public enum States {
        START,
        FLICKER_MOVING_UP,
        FLICKER_UP,
        SPIN,
        FLICKER_MOVING_DOWN,
        DOWN
    }

    public class FireTriple extends Command {
        private boolean firing = false;
        public States state = States.START;

        public int ballsHeld = 0;
        public double runs = 0;

        public int ball1 = 0;
        public int ball2 = 0;

        private Command activeFire;
        public boolean triple;
        public Robot.Mode mode;
        public ArcMotorsV2 arc;
        public ElapsedTime fireTimer = new ElapsedTime();
        public boolean spindexerSet = false;
        public boolean finished = false;

        public FireTriple(boolean triple, Robot.Mode mode, ArcMotorsV2 arcMotorsV2) {
            this.triple = triple;
            this.mode = mode;
            this.arc = arcMotorsV2;
            activeFire = flicker.fire();
        }

        @Override
        public void start() {
            firing = false;
        }

        @Override
        public void update() {
            flicker.flicker.newUpdate();
            if (triple) {
                if (!firing && spindexer.isSpindexerReady() && !spindexerSet && state == States.START) {
                    ballsHeld = getBallsHeld();
                    ball1 = getBall1();
                    ball2 = getBall2();
                    if (mode == Robot.Mode.AUTONOMOUS && arc.arcShooter.isInRange(75)) {
                        startFiring();
                        done = false;
                    } else if (mode != Robot.Mode.AUTONOMOUS) {
                        done = false;
                        startFiring();
                    }
                }
                if (state == States.FLICKER_MOVING_UP){
                    if(!spindexer.isSpindexerReady()){
                        fireTimer.reset();
                    }
                    else {
                        //activeFire.schedule();
                        flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_UP);
                        if (fireTimer.seconds() > 0.2) {
                            state = States.FLICKER_UP;
                            fireTimer.reset();
                        }
                    }
                }
                if (state == States.FLICKER_UP) {
                    if (fireTimer.seconds() > 0.1) {
                        state = States.SPIN;
                        fireTimer.reset();
                    }
                }
                if (state == States.SPIN) {
                    if (!spindexer.isInRange(100)) {
                        fireTimer.reset();
                    }
                    if (fireTimer.seconds() > 0.1 && fireTimer.seconds() < 1.3) {
                        spindexerSet = true;
                        manager.setMode(BallManager.State.SHOOT);
                        if(!manager.isEmpty(2) && manager.isEmpty(3)){
                            spindexer.setState(SpindexerMotorV1.State.SHOOT_ALL_RAMP_2_BALLS);
                            manager.setCurrentBall(5);
                        }
                        else {
                            spindexer.setState(SpindexerMotorV1.State.SHOOT_ALL_RAMP);
                            manager.setCurrentBall(4);
                        }
                        activeFire.cancel();
                        state = States.FLICKER_MOVING_DOWN;
                    }
                }
                if (state == States.FLICKER_MOVING_DOWN) {
                    if(!RobotConstantsV1.usingFlywheelWeights) {
                        double dist = ll.limelight.getTargetDistance();
                        if (!Objects.equals(dist, null) && dist > 90) {
                            switch (ballsHeld) {
                                case 3:
                                    if (Math.abs(spindexer.getCurrentPosition()) > 310) {
                                        arc.disablePIDs();
                                        arc.arcShooter.setMotorPower(0.85);
                                    }
                                    if (Math.abs(spindexer.getCurrentPosition()) > 622) {
                                        arc.disablePIDs();
                                        arc.arcShooter.setMotorPower(0.9);
                                    }
                                    break;
                                case 2:
                                    switch (ball2) {
                                        case 2:
                                            if (Math.abs(spindexer.getCurrentPosition()) > 310) {
                                                arc.disablePIDs();
                                                arc.arcShooter.setMotorPower(0.8);
                                            }
                                            break;
                                        case 3:
                                            if (Math.abs(spindexer.getCurrentPosition()) > 622) {
                                                arc.disablePIDs();
                                                arc.arcShooter.setMotorPower(0.8);
                                            }
                                            break;
                                    }
                                    break;
                            }
                        }
                    }
                    if (fireTimer.seconds() > 1.3 && (Math.abs(spindexer.getCurrentPosition()) > Math.abs(RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_SHOOT) + 10)) {
                        activeFire.cancel();
                        flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
                        state = States.DOWN;
                    }
                    if (fireTimer.seconds() > 2) {
                        activeFire.cancel();
                        flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
                        state = States.DOWN;
                    }
                }
                if (state == States.DOWN) {
                    flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
                    runs += 1;
                    finished = true;
                }
            }
        }

        @Override
        public boolean isDone() {
            return false;
        }

        @Override
        public void stop(boolean interrupted){
            flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
            arc.enablePIDs();
            state = States.START;
            firing = false;
            spindexerSet = false;
        }

        public int getBallsHeld(){
            int sum = 0;
            if(!manager.isEmpty(1)){
                sum += 1;
            }
            if(!manager.isEmpty(2)){
                sum += 1;
            }
            if(!manager.isEmpty(3)){
                sum += 1;
            }
            return sum;
        }

        public int getBall1(){
            if(!manager.isEmpty(1)){
                return 1;
            }
            else if(!manager.isEmpty(2)){
                return 2;
            }
            else {
                return 3;
            }
        }

        public int getBall2(){
            if(ball1 == 3){
                return 0;
            }
            else if(!manager.isEmpty(2)){
                return 2;
            }
            else if(!manager.isEmpty(3)){
                return 3;
            }
            return 0;
        }

        public void startFiring(){
            if(!manager.isEmpty(2) && manager.isEmpty(3)) {
                manager.setMode(BallManager.State.SHOOT);
                spindexer.setState(SpindexerMotorV1.State.BALL_2_SHOOT);
                manager.setCurrentBall(2);
            }
            else {
                manager.setMode(BallManager.State.SHOOT);
                spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                manager.setCurrentBall(1);
            }
            state = States.FLICKER_MOVING_UP;
            fireTimer.reset();
            firing = true;
            finished = false;
        }
    }
}
