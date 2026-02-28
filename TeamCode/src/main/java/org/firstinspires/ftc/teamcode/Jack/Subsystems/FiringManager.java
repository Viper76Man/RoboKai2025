package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class FiringManager implements Subsystem {
    public BallManager manager;
    public FlickerSubsystem.FlickUp  command, command2, command3;
    public FlickerSubsystem flicker;
    public SpindexerMotorV1 spindexer;

    public void init(BallManager manager, FlickerSubsystem flicker, SpindexerMotorV1 spindexer){
        this.flicker = flicker;
        this.command = flicker.fire();
        this.manager = manager;
        this.spindexer = spindexer;
        this.command2 = flicker.fire();
        this.command3 = flicker.fire();
    }

    public FireTriple fireTriple(Robot.Mode mode, ArcMotorsV2 arcMotorsV2){
        return new FireTriple(true, mode, arcMotorsV2);
    }

    public FireTriple fireSingle(ArcMotorsV2 arc){
        return new FireTriple(false, Robot.Mode.TELEOP, arc);
    }
    public enum States {
        START,
        FLICKER_UP,
        SPIN,
        DOWN
    }

    public class FireTriple extends Command {
        private boolean firing = false;
        public boolean done = false;
        public States state = States.START;
        private Command activeFire;
        public boolean triple;
        public Robot.Mode mode;
        public ArcMotorsV2 arc;
        public ElapsedTime fireTimer = new ElapsedTime();
        public boolean spindexerSet = false;

        public FireTriple(boolean triple, Robot.Mode mode, ArcMotorsV2 arcMotorsV2){
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
            if(triple) {
                if (!firing && spindexer.isSpindexerReady() && !spindexerSet && state == States.START) {
                    if(mode == Robot.Mode.AUTONOMOUS && arc.arcShooter.isInRange(75)) {
                        startFiring();
                    }
                    else if(mode != Robot.Mode.AUTONOMOUS){
                        startFiring();
                    }
                }
                if(state == States.FLICKER_UP){
                    //activeFire.schedule();
                    flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_UP);
                    fireTimer.reset();
                    state = States.SPIN;
                }
                if(state == States.SPIN) {
                    if(fireTimer.seconds() > 0.1 && fireTimer.seconds() < 1.3) {
                        spindexer.setState(SpindexerMotorV1.State.SHOOT_ALL_RAMP);
                        activeFire.cancel();
                        manager.setCurrentBall(4);
                        manager.setMode(BallManager.State.SHOOT);
                    }
                    if(fireTimer.seconds() > 1.3 && fireTimer.seconds() < 2.5){
                        activeFire.cancel();
                        flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
                    }
                    if(fireTimer.seconds() > 2.5){
                        state = States.DOWN;
                    }
                }
                if(state == States.DOWN) {
                    spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
                    flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
                    manager.setCurrentBall(1);
                    manager.setMode(BallManager.State.INTAKE);
                    done = true;
                }
            }
        }

        @Override
        public boolean isDone() {
            return state == States.DOWN && manager.mode == BallManager.State.INTAKE;
        }

        @Override
        public void stop(boolean interrupted){
            flicker.flicker.setPositionNew(RobotConstantsV1.FLICKER_SERVO_DOWN);
            state = States.START;
            firing = false;
            done = false;
            spindexerSet = false;
        }

        public void startFiring(){
            manager.setMode(BallManager.State.SHOOT);
            manager.setCurrentBall(1);
            spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
            state = States.FLICKER_UP;
            firing = true;
        }
    }
}
