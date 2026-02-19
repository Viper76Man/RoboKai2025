package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
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

    public class FireTriple extends Command {
        private boolean firing = false;
        private Command activeFire;
        public boolean triple;
        public Robot.Mode mode;
        public ArcMotorsV2 arc;

        public FireTriple(boolean triple, Robot.Mode mode, ArcMotorsV2 arcMotorsV2){
            this.triple = triple;
            this.mode = mode;
            this.arc = arcMotorsV2;
        }

        @Override
        public void start() {
            firing = false;
        }

        @Override
        public void update() {
            if(triple) {
                if (!firing && spindexer.isSpindexerReady()) {
                    if(mode == Robot.Mode.AUTONOMOUS && arc.arcShooter.isInRange(75)) {
                        startFiring();
                    }
                    else if(mode != Robot.Mode.AUTONOMOUS){
                        startFiring();
                    }
                }
                if (firing && activeFire.isDone()) {
                    nextBall();
                    firing = false;
                }
            }
            else {
                if (!firing && spindexer.isSpindexerReady() && ActiveOpMode.gamepad1().right_trigger >= 0.15) {
                    startFiring();
                }
                if (firing && activeFire.isDone()) {
                    nextBall();
                    firing = false;
                }
            }
        }

        @Override
        public boolean isDone() {
            return manager.mode == BallManager.State.INTAKE;
        }

        public void startFiring(){
            if(mode == Robot.Mode.TELEOP) {
                if ((!manager.isEmpty(manager.getCurrentBall())) && manager.getCurrentBall() < 4) {
                    activeFire = flicker.fire();
                    activeFire.schedule();
                    firing = true;
                }
            }
            else {
                if (manager.getCurrentBall() < 4) {
                    activeFire = flicker.fire();
                    activeFire.schedule();
                    firing = true;
                }
            }
            if(manager.getCurrentBall() >= 4){
                nextBall();
            }
        }

        public void nextBall(){
            if (manager.getCurrentBall() >= 4) {
                if(mode == Robot.Mode.AUTONOMOUS) {
                    manager.setEmpty(1);
                    manager.setEmpty(2);
                    manager.setEmpty(3);
                }
                manager.setCurrentBall(1);
                manager.setMode(BallManager.State.INTAKE);
            } else {
                manager.setEmpty(manager.getCurrentBall());
                switch (manager.getCurrentBall()){
                    case 1:
                        manager.setCurrentBall(2);
                        break;
                    case 2:
                        manager.setCurrentBall(3);
                        break;
                    case 3:
                        manager.setCurrentBall(1);
                        manager.setMode(BallManager.State.INTAKE);
                        break;
                    default:
                        manager.setCurrentBall(1);
                        break;
                }
            }
        }
    }
}
