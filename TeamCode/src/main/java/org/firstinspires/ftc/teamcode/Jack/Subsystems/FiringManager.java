package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class FiringManager implements Subsystem {
    public BallManager manager;
    public ParallelGroup command, command2, command3;
    public FlickerSubsystem flicker;
    public SpindexerMotorV1 spindexer;

    public void init(BallManager manager, FlickerSubsystem flicker, SpindexerMotorV1 spindexer){
        this.flicker = flicker;
        this.command = new ParallelGroup(flicker.fire());
        this.manager = manager;
        this.spindexer = spindexer;
        this.command2 = new ParallelGroup(flicker.fire());
        this.command3 = new ParallelGroup(flicker.fire());
    }

    public FireTriple fireTriple(){
        return new FireTriple(true);
    }

    public FireTriple fireSingle(){
        return new FireTriple(false);
    }

    public class FireTriple extends Command {
        private boolean firing = false;
        private Command activeFire;
        public boolean triple;

        public FireTriple(boolean triple){
            this.triple = triple;
        }

        @Override
        public void start() {
            firing = false;
        }

        @Override
        public void update() {
            if(triple) {
                if (!firing && spindexer.isSpindexerReady()) {
                    startFiring();
                }
                if (firing && activeFire.isDone()) {
                    nextBall();
                }
            }
            else {
                if (!firing && spindexer.isSpindexerReady() && ActiveOpMode.gamepad1().right_trigger >= 0.15) {
                    startFiring();
                }
                if (firing && activeFire.isDone()) {
                    nextBall();
                }
            }
        }

        @Override
        public boolean isDone() {
            return manager.mode == BallManager.State.INTAKE;
        }

        public void startFiring(){
            activeFire = flicker.fire();
            activeFire.schedule();
            firing = true;
        }

        public void nextBall(){
            firing = false;
            if (manager.getCurrentBall() == 3) {
                manager.setCurrentBall(1);
                manager.setMode(BallManager.State.INTAKE);
            } else {
                manager.setCurrentBall(manager.getCurrentBall() + 1);
            }
        }
    }
}
