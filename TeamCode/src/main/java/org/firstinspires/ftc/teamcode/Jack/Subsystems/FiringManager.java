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

    public class FireTriple extends Command {
        private boolean firing = false;
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
                        activeFire = flicker.fire();
                        activeFire.schedule();
                    }
                    else if(mode != Robot.Mode.AUTONOMOUS){
                        startFiring();
                        activeFire = flicker.fire();
                        activeFire.schedule();
                    }
                }
                if (firing && activeFire.isDone() && spindexer.isSpindexerReady() && spindexerSet) {
                    //nextBall();
                    firing = false;
                    manager.setCurrentBall(1);
                    manager.setMode(BallManager.State.INTAKE);
                    flicker.flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
                    spindexerSet = false;
                }
                if(firing && activeFire.isDone()) {
                    if(fireTimer.seconds() > 0.15) {
                        spindexer.setState(SpindexerMotorV1.State.SHOOT_ALL_RAMP);
                        spindexerSet = true;
                    }
                }
                else {
                    fireTimer.reset();
                }
            }
        }

        @Override
        public boolean isDone() {
            return manager.mode == BallManager.State.INTAKE;
        }

        public void startFiring(){
            firing = true;
        }
    }
}
