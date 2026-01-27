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
    public ParallelGroup command;
    public SpindexerMotorV1 spindexer;

    public void init(BallManager manager, ParallelGroup flickUpCommand, SpindexerMotorV1 spindexer){
        this.command = flickUpCommand;
        this.manager = manager;
        this.spindexer = spindexer;
    }

    public FireTriple fire(){
        return new FireTriple();
    }


    public class FireTriple extends Command {
        public boolean flickedAlready = false;

        @Override
        public void start(){
            manager.setMode(BallManager.State.SHOOT);
        }

        @Override
        public void update(){
            switch (manager.mode){
                case SHOOT:
                    switch (manager.getCurrentBall()){
                        case 1:
                            if(command.isDone() && flickedAlready){
                                manager.setCurrentBall(2);
                            }
                            else if(!flickedAlready && spindexer.isSpindexerReady()){
                                command.schedule();
                                flickedAlready = true;
                            }
                            break;
                        case 2:
                            if(command.isDone() && flickedAlready){
                                manager.setCurrentBall(3);
                            }
                            else if(!flickedAlready && spindexer.isSpindexerReady()){
                                command.schedule();
                                flickedAlready = true;
                            }
                            break;
                        case 3:
                            if(command.isDone() && flickedAlready){
                                manager.setCurrentBall(1);
                                manager.setMode(BallManager.State.INTAKE);
                            }
                            else if(!flickedAlready && spindexer.isSpindexerReady()){
                                command.schedule();
                                flickedAlready = true;
                            }
                            break;
                    }
                    break;
            }
        }

        @Override
        public boolean isDone() {
            return (ActiveOpMode.isStopRequested()) || (manager.isEmpty(1) && manager.isEmpty(2) && manager.isEmpty(3));
        }
    }
}
