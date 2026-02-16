package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV4;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.Sensors;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class SpindexerV2 implements Subsystem {
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public BallManager manager;
    public void init(BallManager manager, Sensors sensors){
        this.manager = manager;
        spindexer.init(ActiveOpMode.hardwareMap(), RobotConstantsV1.spindexerPIDs, sensors);
    }
    public void init(BallManager manager){
        this.manager = manager;
        spindexer.init(ActiveOpMode.hardwareMap(), RobotConstantsV1.spindexerPIDs);
    }

    public run spindexerRun(){
        return new run(manager);
    }

    public class run extends Command {
        public double position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE;
        public BallManager manager_;
        public run(BallManager manager){
            manager_ = manager;
        }

        @Override
        public void update(){
            switch (manager_.mode) {
                case SHOOT:
                    switch (manager_.getCurrentBall()) {
                        case 1:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_SHOOT;
                            break;
                        case 2:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_SHOOT;
                            break;
                        case 3:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_SHOOT;
                            break;
                    }
                    break;
                case INTAKE:
                    switch (manager_.getCurrentBall()) {
                        case 1:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE;
                            break;
                        case 2:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_INTAKE;
                            break;
                        case 3:
                            position = RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_INTAKE;
                            break;
                    }
                    break;
            }
            spindexer.setTargetPos(position, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
            spindexer.run();
        }

        @Override
        public boolean isDone() {
            return ActiveOpMode.isStopRequested();
        }
    }
}
