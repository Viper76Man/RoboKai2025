package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.CustomCommand;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeSubsystemV1;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ShooterSubsystemV1;

import dev.nextftc.core.commands.CommandManager;

public class RobotV4 {
    public enum SystemStates {
        START,
        INTAKE_BALL_1,
        INTAKE_BALL_2,
        INTAKE_BALL_3,
        SHOOT_ALL
    }

    public enum Mode {
        SHOOT,
        INTAKE
    }

    public SystemStates state = SystemStates.START;
    public IntakeSubsystemV1 intake = new IntakeSubsystemV1();;
    public Mode mode = Mode.INTAKE;
    public ShooterSubsystemV1 shooter = new ShooterSubsystemV1();
    public BallManager manager = new BallManager();


    public ElapsedTime stateTimer = new ElapsedTime();


    public CustomCommand spindexerCommand;
    public CustomCommand intakeUpdateCommand;
    public double spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE;

    public boolean firedAlready = false;


    public void init(HardwareMap hardwareMap){
        intake.init(hardwareMap);
        shooter.init(hardwareMap, Robot.Mode.TELEOP, intake, manager);
        spindexerCommand = intake.spindexerRun(spindexerTarget, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }


    public void systemStatesUpdate(){
        spindexerBallUpdate();
        if(spindexerCommand.isDone() && !intake.spindexer.isSpindexerReady()){
            refreshSpindexerCommand();
            spindexerCommand.schedule();
        }
        switch (state){
            case INTAKE_BALL_1:
            case INTAKE_BALL_2:
            case INTAKE_BALL_3:
                mode = Mode.INTAKE;
                if(!intake.ballDetected && !intake.searching){
                    intake.setIntakePower(RobotConstantsV1.INTAKE_POWER, RobotConstantsV1.intakeDirection);
                    intakeUpdateCommand = intake.ballUpdate();
                    intakeUpdateCommand.schedule();
                }
                else if(intake.ballDetected && !intake.searching){
                    manager.next();
                    setSystemState(SystemStates.INTAKE_BALL_2);
                }
                break;
            case SHOOT_ALL:
                mode = Mode.SHOOT;
                if(!shooter.busy && !firedAlready){
                    shooter.shootTriple();
                    firedAlready = true;
                }
                else if(!shooter.busy){
                    setSystemState(SystemStates.INTAKE_BALL_1);
                    manager.setCurrentBall(1);
                }
                break;

        }
    }

    public void spindexerBallUpdate(){
        switch (mode){
            case INTAKE:
                switch (manager.getCurrentBall()){
                    case 1:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_INTAKE;
                        break;
                    case 2:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_INTAKE;
                        break;
                    case 3:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_INTAKE;
                        break;
                }
                break;
            case SHOOT:
                switch (manager.getCurrentBall()){
                    case 1:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_1_SHOOT;
                        break;
                    case 2:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_2_SHOOT;
                        break;
                    case 3:
                        spindexerTarget = RobotConstantsV1.SPINDEXER_MOTOR_BALL_3_SHOOT;
                        break;
                }
        }
    }

    public void setSystemState(SystemStates state){
        this.state = state;
        stateTimer.reset();
    }

    public void refreshSpindexerCommand(){
        spindexerCommand = intake.spindexerRun(spindexerTarget, SpindexerMotorV1.EncoderMeasurementMethod.MOTOR);
    }

    public void log(Telemetry telemetry){
        if(RobotConstantsV1.panelsEnabled) {
            TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
            for (String command : CommandManager.INSTANCE.snapshot()) {
                telemetryM.addLine(command);
            }
            telemetryM.update(telemetry);
        }
        else {
            for (String command : CommandManager.INSTANCE.snapshot()) {
                telemetry.addLine(command);
            }
        }
    }
}
