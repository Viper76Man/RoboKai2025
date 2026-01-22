package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
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
    public SystemStates state = SystemStates.START;
    public IntakeSubsystemV1 intake = new IntakeSubsystemV1();;
    public ShooterSubsystemV1 shooter;
    public BallManager manager = new BallManager();


    public ElapsedTime stateTimer = new ElapsedTime();



    public boolean firedAlready = false;


    public void init(HardwareMap hardwareMap){
        intake.init(hardwareMap);
        shooter = new ShooterSubsystemV1();
    }


    public void systemStatesUpdate(){
        switch (state){
            case INTAKE_BALL_1:
                if(!intake.ballDetected && !intake.searching){
                    intake.ballUpdate();
                }
                else if(intake.ballDetected && !intake.searching){
                    manager.next();
                    setSystemState(SystemStates.INTAKE_BALL_2);
                }
                break;
            case SHOOT_ALL:
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

    public void setSystemState(SystemStates state){
        this.state = state;
        stateTimer.reset();
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
