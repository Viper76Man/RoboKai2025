package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.PinpointV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

public class Robot {

    public PinpointV1 pinpoint = new PinpointV1();
    public ArcShooterV1 shooter = new ArcShooterV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Mode mode;
    public GamepadV1 gamepad = new GamepadV1();
    public IntakeV1 intake = new IntakeV1();
    public StorageServoV1 storage = new StorageServoV1();

    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    public void init(Mode mode, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.mode = mode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        //TODO: Debug gamepad
        this.gamepad.init(gamepad1, 0.3);
        //pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);
        drive.init(hardwareMap, gamepad1);
        intake.init(hardwareMap);
        storage.init(hardwareMap);
        //if(mode == Mode.AUTONOMOUS){
            //pinpoint.resetPosAndIMU();
        //}

    }

    public void systemStatesUpdate(){
        gamepad.update();
        shooter.setTargetVelocity(RobotConstantsV1.SHOOTER_TARGET_RPM);
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
        drive.drive();
        if(gamepad.circle && gamepad.isGamepadReady()){
            storage.runToIntakeBall(storage.getNextBall(storage.getIntakeBall()));
            gamepad.resetTimer();
        }
        if(gamepad.triangle && gamepad.isGamepadReady()){
            intake.switchDirection();
            gamepad.resetTimer();
        }
        telemetry.addLine("Ball: " + storage.getIntakeBall());
        telemetry.addLine("Next ball: " + storage.getNextBall(storage.getIntakeBall()));
    }
}
