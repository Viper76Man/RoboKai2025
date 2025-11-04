package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.PinpointV1;

public class Robot {

    public PinpointV1 pinpoint = new PinpointV1();
    public ArcShooterV1 shooter = new ArcShooterV1();
    public IntakeV1 intake = new IntakeV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public Mode mode;
    public Gamepad gamepad1;

    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    public void init(Mode mode, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.mode = mode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        //pinpoint.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        drive.init(hardwareMap, gamepad1);
        //if(mode == Mode.AUTONOMOUS){
            //pinpoint.resetPosAndIMU();
        //}

    }

    public void systemStatesUpdate(){
        shooter.setTargetVelocity(RobotConstantsV1.SHOOTER_TARGET_VELOCITY);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
        drive.drive();
    }
}
