package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.modernrobotics.comm.ModernRoboticsDatagram;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.sun.tools.javac.code.Attribute;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.DecodeFieldLocalizer;
import org.firstinspires.ftc.teamcode.Jack.Odometry.PinpointV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.StorageServoV1;

public class Robot {

    public PinpointV1 pinpoint = new PinpointV1();
    public ArcShooterV1 shooter = new ArcShooterV1();
    public MecanumDriveOnly drive = new MecanumDriveOnly();

    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public DecodeFieldLocalizer localizer = new DecodeFieldLocalizer();
    public Mode mode;
    public GamepadV1 gamepad = new GamepadV1();
    public IntakeV1 intake = new IntakeV1();
    public StorageServoV1 storage = new StorageServoV1();
    public Alliance alliance = Alliance.TEST;
    public Follower follower;

    public enum Mode {
        TELEOP,
        AUTONOMOUS
    }

    public enum Alliance {
        BLUE,
        RED,
        TEST
    }

    public void init(Mode mode, Alliance alliance, HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1){
        this.mode = mode;
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.alliance = alliance;
        //TODO: Debug gamepad
        this.gamepad.init(gamepad1, 0.3);
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        drive.init(hardwareMap, gamepad1);
        intake.init(hardwareMap);
    }

    public void systemStatesUpdate(){
        follower.update();
        arcUpdate();
        runIntake();
        if(mode == Mode.TELEOP) {
            gamepad.update();
            drive();
            if (gamepad.triangle && gamepad.isGamepadReady()) {
                switchIntakeDirection();
                gamepad.resetTimer();
            }
        }
    }

    public void shootArtifact(int artifact){
        runShooterActive();
    }

    public void switchIntakeDirection(){
        intake.switchDirection();
    }
    public void runIntake(){
        intake.setDirection(RobotConstantsV1.intakeDirection);
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
    }

    public void runIntakeReverse(){
        if(RobotConstantsV1.intakeDirection == DcMotorSimple.Direction.REVERSE){
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
    }

    public void runShooterIdle(){
        shooter.setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
    }

    public void runShooterActive(){
        shooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
    }

    public void arcUpdate(){
        if(Math.abs(getDistanceFromLaunchZone()) < RobotConstantsV1.maxLaunchZoneDistance + 12){
            runShooterActive();
        }
        else {
            runShooterIdle();
        }
    }

    public void drive(){
        if(inLaunchZone()){
            drive.driveWithRotationLock(alliance, pinpoint.getPose(), telemetry, true);
        }
        else {
            drive.drive();
        }
    }

    public double getDistanceFromLaunchZone(){
        return localizer.getDistanceFromLaunchZone(pinpoint.getPose());
    }

    public boolean inLaunchZone(){
        return localizer.isRobotInBackLaunchZone(pinpoint.getPose());
    }
}
