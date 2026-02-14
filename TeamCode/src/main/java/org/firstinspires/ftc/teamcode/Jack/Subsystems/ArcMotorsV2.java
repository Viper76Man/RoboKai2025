package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.GamepadV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotV4;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ArcMotorsV2 implements Subsystem {
    public HardwareMap hardwareMap;
    public Robot.Mode mode;
    public enum Zone {
        FRONT,
        BACK
    }
    public Zone zone = Zone.BACK;

    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public LimelightV1 limelight;
    public double offset = 0;
    public void init(HardwareMap hardwareMap, Robot.Mode mode, LimelightV1 limelight){
        this.hardwareMap = hardwareMap;
        this.mode = mode;
        this.limelight = limelight;
        switch (mode) {
            case TELEOP:
                arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
                break;
            case AUTONOMOUS:
                arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDsAuto);
                break;
        }
    }

    public run spinUpIdle(){
        setTargetRPM(RobotConstantsV1.SHOOTER_IDLE_RPM);
        return new run();
    }

    public void setZone(Zone zone){
        this.zone = zone;
    }

    public run spinActive(){
        switch (zone){
            case FRONT:
                return spinUpFront();
            case BACK:
                return spinUpBack();
            default:
                return spinUpIdle();
        }
    }

    public run spinUpFront(){
        setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
        return new run();
    }

    public run spinUpBack(){
        switch (mode) {
            case AUTONOMOUS:
                setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO);
                return new run();
            case TELEOP:
            default:
                setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                return new run();
        }
    }

    public void setTargetRPM(double target){
        arcShooter.setTargetRPM(target);
    }

    public class run extends Command{
        public ElapsedTime buttonTimer = new ElapsedTime();
        public Gamepad gamepad = ActiveOpMode.gamepad1();

        @Override
        public void update(){
            arcShooter.run();
            double dist = limelight.getTargetDistance();
            if(dist != 0) {
                setTargetRPM(flywheelSpeed(dist) + offset);
            }
            //if(gamepad.right_bumper && buttonTimer.seconds() > 0.3){
                //setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
                //buttonTimer.reset();
            //}
            //if(gamepad.left_bumper && buttonTimer.seconds() > 0.3){
                //setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
               // buttonTimer.reset();
            //}
            if(gamepad.dpad_up && buttonTimer.seconds() > 0.3){
                offset += 10;
                buttonTimer.reset();
            }
            if(buttonTimer.seconds() > 0.3 && gamepad.dpad_down){
                offset -= 10;
                buttonTimer.reset();
            }
        }

        @Override
        public boolean isDone() {
            return ActiveOpMode.isStopRequested();
        }
    }

    public double flywheelSpeed(double dist){
        return MathFunctions.clamp(((-0.0000295523 * Math.pow(dist, 4)) + (0.00588085 * Math.pow(dist, 3)) - (0.281589 * Math.pow(dist, 2)) + (9.28581 * dist) + 2056.97581), 0, 4000);
    }
}
