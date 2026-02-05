package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
    public void init(HardwareMap hardwareMap, Robot.Mode mode){
        this.hardwareMap = hardwareMap;
        this.mode = mode;
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

        @Override
        public void update(){
            arcShooter.run();
        }

        @Override
        public boolean isDone() {
            return ActiveOpMode.isStopRequested();
        }
    }
}
