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
        return new run(RobotConstantsV1.SHOOTER_IDLE_RPM);
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
        return new run(RobotConstantsV1.SHOOTER_FRONT_RPM);
    }

    public run spinUpBack(){
        switch (mode) {
            case AUTONOMOUS:
                return new run(RobotConstantsV1.SHOOTER_TARGET_RPM_AUTO);
            case TELEOP:
            default:
                return new run(RobotConstantsV1.SHOOTER_TARGET_RPM);
        }
    }

    public void setTargetRPM(double target){
        arcShooter.setTargetRPM(target);
    }

    public class run extends Command{
        public double target;

        public run(double targetRPM){
            this.target = targetRPM;
        }

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
