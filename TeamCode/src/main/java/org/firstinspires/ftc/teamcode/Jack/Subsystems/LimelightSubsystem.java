package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightSubsystem implements Subsystem {
    public TurretServoCR turret = new TurretServoCR();
    public LimelightV1 limelight = new LimelightV1();
    public goBackToZero goBackToZero;

    public LimelightSubsystem(){
        this.goBackToZero = new goBackToZero();
    }

    @Override
    public void initialize() {
        limelight.init(ActiveOpMode.hardwareMap());
        turret.init(ActiveOpMode.hardwareMap());
    }

    public class goBackToZero extends Command {
        public boolean done;

        @Override
        public void start(){

        }

        @Override
        public void update(){
            double error = 236 - turret.getEncoderPos();
            turret.setPower(error * RobotConstantsV1.turretServoPower);
            done = Math.abs(error) < 2;
            if(limelight.getLatestAprilTagResult() != null){
                done = true;
            }
        }

        @Override
        public boolean isDone() {
            return done;
        }
    }
}
