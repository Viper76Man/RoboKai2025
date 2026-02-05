package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightSubsystem implements Subsystem {
    public TurretServoCR turret = new TurretServoCR();
    public LimelightV1 limelight = new LimelightV1();

    public LimelightSubsystem(){
    }

    public void init() {
        limelight.init(ActiveOpMode.hardwareMap());
        turret.init(ActiveOpMode.hardwareMap());
    }

    public void setPipeline(LimelightV1.Pipeline pipeline){
        limelight.setPipeline(pipeline);
    }

    public class updateTurret extends Command {
        public double angle;
        public updateTurret(double offset){
            this.angle = offset;
        }
        public boolean done;

        @Override
        public void start(){

        }

        @Override
        public void update(){
            turret.run(limelight, angle);
        }

        @Override
        public boolean isDone() {
            return done;
        }
    }

    public void log(TelemetryManager telemetryM){
        LLResultTypes.FiducialResult result = limelight.getLatestAprilTagResult();
        if(result != null) {
            telemetryM.addLine("Target Y: " + result.getTargetYDegrees());
            telemetryM.addLine("Distance: " + limelight.getTargetDistance());
            telemetryM.addLine("PIDs: (" + turret.getPIDCoefficients().kP + ", " + turret.getPIDCoefficients().kI + ", " + turret.getPIDCoefficients().kD + ")");
        }
        else {
            telemetryM.addLine("Target Y: Invalid Target");
            telemetryM.addLine("Distance: Invalid Target");
            telemetryM.addLine("PIDs: (" + turret.getPIDCoefficients().kP + ", " + turret.getPIDCoefficients().kI + ", " + turret.getPIDCoefficients().kD + ")");
        }
    }

    public void log(Telemetry telemetry){
        LLResultTypes.FiducialResult result = limelight.getLatestAprilTagResult();
        if(result != null) {
            telemetry.addLine("Target Y: " + result.getTargetYDegrees());
            telemetry.addLine("Distance: " + limelight.getTargetDistance());
            telemetry.addLine("PIDs: (" + turret.getPIDCoefficients().kP + ", " + turret.getPIDCoefficients().kI + ", " + turret.getPIDCoefficients().kD + ")");
        }
        else {
            telemetry.addLine("Target Y: Invalid Target");
            telemetry.addLine("Distance: Invalid Target");
            telemetry.addLine("PIDs: (" + turret.getPIDCoefficients().kP + ", " + turret.getPIDCoefficients().kI + ", " + turret.getPIDCoefficients().kD + ")");
        }
    }
}
