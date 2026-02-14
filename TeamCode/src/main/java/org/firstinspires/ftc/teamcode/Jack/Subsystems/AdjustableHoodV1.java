package org.firstinspires.ftc.teamcode.Jack.Subsystems;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.AdjustableHoodServo;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class AdjustableHoodV1 implements Subsystem {
    public AdjustableHoodServo hoodServo = new AdjustableHoodServo();
    public LimelightV1 limelight;
    public double offset = 0;
    public void init(LimelightV1 limelight){
        this.limelight = limelight;
        hoodServo.init(ActiveOpMode.hardwareMap());
    }

    public class setPosition extends Command{
        public setPosition(double pos){
            hoodServo.setPos(pos);
        }


        @Override
        public boolean isDone() {
            return false;
        }
    }

    public updateServo servoUpdate(){
        return new updateServo();
    }

    public class updateServo extends Command {

        public Gamepad gamepad = ActiveOpMode.gamepad1();
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        public ElapsedTime buttonTimer = new ElapsedTime();
        @Override
        public void update(){
            telemetryM.addLine("Target Hood Angle: "+ (hoodAngle(limelight.getTargetDistance()) + offset));
            if(limelight.getTargetDistance() != 0) {
                hoodServo.setPos(hoodAngle(limelight.getTargetDistance()) + offset);
            }
            if(buttonTimer.seconds() > 0.3 && gamepad.dpad_left){
                offset += 0.05;
                buttonTimer.reset();
            }
            if(buttonTimer.seconds() > 0.3 && gamepad.dpad_right){
                offset -= 0.05;
                buttonTimer.reset();
            }
        }

        @Override
        public boolean isDone() {
            return false;
        }
    }

    public double hoodAngle(double dist){
        return MathFunctions.clamp((((0.00000322524 * Math.pow(dist, 3)) - (0.000664089 * Math.pow(dist, 2)) + (0.0447188 * dist) - 0.887317)), 0, 0.2);
    }

}
