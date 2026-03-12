package org.firstinspires.ftc.teamcode.Coach;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Spindexer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC Spindexer Test", group = "Coach")
public class SpindexerTest extends NextFTCOpMode {
    private Limelight3A limelight3A;
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public SpindexerTest() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    // change the names and directions to suit your robot
    private final MotorEx frontLeftMotor = new MotorEx("fl").reversed();
    private final MotorEx frontRightMotor = new MotorEx("fr");
    private final MotorEx backLeftMotor = new MotorEx("bl").reversed();
    private final MotorEx backRightMotor = new MotorEx("br");

    @Override
    public void onInit(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(2); //Red Apriltag
        limelight3A.start();

    }

    public void onStartButtonPressed() {
        telemetry.addLine("Running");

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(Spindexer.INSTANCE.toSecondPOS);

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(Spindexer.INSTANCE.toFirstPos);

        }

    public void onUpdate(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            telemetry.addData("Target X", llResult.getTx());
            telemetry.update();
    }

    }

}
