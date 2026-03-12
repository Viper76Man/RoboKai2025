package org.firstinspires.ftc.teamcode.Coach;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Spindexer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "NextFTC Spindexer Shoot Test", group = "Coach")
public class SpindexerShootTest extends NextFTCOpMode {
    private Limelight3A limelight3A;
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public SpindexerShootTest() {
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE, Ramp.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );

    }

    enum SpindexerState{
        FIRSTPOSITION,
        SECONDPOSITION,
        THIRDPOSITION
    }

    SpindexerState currentSpindexerState = SpindexerState.FIRSTPOSITION;

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

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(Spindexer.INSTANCE.toFirstPos);

        Gamepads.gamepad1().dpadLeft()
                .whenBecomesTrue(Spindexer.INSTANCE.toSecondPOS);

        Gamepads.gamepad1().dpadRight()
                .whenBecomesTrue(Spindexer.INSTANCE.toThirdPos);

        Gamepads.gamepad1().rightTrigger().greaterThan(.2)
                .whenBecomesTrue(Fire());

        }

    public void onUpdate(){
        LLResult llResult = limelight3A.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botpose = llResult.getBotpose_MT2();
            telemetry.addData("Target X", llResult.getTx());
        }
        telemetry.update();

    }

    public Command Fire(){
        switch (currentSpindexerState) {
            case FIRSTPOSITION:
                return new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        Spindexer.INSTANCE.toFourthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        Spindexer.INSTANCE.toFirstPos
                );

            case SECONDPOSITION:
                return new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        Spindexer.INSTANCE.toFifthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        Spindexer.INSTANCE.toFirstPos
                );
            case THIRDPOSITION:
                return new SequentialGroup(
                        Ramp.INSTANCE.rampUp,
                        new Delay(.4),
                        Spindexer.INSTANCE.toSixthPos,
                        Ramp.INSTANCE.rampDown,
                        new Delay(.4),
                        Spindexer.INSTANCE.toFirstPos
                );
        }
        return null;
    }



}
