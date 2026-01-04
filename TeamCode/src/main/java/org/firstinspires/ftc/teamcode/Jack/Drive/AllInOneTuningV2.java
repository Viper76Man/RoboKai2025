package org.firstinspires.ftc.teamcode.Jack.Drive;

import static org.firstinspires.ftc.teamcode.Jack.Odometry.Tuning.drawCurrent;
import static org.firstinspires.ftc.teamcode.Jack.Odometry.Tuning.drawCurrentAndHistory;
import static org.firstinspires.ftc.teamcode.Jack.Odometry.Tuning.follower;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.ArcShooterV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.IntakeV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.PIDController;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV1;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Constants;
import org.firstinspires.ftc.teamcode.Jack.Odometry.Tuning;
import org.firstinspires.ftc.teamcode.Jack.Other.ArtifactColor;
import org.firstinspires.ftc.teamcode.Jack.Other.LoggerV1;
import org.firstinspires.ftc.teamcode.Jack.Other.MultipleTelemetry;
import org.firstinspires.ftc.teamcode.Jack.Other.RGB;
import org.firstinspires.ftc.teamcode.Jack.Other.Range;
import org.firstinspires.ftc.teamcode.Jack.Other.SlotColorSensorV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.FlickerServoV1;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoCR;
import org.firstinspires.ftc.teamcode.Jack.Servos.TurretServoV1;
import org.firstinspires.ftc.teamcode.R;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@TeleOp
public class AllInOneTuningV2 extends SelectableOpMode {
    public TelemetryManager telemetryM;

    public AllInOneTuningV2() {
        super("Select a tuning OpMode: ", s -> {
            s.folder("Drive", d -> {
                d.add("Mecanum Drive", MecanumDrive::new);
                d.add("Rotation Lock Drive", RotationLockDrive::new);
            });
            s.folder("Hardware", h -> {
                h.add("Pre-TeleOp Test", PreTeleOpTest::new);
                h.folder("Intake", i -> {
                    i.add("Automated Intake Test", AutomatedIntakeTest::new);
                    i.add("Intake Subsystem Test", IntakeSubsystemTest::new);
                    i.add("Intake Test", IntakeTest::new);
                });
                h.folder("Spindexer", sp -> {
                    sp.add("SPINDEXER ENCODER RESET", SpindexerResetEncoder::new);
                    sp.add("Spindexer Encoder Tuner", SpindexerReadEncoderTest::new);
                    sp.add("Spindexer Test", SpindexerTest::new);
                });
                h.folder("Turret", t-> {
                    t.add("Turret PID Test", TurretServoTester::new);
                    t.add("Turret Speed Tester", TurretSpeedTest::new);
                });
                h.folder("Shooter", sh -> {
                    sh.add("Shooter Subsystems test", DeliverySubsystemsTest::new);
                    sh.add("Flicker Test", FlickerTest::new);
                    sh.add("Shooter PID Test", ShooterPIDTest::new);
                });
                h.folder("Camera", c -> {
                    c.add("Limelight V1 Test", LimelightV1Test::new);
                });
                h.folder("Color Sensor", cs->{
                    cs.add("Color Sensor Test", ColorSensorTest::new);
                });
            });
            s.folder("Localization", l -> {
                l.add("Localization Test", LocalizationTest::new);
                l.add("Central Localization Test", CentralLocalizationTest::new);
            });
            s.folder("Logging", logs->{
                logs.add("Log Reader", LogReader::new);
            });

        });
    }


    @Override
    public void onSelect() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        List<String> list = new ArrayList<>();
        list.add(" ");
        telemetryM.setLines(list);
        telemetryM.update(telemetry);
    }

}
    class MecanumDrive extends OpMode {
        public MecanumDriveOnly drive = new MecanumDriveOnly();
        public GamepadV1 gamepad = new GamepadV1();

        @Override
        public void init() {
            gamepad.init(gamepad1, 0.3);
            drive.init(hardwareMap, gamepad);
        }

        @Override
        public void loop() {
            gamepad.update();
            drive.drive();
        }
    }

    class RotationLockDrive extends OpMode {
        public MecanumDriveOnly drive = new MecanumDriveOnly();
        public GamepadV1 gamepad = new GamepadV1();
        public Follower follower = Constants.createFollower(hardwareMap);
        public Robot.Alliance team = Robot.Alliance.BLUE;
        public boolean cam = true;

        @Override
        public void init() {
            gamepad.init(gamepad1, 0.3);
            drive.init(hardwareMap, gamepad);
            telemetry.addLine("This OpMode will test the drive rotation lock to tune PIDs.");
            telemetry.addLine("\tPress Circle to switch teams/alliances.");
            telemetry.addLine("\tPress Square to toggle camera.");
        }

        @Override
        public void loop() {
            gamepad.update();
            if (gamepad.circle && gamepad.isGamepadReady()) {
                switch (team) {
                    case BLUE:
                        team = Robot.Alliance.RED;
                        break;
                    case RED:
                        team = Robot.Alliance.TEST;
                        break;
                    case TEST:
                        team = Robot.Alliance.BLUE;
                        break;
                }
                gamepad.resetTimer();
            }
            if (gamepad.square && gamepad.isGamepadReady()) {
                cam = !cam;
                gamepad.resetTimer();
            }
            drive.driveWithRotationLock(team, follower.getPose(), telemetry, cam);
        }
    }

    class CentralLocalizationTest extends OpMode {
        public MecanumDriveOnly drive = new MecanumDriveOnly();
        public GamepadV1 gamepad = new GamepadV1();
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        @Override
        public void init() {
            drive.init(hardwareMap, gamepad);
            follower.setPose(new Pose(72, 72, Math.toRadians(90)));
            gamepad.init(gamepad1, 0.3);
        }

        /**
         * This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.
         */
        @Override
        public void init_loop() {
            telemetryM.debug("This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1.");
            gamepad.update();
            telemetryM.update(telemetry);
            follower.update();
            drawCurrent();
        }

        @Override
        public void start() {
            follower.startTeleopDrive();
            follower.update();
        }

        /**
         * This updates the robot's pose estimate, the simple mecanum drive, and updates the
         * Panels telemetry with the robot's position as well as draws the robot's position.
         */
        @Override
        public void loop() {
            follower.update();
            gamepad.update();
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
            telemetryM.debug("x:" + follower.getPose().getX());
            telemetryM.debug("y:" + follower.getPose().getY());
            telemetryM.debug("heading:" + follower.getPose().getHeading());
            telemetryM.debug("heading (deg):" + Math.toDegrees(follower.getPose().getHeading()));
            telemetryM.debug("total heading:" + follower.getTotalHeading());
            telemetryM.update(telemetry);

            drawCurrentAndHistory();
        }
    }

    class LocalizationTest extends OpMode {
        public MecanumDriveOnly drive = new MecanumDriveOnly();
        public GamepadV1 gamepad = new GamepadV1();
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        @Override
        public void init() {
            drive.init(hardwareMap, gamepad);
            gamepad.init(gamepad1, 0.3);
        }

        /**
         * This initializes the PoseUpdater, the mecanum drive motors, and the Panels telemetry.
         */
        @Override
        public void init_loop() {
            telemetryM.debug("This will print your robot's position to telemetry while "
                    + "allowing robot control through a basic mecanum drive on gamepad 1.");
            gamepad.update();
            telemetryM.update(telemetry);
            follower.update();
            drawCurrent();
        }

        @Override
        public void start() {
            follower.startTeleopDrive();
            follower.update();
        }

        /**
         * This updates the robot's pose estimate, the simple mecanum drive, and updates the
         * Panels telemetry with the robot's position as well as draws the robot's position.
         */
        @Override
        public void loop() {
            follower.update();
            gamepad.update();
            follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x);
            telemetryM.debug("x:" + follower.getPose().getX());
            telemetryM.debug("y:" + follower.getPose().getY());
            telemetryM.debug("heading:" + follower.getPose().getHeading());
            telemetryM.debug("total heading:" + follower.getTotalHeading());
            telemetryM.update(telemetry);

            drawCurrentAndHistory();
        }
    }

    class LimelightV1Test extends OpMode {
        public LimelightV1 limelight = new LimelightV1();
        public GamepadV1 gamepad = new GamepadV1();
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        @Override
        public void init() {
            limelight.init(hardwareMap);
            gamepad.init(gamepad1, 0.3);
        }

        @Override
        public void start(){
            limelight.startStreaming();
        }

        @Override
        public void loop() {
            gamepad.update();
            if(gamepad.circle && gamepad.isGamepadReady()){
                limelight.setPipeline(Arrays.stream(LimelightV1.Pipeline.values()).iterator().next());
                gamepad.resetTimer();
            }
            telemetryM.addData("Pipeline selected: ", limelight.getPipeline().name());
        }

    }
    class IntakeTest extends OpMode{
        public IntakeV1 intake = new IntakeV1();
        public GamepadV1 gamepad = new GamepadV1();
        public double power = RobotConstantsV1.INTAKE_POWER;
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        @Override
        public void init() {
            intake.init(hardwareMap);
            gamepad.init(gamepad1, 0.3);
            telemetryM.addLine("This OpMode is designed to test intake speed and direction. ");
            telemetryM.addLine("\tPress Circle to Switch directions");
            telemetryM.addLine("\tPress DPad Up/Down to switch intake directions.");
            telemetryM.addLine("Press start to start the OpMode.");
            telemetryM.update(telemetry);
        }

        @Override
        public void start(){
            intake.setPower(power);
        }

        @Override
        public void loop() {
            if(gamepad.circle && gamepad.isGamepadReady()){
                intake.switchDirection();
            }
            if(gamepad.dpad_up && gamepad.isGamepadReady()){
                power += 0.1;
                power = Math.max(Math.min(power, 1), -1);
            }
            telemetryM.addLine("Power: "+ power);
            telemetryM.addLine("Direction: " + intake.getDirection());
        }
    }

    class ShooterPIDTest extends OpMode {
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        public ArcShooterV1 arcShooter = new ArcShooterV1();
        public GamepadV1 gamepad = new GamepadV1();
        public MultipleTelemetry multipleTelemetry;
        public boolean leftEnabled = RobotConstantsV1.useBothArcMotors;

        @Override
        public void init() {
            multipleTelemetry = new MultipleTelemetry(telemetry, telemetryM);
            arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
            gamepad.init(gamepad1, 0.3);
        }

        @Override
        public void loop() {
            gamepad.update();
            arcShooter.run();
            arcShooter.updatePIDsFromConstants();
            if(gamepad.dpad_up && gamepad.isGamepadReady()){
                arcShooter.setTargetRPM(arcShooter.getTargetRPM() + RobotConstantsV1.velocityUpStep);
                gamepad.resetTimer();
            }
            else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
                arcShooter.setTargetRPM(arcShooter.getTargetRPM() - RobotConstantsV1.velocityDownStep);
                gamepad.resetTimer();
            }
            arcShooter.graph(telemetry);

        }
    }

    class SpindexerTest extends OpMode {
        public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
        public boolean readyToReset = false;
        public GamepadV1 gamepad = new GamepadV1();
        public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        @Override
        public void init() {
            spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
            gamepad.init(gamepad1, 0.3);
            telemetry.clear();
            telemetryM.addLine("This OpMode will tune the spindexer.");
            telemetryM.addLine("\tPress Circle to switch positions.");
            telemetryM.addLine("\tPress Triangle to switch encoder measurement modes.");
            telemetryM.addLine("\tPress BOTH bumpers to reset encoder position.");
            telemetryM.update(telemetry);
            spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
        }

        @Override
        public void loop() {
            gamepad.update();
            telemetryM.update(telemetry);
            telemetryM.addLine("Encoder measurement method: " + spindexer.getMeasurementMethod().name());
            telemetryM.addLine("Pos: " + spindexer.getCurrentPosition());
            telemetryM.addLine("Target pos (encoder): "+ spindexer.targetPositionEncoder);
            telemetryM.addLine("Error: " + spindexer.getError());
            telemetryM.addLine("Target pos: "+ spindexer.targetPosition);
            telemetryM.addLine("Next state: " + spindexer.getNextState());
            telemetryM.addLine("Timer: "  + gamepad.buttonTimer.seconds());
            telemetryM.addLine("Controller Power: " + spindexer.getControllerPower());
            spindexer.run();
            if(gamepad.circle && gamepad.isGamepadReady()) {
                spindexer.setState(spindexer.getNextState());
                gamepad.resetTimer();
                readyToReset = false;
            }
            if(gamepad.triangle && gamepad.isGamepadReady()) {
                spindexer.switchEncoderMeasurementMethods();
                gamepad.resetTimer();
                readyToReset = false;
            }
            if(gamepad.dpad_up && gamepad.isGamepadReady()){
                spindexer.setTargetPos(spindexer.getTargetPos() + 1000, spindexer.getMeasurementMethod());
                gamepad.resetTimer();
                readyToReset = false;
            }
            if(gamepad.dpad_down && gamepad.isGamepadReady()){
                spindexer.setTargetPos(spindexer.getTargetPos() - 1000, spindexer.getMeasurementMethod());
                gamepad.resetTimer();
                readyToReset = false;
            }
            if(gamepad.left_bumper && gamepad.right_bumper){
                readyToReset = true;
            }
            if(readyToReset){
                telemetryM.addLine("Press square to reset. ");
                if(gamepad.square && gamepad.isGamepadReady()){
                    spindexer.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    spindexer.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    spindexer.encoder.close();
                    spindexer.init(hardwareMap);
                    telemetryM.addLine("RESET COMPLETE.");
                    telemetry.clear();
                    gamepad.resetTimer();
                    readyToReset = false;
                }
            }
        }
    }

class SpindexerReadEncoderTest extends OpMode {
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        gamepad.init(gamepad1, 0.3);
        telemetry.clear();
        telemetryM.addLine("This OpMode will read spindexer motor encoder position.");
        telemetryM.addLine("\t Press CIRCLE to switch modes.");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        if(gamepad.circle && gamepad.isGamepadReady()){
            spindexer.switchEncoderMeasurementMethods();
            gamepad.resetTimer();
        }
        gamepad.update();
        telemetryM.update(telemetry);
        telemetryM.addLine("Encoder measurement method: " + spindexer.getMeasurementMethod().name());
        telemetryM.addLine("Pos: " + spindexer.getCurrentPosition());
        telemetryM.addLine("Target pos: "+ spindexer.getTargetPos());
    }
}

class SpindexerResetEncoder extends OpMode {
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        gamepad.init(gamepad1, 0.3);
        telemetry.clear();
        telemetryM.addLine("This OpMode will reset spindexer motor encoder position. Press START to reset, and press circle to reset again if needed while in loop.");
        telemetryM.update(telemetry);
    }

    @Override
    public void start(){
        spindexer.resetEncoder();
    }

    @Override
    public void loop() {
        if(gamepad.circle && gamepad.isGamepadReady()){
            gamepad.resetTimer();
            spindexer.resetEncoder();
        }
        gamepad.update();
        telemetryM.update(telemetry);
        telemetryM.addLine("Encoder reset.");
    }
}

class FlickerTest extends OpMode {
    public FlickerServoV1 flicker = new FlickerServoV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public boolean up = false;
    public double pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
    @Override
    public void init() {
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        gamepad.init(gamepad1, 0.3);
    }

    @Override
    public void start(){
        flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
    }

    @Override
    public void loop() {
        telemetryM.addLine("Position: " + pos);
        telemetryM.update(telemetry);
        gamepad.update();
        if(gamepad.isGamepadReady() && gamepad1.circle){
            pos = RobotConstantsV1.FLICKER_SERVO_UP;
            flicker.resetTimer();
            gamepad.resetTimer();
            up = true;
        }
        if(gamepad.isGamepadReady() && gamepad.dpad_up){
            pos = pos + 0.05;
            gamepad.resetTimer();
        }
        if(gamepad.isGamepadReady() && gamepad.dpad_down){
            pos = pos - 0.05;
            gamepad.resetTimer();
        }

        if(flicker.timer.seconds() > 1.5 && up){
            pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
            up = false;
        }
        flicker.setPosition(pos);
    }
}
class TurretServoTester extends OpMode {
    public LimelightV1 limelight = new LimelightV1();
    public GamepadV1 gamepad = new GamepadV1();
    public PIDController controller;
    public TurretServoCR turret = new TurretServoCR();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public int latestID = -1;
    public double rotationalError = 0;
    public double tx,ty, targetAngle, power = 0;
    public ElapsedTime noResultTimer = new ElapsedTime();

    @Override
    public void init() {
        controller = new PIDController(RobotConstantsV1.turretPIDs.p, RobotConstantsV1.turretPIDs.i, RobotConstantsV1.turretPIDs.d);
        limelight.init(hardwareMap);
        turret.init(hardwareMap);
        turret.setPower(0);
        gamepad.init(gamepad1, 0.3);
    }

    @Override
    public void start() {
        limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
        limelight.startStreaming();
    }

    @Override
    public void loop() {
        gamepad.update();
        LLResultTypes.FiducialResult latest_result = limelight.getLatestAprilTagResult();
        if(latest_result != null) {
            latestID = latest_result.getFiducialId();
            tx = latest_result.getTargetXDegreesNoCrosshair();
            ty = latest_result.getTargetYDegreesNoCrosshair();
            noResultTimer.reset();
        }
        if(noResultTimer.seconds() > 1){
            turret.setPower(0);
            tx = 0;
        }
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
             gamepad.resetTimer();
             power += 1;
        }
        if(gamepad.dpad_down && gamepad.isGamepadReady()){
             power -= 1;
             gamepad.resetTimer();
        }
        power = -controller.getOutput(tx + RobotConstantsV1.TURRET_OFFSET_ANGLE_BLUE);
        controller.updatePIDsFromConstants(RobotConstantsV1.turretPIDs);
        if(Math.abs((tx + RobotConstantsV1.TURRET_OFFSET_ANGLE_BLUE)) < RobotConstantsV1.degreeToleranceCamera){
            power = power / 3;
        }
        turret.setPower(power);
        telemetryM.addData("Position: ", turret.getEncoderPos());
        telemetryM.addLine("\tTarget X: "+ tx);
        telemetryM.addLine("\tTarget Y: "+ ty);
        telemetryM.addLine("\tPower: "+ power);
        telemetryM.addData("Pipeline selected: ", limelight.getPipeline().name());
        telemetryM.addData("Latest tag: ", latestID);
        telemetryM.update(telemetry);
    }
}
class TurretSpeedTest extends OpMode {
    public LimelightV1 limelight = new LimelightV1();
    public TurretServoCR servo = new TurretServoCR();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public double latestPos = 0;

    @Override
    public void init() {
        limelight.init(hardwareMap);
        servo.init(hardwareMap);
        gamepad.init(gamepad1, 0.3);
    }

    @Override
    public void start() {
        limelight.setPipeline(LimelightV1.Pipeline.BLUE_GOAL);
        limelight.startStreaming();
    }

    @Override
    public void loop() {
        double tx = 0;
        if(limelight.getLatestResult().isValid() && limelight.getLatestResult() != null) {
            tx = limelight.getLatestResult().getTxNC();
        }
        gamepad.update();
        servo.setPower(gamepad.left_stick_x);
        telemetryM.addData("Target X: ", tx);
        telemetryM.addData("Power: ", gamepad.left_stick_x);
        telemetryM.addData("Pos: ", servo.getEncoderPos());
        telemetryM.update(telemetry);
    }
}

class DeliverySubsystemsTest extends OpMode {
    public FlickerServoV1 flicker = new FlickerServoV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public boolean up = false;
    public boolean front = false;
    public double pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public MultipleTelemetry multipleTelemetry;
    public TurretServoV1 servo = new TurretServoV1();
    @Override
    public void init() {
        multipleTelemetry = new MultipleTelemetry(telemetry, telemetryM);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        gamepad.init(gamepad1, 0.3);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
        telemetryM.addLine("Controls: ");
        telemetryM.addLine("\tCircle: Fire ball");
        telemetryM.addLine("\tDpad Up/Down: Shooter RPM Up/Down");
        telemetryM.addLine("\tBumpers: Turret Control");
    }

    @Override
    public void start(){
        flicker.setPosition(RobotConstantsV1.FLICKER_SERVO_DOWN);
    }

    @Override
    public void loop() {
        telemetryM.addLine("Position: " + pos);
        telemetryM.update(telemetry);
        gamepad.update();
        arcShooter.run();

        arcShooter.updatePIDsFromConstants();
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            arcShooter.setTargetRPM(arcShooter.getTargetRPM() + RobotConstantsV1.velocityUpStep);
            gamepad.resetTimer();
        }
        else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
            arcShooter.setTargetRPM(arcShooter.getTargetRPM() - RobotConstantsV1.velocityDownStep);
            gamepad.resetTimer();
        }
        arcShooter.graph(multipleTelemetry);
        if(gamepad.isGamepadReady() && gamepad1.circle){
            pos = RobotConstantsV1.FLICKER_SERVO_UP;
            flicker.resetTimer();
            gamepad.resetTimer();
            up = true;
        }
        if(front){
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
        }
        else {
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
        }
        if(gamepad.isGamepadReady() && gamepad.left_bumper){
            servo.setPosition(servo.getPosition() + 0.05);
            gamepad.resetTimer();
        }

        if(gamepad.isGamepadReady() && gamepad.right_bumper){
            servo.setPosition(servo.getPosition() - 0.05);
            gamepad.resetTimer();
        }

        if(gamepad.isGamepadReady() && gamepad.options){
            front = !front;
            gamepad.resetTimer();
        }

        if(flicker.timer.seconds() > 1 && up){
            pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
            up = false;
        }
        flicker.setPosition(pos);
    }
}
class ColorSensorTest extends OpMode {
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public MultipleTelemetry multipleTelemetry;
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public IntakeV1 intake = new IntakeV1();
    public boolean intakeOn = true;
    public double dist = 0;
    public double gain = 1;

    @Override
    public void init() {
        multipleTelemetry = new MultipleTelemetry(telemetry, telemetryM);
        gamepad.init(gamepad1, 0.3);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        intake.init(hardwareMap);
        sensor.sensor.setGain(1);
        telemetryM.addLine("This OpMode reads the value from the robot's color sensor. ");
    }

    @Override
    public void loop() {
        dist = sensor.sensor.getDistance(DistanceUnit.MM);
        if(intakeOn) {
            intake.setDirection(RobotConstantsV1.intakeDirection);
            intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        else {
            intake.setPower(0);
        }
        RGB rgb = sensor.getRGB();
        NormalizedRGBA norm = sensor.getNormalizedRGB();
        gamepad.update();
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            gain += 1;
            gamepad.resetTimer();
        }
        if(gamepad.dpad_down && gamepad.isGamepadReady()){
            gain -= 1;
            gamepad.resetTimer();
        }
        if(gamepad.circle && gamepad.isGamepadReady()){
            intakeOn = !intakeOn;
            gamepad.resetTimer();
        }
        if(gain < 1){
            gain = 1;
        }
        sensor.sensor.setGain((float) gain);
        telemetryM.addLine("Red: " + rgb.r);
        telemetryM.addLine("Green: " + rgb.g);
        telemetryM.addLine("Blue: " + rgb.b);
        telemetryM.addLine("NORM_Red_PCT: " + norm.red);
        telemetryM.addLine("NORM_Green_PCT: " + norm.green);
        telemetryM.addLine("NORM_Blue_PCT: " + norm.blue);
        telemetryM.addLine("NORM_Red: " + norm.red / norm.alpha);
        telemetryM.addLine("NORM_Green: " + norm.green / norm.alpha);
        telemetryM.addLine("NORM_Blue: " + norm.blue / norm.alpha);
        telemetryM.addLine("NORM_ALPHA: " + norm.alpha);
        telemetryM.addLine("Gain: " + sensor.sensor.getGain());
        telemetryM.addLine("DISTANCE: " + dist);
        telemetryM.addLine("IS GREEN: " + sensor.isGreen());
        telemetryM.addLine("IS PURPLE: " + sensor.isPurple());
        telemetryM.update(telemetry);

    }
}
class IntakeSubsystemTest extends OpMode {
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    public IntakeV1 intake = new IntakeV1();

    @Override
    public void init() {
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        gamepad.init(gamepad1, 0.3);
        drive.init(hardwareMap, gamepad1);
        intake.init(hardwareMap);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        telemetry.clear();
        telemetryM.addLine("This OpMode will test the intake subsystem.");
        telemetryM.addLine("\tPress CIRCLE to switch spindexer states.");
        telemetryM.addLine("\tPress SQUARE to switch spindexer to shoot/intake positions.");
        telemetryM.update(telemetry);
        spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
    }

    @Override
    public void loop() {
        gamepad.update();
        drive.drive();
        intake.setPower(RobotConstantsV1.INTAKE_POWER);
        telemetryM.update(telemetry);
        telemetryM.addLine("Spindexer Pos: " + spindexer.getCurrentPosition());
        telemetryM.addLine("Spindexer target pos (encoder): "+ spindexer.targetPositionEncoder);
        telemetryM.addLine("Error: " + spindexer.getError());
        telemetryM.addLine("Next state: " + spindexer.getNextState());
        telemetryM.addLine("Button Timer: "  + gamepad.buttonTimer.seconds());
        telemetryM.addLine("Controller Power: " + spindexer.getControllerPower());
        spindexer.run();
        if(gamepad.circle && gamepad.isGamepadReady()) {
            spindexer.setState(spindexer.getNextState());
            gamepad.resetTimer();
        }
        if(gamepad.square && gamepad.isGamepadReady()) {
            intake.switchDirection();
            gamepad.resetTimer();
        }
        if(gamepad.left_trigger > 0.05 && gamepad.isGamepadReady()) {
            spindexer.switchToShootOrIntake();
            gamepad.resetTimer();
        }
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() + 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
        if(gamepad.dpad_down && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() - 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
    }
}

class AutomatedIntakeTest extends OpMode {
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public ArtifactColor slot1 = ArtifactColor.NONE;
    public ArtifactColor slot2 = ArtifactColor.NONE;
    public ArtifactColor slot3 = ArtifactColor.NONE;
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public IntakeV1 intake = new IntakeV1();
    public ElapsedTime captureTimer = new ElapsedTime();
    public double green = 0;
    public double lastGreen = 0;
    public double loops = 0;
    public double avgGreen = 0;
    public boolean finished = false;
    public double dist = 0;

    @Override
    public void init() {
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        gamepad.init(gamepad1, 0.3);
        drive.init(hardwareMap, gamepad);
        intake.init(hardwareMap);
        spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        dist = sensor.sensor.getDistance(DistanceUnit.MM);
    }

    @Override
    public void init_loop(){
        telemetry.clear();
        telemetryM.addLine("This OpMode will automate the intake subsystem using the color sensor.");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        drive.drive();
        dist = sensor.sensor.getDistance(DistanceUnit.MM);
        if(loops > 0) {
            avgGreen = green / loops;
        }
        else {
            avgGreen = 0;
        }

        gamepad.update();
       /* telemetryM.addLine("Distance: " + dist);
        telemetryM.addLine("Has ball? " + (RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist));
        telemetryM.addLine("Is not too close?: " + (dist > 26));
        telemetryM.addLine("Green captures: " + loops);
        telemetryM.addLine("Total green: " + green);
        telemetryM.addLine("Avg green: " + avgGreen);
        telemetryM.addLine("\n");
        telemetryM.addLine("Red: " + sensor.sensor.red());
        telemetryM.addLine("Green: "+ sensor.sensor.green());
        telemetryM.addLine("Blue: "+ sensor.sensor.blue());
        telemetryM.addLine("\n");
        telemetryM.addLine("SLOT: " + spindexer.state.name());
        telemetryM.addLine("Is green? : " + sensor.isGreen());
        telemetryM.addLine("Is purple? : " + sensor.isPurple());
        telemetryM.addLine("Is ready? : " + spindexer.isSpindexerReady());
        */
        telemetryM.setUpdateInterval(100);
        telemetryM.addLine("SLOT 1: " + slot1.name());
        telemetryM.addLine("SLOT 2: " + slot2.name());
        telemetryM.addLine("SLOT 3: " + slot3.name());

        telemetryM.update(telemetry);

        spindexer.run();

        if(spindexer.isSpindexerReady()) {
            if(slot1 == ArtifactColor.NONE){
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
            }
            else if(slot2 == ArtifactColor.NONE){
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
            }
            else if(slot3 == ArtifactColor.NONE){
                intake.setPower(RobotConstantsV1.INTAKE_POWER);
            }
            else {
                if(finished) {
                    spindexer.setState(SpindexerMotorV1.State.BALL_1_SHOOT);
                    intake.setPower(RobotConstantsV1.INTAKE_POWER * 0.9);
                }
            }
            switch (spindexer.state) {
                case BALL_1_INTAKE:
                    if(slot1 != ArtifactColor.NONE) {
                        spindexer.setState(spindexer.getNextState());
                        break;
                    }
                    break;
                case BALL_2_INTAKE:
                    if(slot2 != ArtifactColor.NONE) {
                        spindexer.setState(spindexer.getNextState());
                        break;
                    }
                    break;
                case BALL_3_INTAKE:
                    if(slot3 != ArtifactColor.NONE) {
                        finished = true;
                        break;
                    }
                    break;

            }
        }
        if(avgGreen < RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 2){
            switch (spindexer.state){
                case BALL_1_INTAKE:
                    slot1 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_2_INTAKE:
                    slot2 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_3_INTAKE:
                    slot3 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
            }
        }
        else if(avgGreen > RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 2){
            switch (spindexer.state){
                case BALL_1_INTAKE:
                    slot1 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_2_INTAKE:
                    slot2 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_3_INTAKE:
                    slot3 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
            }
        }
        //change 26 to 10
        else if(loops < 2 && captureTimer.seconds() > 0.03 && spindexer.isSpindexerReady() && RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist && dist > 10 && !finished){
            lastGreen = sensor.sensor.green();
            double brightness = sensor.sensor.red() +
                    lastGreen +
                    sensor.sensor.blue();
            if(brightness > 185) {
                green = green + lastGreen;
                lastGreen = 0;
                loops = loops + 1;
                captureTimer.reset();
            }
        }
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() + 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
        if(gamepad.dpad_down && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() - 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
    }
}

class PreTeleOpTest extends OpMode {
    public MecanumDriveOnly drive = new MecanumDriveOnly();
    public SpindexerMotorV1 spindexer = new SpindexerMotorV1();
    public GamepadV1 gamepad = new GamepadV1();
    public TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    public ArtifactColor slot1 = ArtifactColor.NONE;
    public ArtifactColor slot2 = ArtifactColor.NONE;
    public ArtifactColor slot3 = ArtifactColor.NONE;
    public SlotColorSensorV1 sensor = new SlotColorSensorV1();
    public IntakeV1 intake = new IntakeV1();
    public ElapsedTime captureTimer = new ElapsedTime();
    public double green = 0;
    public double lastGreen = 0;
    public double loops = 0;
    public double avgGreen = 0;
    public boolean finished = false;
    public double dist = 0;
    public FlickerServoV1 flicker = new FlickerServoV1();
    public boolean up = false;
    public boolean front = false;
    public double pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
    public ArcShooterV1 arcShooter = new ArcShooterV1();
    public MultipleTelemetry multipleTelemetry;
    public TurretServoV1 servo = new TurretServoV1();
    public enum Mode {
        SHOOT,
        INTAKE
    }
    public Mode mode = Mode.INTAKE;


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        multipleTelemetry = new MultipleTelemetry(telemetry, telemetryM);
        spindexer.init(hardwareMap, RobotConstantsV1.spindexerPIDs);
        sensor.init(hardwareMap, RobotConstantsV1.colorSensor1);
        servo.init(hardwareMap, false);
        gamepad.init(gamepad1, 0.3);
        drive.init(hardwareMap, gamepad);
        intake.init(hardwareMap);
        spindexer.setState(SpindexerMotorV1.State.BALL_1_INTAKE);
        intake.setDirection(RobotConstantsV1.intakeDirection);
        dist = sensor.sensor.getDistance(DistanceUnit.MM);
        flicker.init(hardwareMap, RobotConstantsV1.flickerServoName);
        gamepad.init(gamepad1, 0.3);
        arcShooter.init(hardwareMap, RobotConstantsV1.arcPIDs);
    }

    @Override
    public void init_loop(){
        telemetry.clear();
        telemetryM.addLine("This OpMode will automate the intake subsystem using the color sensor.");
        telemetryM.update(telemetry);
    }

    @Override
    public void loop() {
        drive.drive();
        drive.log(telemetry);
        dist = sensor.sensor.getDistance(DistanceUnit.MM);
        if(loops > 0) {
            avgGreen = green / loops;
        }
        else {
            avgGreen = 0;
        }

        gamepad.update();
       /* telemetryM.addLine("Distance: " + dist);
        telemetryM.addLine("Has ball? " + (RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist));
        telemetryM.addLine("Is not too close?: " + (dist > 26));
        telemetryM.addLine("Green captures: " + loops);
        telemetryM.addLine("Total green: " + green);
        telemetryM.addLine("Avg green: " + avgGreen);
        telemetryM.addLine("\n");
        telemetryM.addLine("Red: " + sensor.sensor.red());
        telemetryM.addLine("Green: "+ sensor.sensor.green());
        telemetryM.addLine("Blue: "+ sensor.sensor.blue());
        telemetryM.addLine("\n");
        telemetryM.addLine("SLOT: " + spindexer.state.name());
        telemetryM.addLine("Is green? : " + sensor.isGreen());
        telemetryM.addLine("Is purple? : " + sensor.isPurple());
        telemetryM.addLine("Is ready? : " + spindexer.isSpindexerReady());
        */
        telemetryM.setUpdateInterval(100);
        telemetryM.addLine("SLOT 1: " + slot1.name());
        telemetryM.addLine("SLOT 2: " + slot2.name());
        telemetryM.addLine("SLOT 3: " + slot3.name());

        telemetryM.update(telemetry);

        spindexer.run();
        if(mode == Mode.INTAKE){
            intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        else {
            intake.setPower(0.3);
        }

        if(spindexer.isSpindexerReady()) {
            switch (spindexer.state) {
                case BALL_1_INTAKE:
                    if(slot1 != ArtifactColor.NONE) {
                        spindexer.setState(spindexer.getNextState());
                        break;
                    }
                    break;
                case BALL_2_INTAKE:
                    if(slot2 != ArtifactColor.NONE) {
                        spindexer.setState(spindexer.getNextState());
                        break;
                    }
                    break;
                case BALL_3_INTAKE:
                    if(slot3 != ArtifactColor.NONE) {
                        finished = true;
                        break;
                    }
                    break;

            }
        }
        if(avgGreen < RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 5){
            switch (spindexer.state){
                case BALL_1_INTAKE:
                    slot1 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_2_INTAKE:
                    slot2 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_3_INTAKE:
                    slot3 = ArtifactColor.PURPLE;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
            }
        }
        else if(avgGreen > RobotConstantsV1.MIN_G_VALUE_COLOR_SENSOR && loops >= 5){
            switch (spindexer.state){
                case BALL_1_INTAKE:
                    slot1 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_2_INTAKE:
                    slot2 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
                case BALL_3_INTAKE:
                    slot3 = ArtifactColor.GREEN;
                    loops = 0;
                    green = 0;
                    avgGreen = 0;
                    break;
            }
        }
        //change 26 to 10
        else if(loops < 5 && captureTimer.seconds() > 0.03 && spindexer.isSpindexerReady() && RobotConstantsV1.MAX_DISTANCE_COLOR_SENSOR > dist && dist > 10 && !finished){
            lastGreen = sensor.sensor.green();
            double brightness = sensor.sensor.red() +
                    lastGreen +
                    sensor.sensor.blue();
            if(brightness > 185) {
                green = green + lastGreen;
                lastGreen = 0;
                loops = loops + 1;
                captureTimer.reset();
            }
        }
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() + 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
        if(gamepad.dpad_down && gamepad.isGamepadReady()){
            spindexer.setTargetPos(spindexer.getTargetPos() - 1000, spindexer.getMeasurementMethod());
            gamepad.resetTimer();
        }
        gamepad.update();
        if(spindexer.state == SpindexerMotorV1.State.BALL_1_SHOOT || spindexer.state == SpindexerMotorV1.State.BALL_2_SHOOT || spindexer.state == SpindexerMotorV1.State.BALL_3_SHOOT) {
            arcShooter.run();
        }
        else {
            arcShooter.setTargetRPM(2000);
            arcShooter.run();
        }

        arcShooter.updatePIDsFromConstants();
        if(gamepad.dpad_up && gamepad.isGamepadReady()){
            arcShooter.setTargetRPM(arcShooter.getTargetRPM() + RobotConstantsV1.velocityUpStep);
            gamepad.resetTimer();
        }
        else if(gamepad.dpad_down && gamepad.isGamepadReady()) {
            arcShooter.setTargetRPM(arcShooter.getTargetRPM() - RobotConstantsV1.velocityDownStep);
            gamepad.resetTimer();
        }
        arcShooter.graph(multipleTelemetry);
        if(gamepad.isGamepadReady() && gamepad1.circle){
            pos = RobotConstantsV1.FLICKER_SERVO_UP;
            flicker.resetTimer();
            gamepad.resetTimer();
            up = true;
        }
        if(gamepad.isGamepadReady() && gamepad.options){
            spindexer.switchToShootOrIntake();
            if(mode == Mode.SHOOT){
                mode = Mode.INTAKE;
            }
            else {
                mode  = Mode.SHOOT;
            }
            gamepad.resetTimer();
        }
        if(front){
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_FRONT_RPM);
        }
        else {
            arcShooter.setTargetRPM(RobotConstantsV1.SHOOTER_TARGET_RPM);
        }
        if(gamepad.isGamepadReady() && gamepad.left_bumper){
            servo.setPosition(servo.getPosition() + 0.05);
            gamepad.resetTimer();
        }

        if(gamepad.isGamepadReady() && gamepad.right_bumper){
            servo.setPosition(servo.getPosition() - 0.05);
            gamepad.resetTimer();
        }

        if(gamepad.isGamepadReady() && gamepad.options){
            front = !front;
            gamepad.resetTimer();
        }

        if(flicker.timer.seconds() > 1 && up){
            pos = RobotConstantsV1.FLICKER_SERVO_DOWN;
            spindexer.setState(spindexer.getNextState());
            up = false;
        }
        flicker.setPosition(pos);
    }
}

class LogReader extends OpMode {
    public LoggerV1 logger = new LoggerV1();
    public String[] lines;
    @Override
    public void init() {
        logger.init(telemetry);
        lines = logger.read("robotLogNewest.txt");
    }

    @Override
    public void loop() {
        if(lines != null) {
            for (String line : lines) {
                telemetry.addLine(line);
            }
        }
        else {
            telemetry.addLine("Logfile is null.");
        }
    }
}

