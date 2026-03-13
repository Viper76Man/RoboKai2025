package org.firstinspires.ftc.teamcode.Jack.Drive;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Coach.subsystems.Ramp;
import org.firstinspires.ftc.teamcode.Jack.Camera.Limelight3A.LimelightV1;
import org.firstinspires.ftc.teamcode.Jack.Motors.SpindexerMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Other.BallManager;
import org.firstinspires.ftc.teamcode.Jack.Other.Sensors;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ArcMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.ColorSensorV3;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.DriveMotorsV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.IntakeMotorV2;
import org.firstinspires.ftc.teamcode.Jack.Subsystems.LimelightSubsystem;

import java.util.Objects;
import java.util.logging.Handler;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class RobotV4 implements Subsystem {
    public ParallelGroup driveCommand;
    public Command fireCommand;



    public HardwareMap hardwareMap;
    public Telemetry telemetry;
    public TelemetryManager telemetryM;
    public GamepadV1 gamepad = new GamepadV1();

    public enum SystemStates {
        START,
        BALL_1_INTAKE,
        BALL_2_INTAKE,
        BALL_3_INTAKE,
        SHOOT_ALL,
        SHOOT_SINGLE
    }


    public SpindexerMotorV2 spindexer = SpindexerMotorV2.INSTANCE;
    public ColorSensorV3 sensor = new ColorSensorV3();
    public IntakeMotorV2 intake = new IntakeMotorV2();
    public ArcMotorsV2 arc = new ArcMotorsV2();
    public LimelightV1 limelight = new LimelightV1();
    public DriveMotorsV2 drive = new DriveMotorsV2();
    public Sensors sensors = new Sensors();
    public LimelightSubsystem ll;

    public Robot.Alliance alliance;

    public BallManager manager = new BallManager();
    public SystemStates state = SystemStates.START;

    public boolean shooting = false;


    private void init(){
        hardwareMap = ActiveOpMode.hardwareMap();
        telemetry = ActiveOpMode.telemetry();
        gamepad.init(ActiveOpMode.gamepad1(), 0.3);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        ll = new LimelightSubsystem(Robot.Mode.TELEOP, alliance);
        ll.init();
        drive.init(hardwareMap, gamepad);
        sensors.init(hardwareMap);
        spindexer.init(manager);
        sensor.init(hardwareMap, manager, spindexer);
        arc.init(hardwareMap, Robot.Mode.TELEOP, ll.limelight, sensors);
        intake.init(hardwareMap);
        spindexer.resetMotor();
    }

    public RobotV4(Robot.Alliance alliance){
        this.alliance = alliance;
    }

    public void initialize(){
        init();
    }

    public void buildCommands(){
        driveCommand = new ParallelGroup(
                drive.drive(),
                sensor.update(),
                spindexer.spindexerUpdate()
        );
        fireCommand = buildShot();
    }

    public Command buildShot(){
        return new ParallelRaceGroup(
                spindexer.Fire(),
                arc.spinActive(),
                new Delay(5)
        );
    }

    public void systemStatesUpdate(){
        if(gamepad.left_trigger >= 0.15){
            intake.intake.setDirection(intake.intake.invertDirection(RobotConstantsV1.intakeDirection));
            intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        else {
            intake.intake.setDirection(RobotConstantsV1.intakeDirection);
            intake.intake.setPower(RobotConstantsV1.INTAKE_POWER);
        }
        sensors.update();
        gamepad.update();
        if(gamepad.right_trigger > 0.15 && !fireCommand.isScheduled() && gamepad.isGamepadReady()){
            fireCommand.schedule();
            setSystemState(SystemStates.START);
            shooting = true;
            gamepad.resetTimer();
        }
        switch (state){
            case START:
                if (Objects.equals(fireCommand, null)) {
                    fireCommand = buildShot();
                }
                if(fireCommand.isDone() && shooting) {
                    fireCommand.cancel();
                    fireCommand = buildShot();
                    driveCommand.cancel();
                    driveCommand.schedule();
                    Ramp.INSTANCE.rampDown.schedule();
                    manager.setMode(BallManager.State.INTAKE);
                    setSystemState(SystemStates.BALL_1_INTAKE);
                    spindexer.setState(SpindexerMotorV2.SpindexerState.INTAKE_BALL_1);
                    manager.setCurrentBall(1);
                    shooting = false;
                    arc.spinUpIdle().schedule();
                }
                if(!shooting){
                    fireCommand.cancel();
                    fireCommand = buildShot();
                    driveCommand.cancel();
                    driveCommand.schedule();
                    Ramp.INSTANCE.rampDown.schedule();
                    manager.setMode(BallManager.State.INTAKE);
                    spindexer.setState(SpindexerMotorV2.SpindexerState.INTAKE_BALL_1);
                    setSystemState(SystemStates.BALL_1_INTAKE);
                    arc.spinUpIdle().schedule();
                }
                break;
            case BALL_1_INTAKE:
                manager.setCurrentBall(1);
                if(sensor.hasBall() && !spindexer.isActive()){
                    spindexer.next();
                    setSystemState(SystemStates.BALL_2_INTAKE);
                    sensor.clear();
                }
                break;
            case BALL_2_INTAKE:
                manager.setCurrentBall(2);
                if(sensor.hasBall() && !spindexer.isActive()){
                    spindexer.next();
                    setSystemState(SystemStates.BALL_3_INTAKE);
                    sensor.clear();
                }
                break;
            case BALL_3_INTAKE:
                manager.setCurrentBall(3);
                if(sensor.hasBall() && !spindexer.isActive()){
                    manager.setCurrentBall(4);
                    manager.setMode(BallManager.State.SHOOT);
                    setSystemState(SystemStates.SHOOT_ALL);
                    sensor.clear();
                }
                break;
            case SHOOT_ALL:
                if(!fireCommand.isScheduled() && gamepad.isGamepadReady() && gamepad.right_trigger > 0.15) {
                    shoot();
                    gamepad.resetTimer();
                }
                break;
        }

    }

    public void shoot(){
        fireCommand = buildShot();
        fireCommand.schedule();
        setSystemState(SystemStates.START);
        shooting = true;
    }

    public void setSystemState(SystemStates state){
        this.state = state;
    }

    public void log(){
        telemetryM.addLine("Commands running: " + CommandManager.INSTANCE.snapshot().size());
        telemetryM.update(telemetry);
    }
}
