package org.firstinspires.ftc.teamcode.Jack.Other;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Jack.Drive.Robot;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.Calendar;

public class LoggerV1 {
    public String logfile;
    public File file;
    public MultipleTelemetry multipleTelemetry;
    public Telemetry telemetry;

    public void init(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public void init(MultipleTelemetry telemetry) {
        this.multipleTelemetry = telemetry;
    }

    public void open(String filename) {
        this.file = AppUtil.getInstance().getSettingsFile(filename);
    }

    public void info(String string) {
        if(this.file != null) {
            string = getTime() + " [INFO] " + string;
            ReadWriteFile.writeFile(this.file, string);
            logToTelemetry(string);
        }
        else {
            logToTelemetry("ERROR: LOGFILE IS NULL. PLEASE OPEN FILE BEFORE WRITING.");
        }
    }

    public void error(String string) {
        if(this.file != null) {
            string = getTime() + " [ERROR] " + string;
            ReadWriteFile.writeFile(this.file, string);
            logToTelemetry(string);
        }
        else {
            logToTelemetry("ERROR: LOGFILE IS NULL. PLEASE OPEN FILE BEFORE WRITING.");
        }
    }

    public void logToTelemetry(String line){
        if(telemetry == null){
            multipleTelemetry.addLine(line);
        }
        else {
            telemetry.addLine(line);
        }
    }
    public String getTime(){
        Calendar calendar = Calendar.getInstance();

        // 24-hour format HH:mm:ss
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");

        return "[" + sdf.format(calendar.getTime()) + "]";
    }

    public void saveSideToFile(Robot.Alliance alliance){
        File file_ = getFile("alliance.txt");
        ReadWriteFile.writeFile(file_, alliance.name() + "\n");
    }

    public Robot.Alliance readSideFromFile(){
        String[] types = ReadWriteFile.readFile(getFile("alliance.txt")).split("\n");
        return Robot.Alliance.valueOf(types[types.length - 1]);
    }
    public File getFile(String filename){
        return AppUtil.getInstance().getSettingsFile(filename);
    }
}