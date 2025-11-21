package org.firstinspires.ftc.teamcode.Jack.Other;

import android.net.wifi.WifiManager;
import android.text.format.Time;

import com.bylazar.panels.Panels;

import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.LocalTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

public class LoggerV1 {
    public enum Status {
        BROWSING,
        IN_FILE
    }

    public File currentDir;
    public Status status = Status.BROWSING;
    public BufferedWriter writer;
    public String currentDirPath = "";

    public Telemetry telemetry;

    public void init(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public int setCurrentFolder(String path) {
        File folder = new File(path);
        if (folder.exists() && folder.isDirectory()) {
            currentDirPath = path;
            currentDir = folder;
            return 0;
        } else if (!folder.exists() && folder.isDirectory()) {
            return 1;
        } else if (folder.exists() && !folder.isDirectory()) {
            return 2;
        }
        return 3;
    }

    public File joinStingsIntoPath(String folder, String file) {
        return new File(folder + file);
    }

    public int openFile(String file, boolean create) {
        File current = new File(file);
        if (create) {
            if (current.exists() && current.isFile()) {
                int status_ = createFileWriter(file);
                status = Status.IN_FILE;
                return 0;
            }
            else if (!current.exists()) {
                status = Status.BROWSING;
                return 1;
            } else if (current.exists() && !current.isFile()) {
                status = Status.BROWSING;
                return 2;
            }
            return 3;
        }
        else {
            if (current.exists() && current.isFile()) {
                int status_ = createFileWriter(file);
                status = Status.IN_FILE;
                return 1;
            } else if (!current.exists()) {
                status = Status.BROWSING;
                return 0;
            } else if (current.exists() && !current.isDirectory()) {
                status = Status.BROWSING;
                return 2;
            }
            return 3;
            }
        }

    public void info(String message){
        try {
            if (status == Status.IN_FILE) {
                writer.write("["+getTime()+"][INFO] " + message);
            }
        }
        catch (IOException e){
            telemetry.addData("Error: ", e);
        }
    }

    public List<String> readLines(String file){
        List<String> lines = new ArrayList<>();
        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;
            while ((line = br.readLine()) != null) {
                lines.add(line);
            }
        }
        catch (RuntimeException | IOException e){
            telemetry.addLine(e.getMessage());
        }
        return lines;
    }

    public void error(String message){
        try {
            if (status == Status.IN_FILE) {
                writer.write("["+getTime()+"][ERROR] " + message);
            }
        }
        catch (IOException e){
            telemetry.addData("Error: ", e);
        }
    }

    public void close() throws IOException {
        if (status == Status.IN_FILE) {
            status = Status.BROWSING;
            writer.close();
        }
    }

    public int createFileWriter(String file) {
        try {
            writer = new BufferedWriter(new FileWriter(file));
            return 0;
        }
        catch (IOException e){
            telemetry.addLine("Error: " + e.getMessage());
            return 1;
        }
    }
    public String getTime(){
        Calendar calendar = Calendar.getInstance();

        // 24-hour format HH:mm:ss
        SimpleDateFormat sdf = new SimpleDateFormat("HH:mm:ss");

        return sdf.format(calendar.getTime());
    }
    public boolean exists(String file){
        return new File(file).exists();
    }
    public boolean isFile(String file){
        return new File(file).isFile();
    }
    public boolean isDirectory(String file){
        return new File(file).isDirectory();
    }

}
