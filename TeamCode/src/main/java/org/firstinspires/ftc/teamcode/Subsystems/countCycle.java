package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import android.os.Environment;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class countCycle {
    // var for servo name and file name and start count and current count
    String filename;
    public double startCount;
    public double currentCount;

    // constructor taking in servo name and file name and setting the class variables to them

    public countCycle(String filename)
    {
        createFile(filename);
        this.startCount = readFile(filename);
        this.currentCount = 0;
    }

    public double readFile(String filename) {
        File file = new File(Environment.getExternalStorageDirectory()+filename);
        Double readData = null;
        try (BufferedReader reader = new BufferedReader(new FileReader(filename))) {
            String line;
            // Read each line until the end of the file (returns null)
            while ((line = reader.readLine()) != null) {
                readData = Double.parseDouble(line);
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return readData;
    }
    public void updateCount(double data){
        File file = new File(Environment.getExternalStorageDirectory()+filename);

        //PRINT to file
        try (PrintWriter out = new PrintWriter(new FileWriter(filename))) {
            out.println(data);

        } catch (IOException e) {
            telemetry.addLine("Error writing to file: " + e.getMessage());
        }

    }

    public void createFile(String filename){
        File file = new File(Environment.getExternalStorageDirectory()+filename);
        try{
            if (file.createNewFile()) {
                telemetry.addLine("File created successfully: " + file.getAbsolutePath());
            } else {
                telemetry.addLine("File already exists: " + file.getAbsolutePath());
            }
        }
        catch(IOException e){
            telemetry.addLine(e.getMessage());
        }
    }
}
