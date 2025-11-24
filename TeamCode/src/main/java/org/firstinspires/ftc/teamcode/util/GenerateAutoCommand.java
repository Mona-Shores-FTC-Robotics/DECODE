package org.firstinspires.ftc.teamcode.util;

import org.json.JSONException;

import java.io.IOException;

/**
 * Simple test runner to generate autonomous commands from .pp files
 *
 * Edit the main method to change which .pp file to generate from
 */
public class GenerateAutoCommand {

    public static void main(String[] args) {
        // ===== EDIT THESE VALUES =====
        String ppFileName = "trajectory.pp";  // .pp file in TeamCode/src/main/assets/
        String commandName = "TrajectoryAuto";  // Name for generated command

        // Paths (you shouldn't need to change these)
        String assetsPath = "TeamCode/src/main/assets/";
        String outputPath = "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands/generated/";

        System.out.println("===========================================");
        System.out.println("Generating Autonomous Command from .pp file");
        System.out.println("===========================================");
        System.out.println("Input:  " + ppFileName);
        System.out.println("Output: " + commandName + "Command.java");
        System.out.println("        " + commandName + "Config.java");
        System.out.println();

        try {
            PpFileCommandGenerator.generate(ppFileName, commandName, assetsPath, outputPath);

            System.out.println();
            System.out.println("✓ SUCCESS!");
            System.out.println();
            System.out.println("Next steps:");
            System.out.println("1. Open " + commandName + "Command.java");
            System.out.println("2. Fill in TODO comments with robot actions");
            System.out.println("3. Create an OpMode that instantiates the command");
            System.out.println("4. Tune waypoints in FTC Dashboard if needed");

        } catch (IOException e) {
            System.err.println("ERROR: Could not read/write files");
            System.err.println("  " + e.getMessage());
            e.printStackTrace();
        } catch (JSONException e) {
            System.err.println("ERROR: Invalid .pp file format");
            System.err.println("  " + e.getMessage());
            e.printStackTrace();
        }
    }
}
