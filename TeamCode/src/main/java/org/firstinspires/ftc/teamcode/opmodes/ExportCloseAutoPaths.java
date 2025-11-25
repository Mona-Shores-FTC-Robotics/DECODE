package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Utility OpMode to export autonomous close paths as .pp files for Pedro Pathing visualizer.
 *
 * Usage:
 * 1. Run this OpMode on the robot
 * 2. Press START to generate files
 * 3. Files are saved to /sdcard/FIRST/
 * 4. Transfer files to your computer
 * 5. Load in Pedro Pathing GUI (https://pedro-path-generator.vercel.app/)
 *
 * Generated files:
 * - autonomous_close_blue.pp
 * - autonomous_close_red.pp
 */
@TeleOp(name = "Export Close Auto Paths", group = "Utility")
public class ExportCloseAutoPaths extends LinearOpMode {

    private static final String OUTPUT_DIR = "/sdcard/FIRST/";

    @Override
    public void runOpMode() {
        telemetry.addLine("=== Path Exporter ===");
        telemetry.addLine();
        telemetry.addLine("This will generate .pp files for:");
        telemetry.addLine("- Blue Alliance");
        telemetry.addLine("- Red Alliance");
        telemetry.addLine();
        telemetry.addLine("Files will be saved to:");
        telemetry.addLine(OUTPUT_DIR);
        telemetry.addLine();
        telemetry.addLine("Press START to generate");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.clear();
            telemetry.addLine("Generating path files...");
            telemetry.update();

            try {
                // Ensure output directory exists
                File dir = new File(OUTPUT_DIR);
                if (!dir.exists()) {
                    dir.mkdirs();
                }

                // Export Blue Alliance paths
                String blueJson = DecodeAutonomousCloseCommand.exportToPedroPathFile(Alliance.BLUE);
                File blueFile = new File(OUTPUT_DIR + "autonomous_close_blue.pp");
                writeFile(blueFile, blueJson);
                telemetry.addLine("✓ Blue alliance: " + blueFile.getAbsolutePath());

                // Export Red Alliance paths
                String redJson = DecodeAutonomousCloseCommand.exportToPedroPathFile(Alliance.RED);
                File redFile = new File(OUTPUT_DIR + "autonomous_close_red.pp");
                writeFile(redFile, redJson);
                telemetry.addLine("✓ Red alliance: " + redFile.getAbsolutePath());

                telemetry.addLine();
                telemetry.addLine("=== SUCCESS ===");
                telemetry.addLine();
                telemetry.addLine("Next steps:");
                telemetry.addLine("1. Connect robot to computer via USB");
                telemetry.addLine("2. Copy files from " + OUTPUT_DIR);
                telemetry.addLine("3. Open Pedro Path Generator:");
                telemetry.addLine("   https://pedro-path-generator.vercel.app/");
                telemetry.addLine("4. File → Load → select .pp file");
                telemetry.update();

            } catch (IOException e) {
                telemetry.addLine("ERROR: " + e.getMessage());
                telemetry.update();
            }

            // Keep OpMode running so telemetry is visible
            while (opModeIsActive()) {
                sleep(100);
            }
        }
    }

    private void writeFile(File file, String content) throws IOException {
        try (FileWriter writer = new FileWriter(file)) {
            writer.write(content);
        }
    }
}
