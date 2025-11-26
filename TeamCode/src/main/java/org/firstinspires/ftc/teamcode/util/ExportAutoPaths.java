package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.opmodes.DecodeAutonomousCloseCommand;

import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

/**
 * Command-line utility to export autonomous paths as .pp files for Pedro Pathing visualizer.
 *
 * Usage from command line:
 *   ./gradlew :TeamCode:exportPaths
 *
 * Or run directly:
 *   ./gradlew :TeamCode:classes
 *   java -cp TeamCode/build/intermediates/javac/debug/classes org.firstinspires.ftc.teamcode.util.ExportAutoPaths [output_dir]
 *
 * Generated files:
 *   - autonomous_close_blue.pp
 *   - autonomous_close_red.pp
 *
 * Load these files in Pedro Path Generator: https://pedro-path-generator.vercel.app/
 */
public class ExportAutoPaths {

    private static final String DEFAULT_OUTPUT_DIR = ".";

    public static void main(String[] args) {
        String outputDir = args.length > 0 ? args[0] : DEFAULT_OUTPUT_DIR;

        System.out.println("=== Autonomous Path Exporter ===");
        System.out.println();
        System.out.println("Generating .pp files for Pedro Pathing visualizer...");
        System.out.println("Output directory: " + outputDir);
        System.out.println();

        try {
            // Ensure output directory exists
            Files.createDirectories(Paths.get(outputDir));

            // Export Blue Alliance
            String blueJson = DecodeAutonomousCloseCommand.exportToPedroPathFile(Alliance.BLUE);
            String blueFile = outputDir + "/autonomous_close_blue.pp";
            writeFile(blueFile, blueJson);
            System.out.println("✓ Generated: " + blueFile);

            // Export Red Alliance
            String redJson = DecodeAutonomousCloseCommand.exportToPedroPathFile(Alliance.RED);
            String redFile = outputDir + "/autonomous_close_red.pp";
            writeFile(redFile, redJson);
            System.out.println("✓ Generated: " + redFile);

            System.out.println();
            System.out.println("=== SUCCESS ===");
            System.out.println();
            System.out.println("Next steps:");
            System.out.println("1. Open Pedro Path Generator: https://pedro-path-generator.vercel.app/");
            System.out.println("2. File → Load → select .pp file");
            System.out.println("3. Visualize and verify your autonomous paths");
            System.out.println();
            System.out.println("To adjust paths:");
            System.out.println("- Edit CloseAutoWaypoints in DecodeAutonomousCloseCommand.java");
            System.out.println("- Re-run: ./gradlew :TeamCode:exportPaths");
            System.out.println("- Reload .pp file in visualizer");

        } catch (IOException e) {
            System.err.println("ERROR: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static void writeFile(String filePath, String content) throws IOException {
        try (FileWriter writer = new FileWriter(filePath)) {
            writer.write(content);
        }
    }
}
