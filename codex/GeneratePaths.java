import java.io.FileWriter;
import java.io.IOException;

/**
 * Standalone path generator - no Gradle, no Android, no dependencies.
 *
 * Usage:
 *   javac codex/GeneratePaths.java
 *   java -cp codex GeneratePaths
 *
 * Generates:
 *   autonomous_close_blue.pp
 *   autonomous_close_red.pp
 */
public class GeneratePaths {

    // ========== EDIT THESE VALUES ==========
    // Copy from CloseAutoWaypoints in DecodeAutonomousCloseCommand.java

    static double startX = 26.445;
    static double startY = 131.374;
    static double startHeading = 144;

    static double launchX = 30.19905213270142;
    static double launchY = 112.9478672985782;
    static double launchHeading = 136;

    static double preArtifactsX = 28;
    static double preArtifactsY = 112;
    static double preArtifactsHeading = 270;

    static double artifacts1X = 22;
    static double artifacts1Y = 87;
    static double artifacts1Heading = 270;

    static double transition2X = 40;
    static double transition2Y = 90;
    static double transition2Heading = 270;

    static double artifacts2X = 34;
    static double artifacts2Y = 70;
    static double artifacts2Heading = 270;

    static double transition3X = 40;
    static double transition3Y = 62;
    static double transition3Heading = 270;

    static double artifacts3X = 40;
    static double artifacts3Y = 32.5;
    static double artifacts3Heading = 270;

    static double finalX = 37;
    static double finalY = 128;
    static double finalHeading = 142;

    // ========================================

    static final double FIELD_WIDTH = 144.0;

    public static void main(String[] args) {
        System.out.println("=== Pedro Path Generator ===");
        System.out.println();

        try {
            // Generate Blue alliance
            String blueJson = generatePaths(false);
            writeFile("autonomous_close_blue.pp", blueJson);
            System.out.println("✓ Generated: autonomous_close_blue.pp");

            // Generate Red alliance
            String redJson = generatePaths(true);
            writeFile("autonomous_close_red.pp", redJson);
            System.out.println("✓ Generated: autonomous_close_red.pp");

            System.out.println();
            System.out.println("=== SUCCESS ===");
            System.out.println();
            System.out.println("Load files in: https://pedro-path-generator.vercel.app/");

        } catch (IOException e) {
            System.err.println("ERROR: " + e.getMessage());
            System.exit(1);
        }
    }

    static String generatePaths(boolean isRed) {
        StringBuilder json = new StringBuilder();
        json.append("{");

        // Convert and mirror poses
        Pose start = pose(startX, startY, startHeading, isRed);
        Pose launch = pose(launchX, launchY, launchHeading, isRed);
        Pose preArtifacts = pose(preArtifactsX, preArtifactsY, preArtifactsHeading, isRed);
        Pose set1 = pose(artifacts1X, artifacts1Y, artifacts1Heading, isRed);
        Pose trans2 = pose(transition2X, transition2Y, transition2Heading, isRed);
        Pose set2 = pose(artifacts2X, artifacts2Y, artifacts2Heading, isRed);
        Pose trans3 = pose(transition3X, transition3Y, transition3Heading, isRed);
        Pose set3 = pose(artifacts3X, artifacts3Y, artifacts3Heading, isRed);
        Pose finalPos = pose(finalX, finalY, finalHeading, isRed);

        // Start point
        json.append("\"startPoint\":{");
        json.append("\"x\":").append(start.x).append(",");
        json.append("\"y\":").append(start.y).append(",");
        json.append("\"heading\":\"linear\",");
        json.append("\"startDeg\":").append(start.heading).append(",");
        json.append("\"endDeg\":").append(launch.heading);
        json.append("},");

        // Paths
        json.append("\"lines\":[");

        addPath(json, "To Launch", start, launch);
        json.append(",");
        addPath(json, "To Pre-Artifacts", launch, preArtifacts);
        json.append(",");
        addPath(json, "Collect Set 1", preArtifacts, set1);
        json.append(",");
        addPath(json, "Score Set 1", set1, launch);
        json.append(",");
        addPath(json, "To Transition 2", launch, trans2);
        json.append(",");
        addPath(json, "Collect Set 2", trans2, set2);
        json.append(",");
        addPath(json, "Score Set 2", set2, launch);
        json.append(",");
        addPath(json, "To Transition 3", launch, trans3);
        json.append(",");
        addPath(json, "Collect Set 3", trans3, set3);
        json.append(",");
        addPath(json, "To Final", set3, finalPos);

        json.append("],");
        json.append("\"shapes\":[]");
        json.append("}");

        return json.toString();
    }

    static Pose pose(double x, double y, double headingDeg, boolean isRed) {
        if (isRed) {
            double mirroredX = FIELD_WIDTH - x;
            double mirroredHeading = 180.0 - headingDeg;
            return new Pose(mirroredX, y, mirroredHeading);
        }
        return new Pose(x, y, headingDeg);
    }

    static void addPath(StringBuilder json, String name, Pose start, Pose end) {
        int colorHash = name.hashCode();
        String color = String.format("#%06X", (colorHash & 0xFFFFFF));

        json.append("{");
        json.append("\"name\":\"").append(name).append("\",");
        json.append("\"endPoint\":{");
        json.append("\"x\":").append(end.x).append(",");
        json.append("\"y\":").append(end.y).append(",");
        json.append("\"heading\":\"linear\",");
        json.append("\"startDeg\":").append(start.heading).append(",");
        json.append("\"endDeg\":").append(end.heading);
        json.append("},");
        json.append("\"controlPoints\":[],");
        json.append("\"color\":\"").append(color).append("\"");
        json.append("}");
    }

    static void writeFile(String filename, String content) throws IOException {
        try (FileWriter writer = new FileWriter(filename)) {
            writer.write(content);
        }
    }

    static class Pose {
        double x, y, heading;
        Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }
}
