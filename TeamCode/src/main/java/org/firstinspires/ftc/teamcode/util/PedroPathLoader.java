package org.firstinspires.ftc.teamcode.util;

import android.content.res.AssetManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * Loads Pedro Pathing .pp files (JSON format) and converts them to PathChain segments.
 *
 * Supports:
 * - Straight lines (BezierLine)
 * - Curved paths with control points (BezierCurve)
 * - Alliance mirroring for red/blue
 * - Reverse paths
 * - Linear, tangential, and constant heading interpolation
 */
public class PedroPathLoader {

    /**
     * Represents a single path segment from a .pp file
     */
    public static class PathSegment {
        public final String name;
        public final Pose startPose;
        public final Pose endPose;
        public final List<Pose> controlPoints;
        public final boolean isReverse;
        public final String headingMode; // "linear", "tangential", "constant"

        public PathSegment(String name, Pose startPose, Pose endPose,
                          List<Pose> controlPoints, boolean isReverse, String headingMode) {
            this.name = name;
            this.startPose = startPose;
            this.endPose = endPose;
            this.controlPoints = controlPoints;
            this.isReverse = isReverse;
            this.headingMode = headingMode;
        }

        /**
         * Builds a PathChain for this segment with custom heading interpolation
         * @param follower Pedro follower instance
         * @param headingInterpolation Override heading interpolation (0.0 - 1.0), or -1 to use default
         * @return PathChain ready for FollowPath command
         */
        public PathChain buildPathChain(Follower follower, double headingInterpolation) {
            double interpolation = headingInterpolation >= 0 ? headingInterpolation : 0.5;

            if (controlPoints.isEmpty()) {
                // Straight line
                return follower.pathBuilder()
                    .addPath(new BezierLine(startPose, endPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), interpolation)
                    .build();
            } else {
                // Curved path with control points
                Pose[] allPoints = new Pose[2 + controlPoints.size()];
                allPoints[0] = startPose;
                for (int i = 0; i < controlPoints.size(); i++) {
                    allPoints[i + 1] = controlPoints.get(i);
                }
                allPoints[allPoints.length - 1] = endPose;

                return follower.pathBuilder()
                    .addPath(new BezierCurve(allPoints))
                    .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading(), interpolation)
                    .build();
            }
        }

        /**
         * Builds a PathChain with default heading interpolation (0.5)
         */
        public PathChain buildPathChain(Follower follower) {
            return buildPathChain(follower, -1);
        }
    }

    /**
     * Loads a .pp file from Android assets and converts to path segments
     *
     * @param ppFileName Name of .pp file (e.g., "trajectory.pp")
     * @param hardwareMap HardwareMap from OpMode (for asset access)
     * @param alliance Alliance for coordinate mirroring
     * @return List of PathSegment objects
     * @throws IOException If file cannot be read
     * @throws JSONException If file is not valid JSON
     */
    public static List<PathSegment> loadFromAssets(String ppFileName, HardwareMap hardwareMap,
                                                    Alliance alliance)
            throws IOException, JSONException {

        AssetManager assetManager = hardwareMap.appContext.getAssets();
        InputStream is = assetManager.open(ppFileName);

        StringBuilder json = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(is))) {
            String line;
            while ((line = reader.readLine()) != null) {
                json.append(line);
            }
        }

        return parsePathFile(json.toString(), alliance);
    }

    /**
     * Loads a .pp file from TeamCode resources (for testing)
     * NOTE: Use loadFromAssets() for actual OpModes
     *
     * @param ppFileName Name of .pp file (e.g., "trajectory.pp")
     * @param alliance Alliance for coordinate mirroring
     * @return List of PathSegment objects
     * @throws IOException If file cannot be read
     * @throws JSONException If file is not valid JSON
     */
    public static List<PathSegment> loadFromResources(String ppFileName, Alliance alliance)
            throws IOException, JSONException {

        // Read .pp file from resources
        InputStream is = PedroPathLoader.class.getClassLoader()
            .getResourceAsStream(ppFileName);

        if (is == null) {
            throw new IOException("Could not find .pp file: " + ppFileName);
        }

        StringBuilder json = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(is))) {
            String line;
            while ((line = reader.readLine()) != null) {
                json.append(line);
            }
        }

        return parsePathFile(json.toString(), alliance);
    }

    /**
     * Parses a .pp file JSON string into PathSegment objects
     *
     * @param jsonContent JSON content of .pp file
     * @param alliance Alliance for coordinate mirroring
     * @return List of PathSegment objects
     * @throws JSONException If JSON is malformed
     */
    public static List<PathSegment> parsePathFile(String jsonContent, Alliance alliance)
            throws JSONException {

        JSONObject root = new JSONObject(jsonContent);
        JSONObject startPointJson = root.getJSONObject("startPoint");
        JSONArray linesJson = root.getJSONArray("lines");

        List<PathSegment> segments = new ArrayList<>();

        // Parse start point
        Pose currentPose = parsePose(startPointJson, alliance);

        // Parse each path segment
        for (int i = 0; i < linesJson.length(); i++) {
            JSONObject lineJson = linesJson.getJSONObject(i);

            String name = lineJson.optString("name", "Path " + (i + 1));
            JSONObject endPointJson = lineJson.getJSONObject("endPoint");
            JSONArray controlPointsJson = lineJson.optJSONArray("controlPoints");
            boolean isReverse = endPointJson.optBoolean("reverse", false);
            String headingMode = endPointJson.optString("heading", "linear");

            Pose endPose = parsePose(endPointJson, alliance);
            List<Pose> controlPoints = new ArrayList<>();

            if (controlPointsJson != null && controlPointsJson.length() > 0) {
                for (int j = 0; j < controlPointsJson.length(); j++) {
                    JSONObject cpJson = controlPointsJson.getJSONObject(j);
                    double cpX = cpJson.getDouble("x");
                    double cpY = cpJson.getDouble("y");
                    Pose cp = mirrorPoseForAlliance(new Pose(cpX, cpY, 0), alliance);
                    controlPoints.add(cp);
                }
            }

            PathSegment segment = new PathSegment(name, currentPose, endPose,
                                                  controlPoints, isReverse, headingMode);
            segments.add(segment);

            // Next segment starts where this one ended
            currentPose = endPose;
        }

        return segments;
    }

    /**
     * Parses a pose from .pp JSON format
     */
    private static Pose parsePose(JSONObject json, Alliance alliance) {
        double x = json.getDouble("x");
        double y = json.getDouble("y");

        // Handle heading - can be specified as "degrees", "startDeg", or "endDeg"
        double headingDeg = 0;
        if (json.has("degrees")) {
            headingDeg = json.getDouble("degrees");
        } else if (json.has("endDeg")) {
            headingDeg = json.getDouble("endDeg");
        } else if (json.has("startDeg")) {
            headingDeg = json.getDouble("startDeg");
        }

        double headingRad = Math.toRadians(headingDeg);
        Pose pose = new Pose(x, y, headingRad);

        return mirrorPoseForAlliance(pose, alliance);
    }

    /**
     * Mirrors a pose for red alliance (same logic as AutoField.poseForAlliance)
     */
    private static Pose mirrorPoseForAlliance(Pose bluePose, Alliance alliance) {
        if (alliance == Alliance.RED) {
            double fieldWidth = FieldConstants.FIELD_WIDTH_INCHES;
            double mirroredX = fieldWidth - bluePose.getX();
            double mirroredHeading = Math.PI - bluePose.getHeading();
            return new Pose(mirroredX, bluePose.getY(), mirroredHeading);
        }
        return bluePose;
    }
}
