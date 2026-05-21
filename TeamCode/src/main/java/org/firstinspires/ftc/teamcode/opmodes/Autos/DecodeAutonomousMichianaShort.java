package org.firstinspires.ftc.teamcode.opmodes.Autos;

import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.Autos.Commands.MichianaShortCommand;
import org.firstinspires.ftc.teamcode.util.AutoField;
import org.firstinspires.ftc.teamcode.util.PpPathLoader;
import org.firstinspires.ftc.teamcode.util.RobotState;

/**
 * Autonomous variant for the Michiana regional event. Unlike the other autos,
 * this one loads its path from a {@code .pp} file (either from the robot's
 * SD card for hot-reload, or from the bundled APK assets as a fallback) so the
 * routine can be re-tuned without rebuilding the code.
 */
@Autonomous(name = "Michiana Short", group = "Auto")
public class DecodeAutonomousMichianaShort extends BaseAutonomousOpMode {

    /**
     * Name of the .pp file to load. We check {@code /sdcard/FIRST/<PP_FILE_NAME>} first
     * (hot-reload via USB), then fall back to the {@code TeamCode/src/main/assets/} bundle.
     */
    private static final String PP_FILE_NAME = "MichianaClose.pp";

    // Parsed .pp loaded once in onInit() — source of truth for paths, start pose,
    // expected position. sdcard takes priority for hot-reload; asset is the
    // committed fallback that ships with the APK.
    private PpPathLoader.ParsedPp parsedPp;
    private String pathSource = "uninitialized";

    // -----------------------------------------------------------------------
    // Hooks
    // -----------------------------------------------------------------------

    @Override
    protected void onInit() {
        parsedPp = PpPathLoader.loadFromSdcardOrNull(PP_FILE_NAME);
        if (parsedPp != null) {
            pathSource = "sdcard:/FIRST/" + PP_FILE_NAME;
        } else {
            try {
                parsedPp = PpPathLoader.loadFromAssets(hardwareMap.appContext, PP_FILE_NAME);
                pathSource = "asset:" + PP_FILE_NAME;
            } catch (Exception e) {
                throw new RuntimeException(
                        "MichianaShort: failed to load " + PP_FILE_NAME
                                + " from sdcard or assets: " + e.getMessage(), e);
            }
        }
        RobotState.packet.put("Auto/PathSource", pathSource);
    }

    @Override
    protected void onStart() {
        telemetry.addData("Path source", pathSource);
    }

    // -----------------------------------------------------------------------
    // Start coordinates — sourced from the parsed .pp file
    // -----------------------------------------------------------------------

    @Override
    protected double getStartX() { return parsedPp.startX; }

    @Override
    protected double getStartY() { return parsedPp.startY; }

    @Override
    protected double getStartHeadingDeg() { return parsedPp.startHeadingDeg; }

    /**
     * Override: Michiana's default pose is alliance-mirrored so the follower is
     * set to the correct side of the field even before vision locks on.
     */
    @Override
    protected Pose getDefaultStartPose() {
        return AutoField.poseForAlliance(parsedPp.startX, parsedPp.startY, parsedPp.startHeadingDeg, activeAlliance);
    }

    // -----------------------------------------------------------------------
    // Routine
    // -----------------------------------------------------------------------

    @Override
    protected Command buildAutoRoutine(Pose startPoseOverride) {
        return MichianaShortCommand.createFromPp(robot, activeAlliance, parsedPp, startPoseOverride);
    }
}
