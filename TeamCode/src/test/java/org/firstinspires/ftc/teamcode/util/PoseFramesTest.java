package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.geometry.Pose;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class PoseFramesTest {

    private static final double TOLERANCE = 1e-6;

    @Test
    public void ftcToPedroRoundTripPreservesPose() {
        Pose ftc = new Pose(10.0, -20.0, Math.toRadians(45.0));
        Pose pedro = PoseFrames.ftcToPedro(ftc);
        Pose ftcBack = PoseFrames.pedroToFtc(pedro);

        assertEquals(ftc.getX(), ftcBack.getX(), TOLERANCE);
        assertEquals(ftc.getY(), ftcBack.getY(), TOLERANCE);
        assertEquals(ftc.getHeading(), ftcBack.getHeading(), TOLERANCE);
    }

    @Test
    public void pedroToFtcRoundTripPreservesPose() {
        Pose pedro = new Pose(90.0, 60.0, Math.toRadians(135.0));
        Pose ftc = PoseFrames.pedroToFtc(pedro);
        Pose pedroBack = PoseFrames.ftcToPedro(ftc);

        assertEquals(pedro.getX(), pedroBack.getX(), TOLERANCE);
        assertEquals(pedro.getY(), pedroBack.getY(), TOLERANCE);
        assertEquals(pedro.getHeading(), pedroBack.getHeading(), TOLERANCE);
    }

    @Test
    public void fieldCenterIsInvariant() {
        Pose center = new Pose(0.0, 0.0, Math.toRadians(90.0));
        Pose pedro = PoseFrames.ftcToPedro(center);
        Pose ftcBack = PoseFrames.pedroToFtc(pedro);

        assertEquals(center.getX(), ftcBack.getX(), TOLERANCE);
        assertEquals(center.getY(), ftcBack.getY(), TOLERANCE);
        assertEquals(center.getHeading(), ftcBack.getHeading(), TOLERANCE);
    }
}
