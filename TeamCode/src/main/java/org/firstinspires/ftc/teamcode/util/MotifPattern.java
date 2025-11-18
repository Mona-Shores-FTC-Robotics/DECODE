package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.ArtifactColor;
import org.firstinspires.ftc.teamcode.util.FieldConstants;

/**
 * Represents the randomized Decode motif pattern identified by AprilTags 21-23.
 */
public enum MotifPattern {
    GPP(new ArtifactColor[]{ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.PURPLE}, FieldConstants.DECODE_PATTERN_GREEN_PURPLE_PURPLE_ID),
    PGP(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.PURPLE}, FieldConstants.DECODE_PATTERN_PURPLE_GREEN_PURPLE_ID),
    PPG(new ArtifactColor[]{ArtifactColor.PURPLE, ArtifactColor.PURPLE, ArtifactColor.GREEN}, FieldConstants.DECODE_PATTERN_PURPLE_PURPLE_GREEN_ID),
    UNKNOWN(new ArtifactColor[]{ArtifactColor.NONE, ArtifactColor.NONE, ArtifactColor.NONE}, -1);

    private final ArtifactColor[] laneColors;
    private final int tagId;

    MotifPattern(ArtifactColor[] laneColors, int tagId) {
        this.laneColors = laneColors;
        this.tagId = tagId;
    }

    public ArtifactColor[] getLaneColors() {
        return laneColors.clone();
    }

    public int getTagId() {
        return tagId;
    }

    public static MotifPattern fromTagId(int tagId) {
        for (MotifPattern pattern : values()) {
            if (pattern.tagId == tagId) {
                return pattern;
            }
        }
        return UNKNOWN;
    }
}
