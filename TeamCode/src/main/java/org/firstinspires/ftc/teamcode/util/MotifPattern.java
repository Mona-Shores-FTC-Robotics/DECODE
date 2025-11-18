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

    /**
     * Rotates the pattern left by the specified offset to account for motif tail.
     *
     * The motif tail is calculated as (artifactsInRamp % 3), which represents how many
     * artifacts are already in the field ramp as part of an incomplete decode motif.
     *
     * Examples for PPG pattern:
     * - Tail = 0 (0, 3, 6, ... artifacts in ramp): PPG (no rotation)
     * - Tail = 1 (1, 4, 7, ... artifacts in ramp): PGP (rotate left by 1)
     * - Tail = 2 (2, 5, 8, ... artifacts in ramp): GPP (rotate left by 2)
     *
     * @param motifTailOffset The offset (0-2) calculated from artifactsInRamp % 3
     * @return A new array with the pattern rotated left by the offset
     */
    public ArtifactColor[] getRotatedPattern(int motifTailOffset) {
        if (this == UNKNOWN) {
            return laneColors.clone();
        }

        int offset = motifTailOffset % 3;
        if (offset < 0) {
            offset += 3;
        }

        ArtifactColor[] rotated = new ArtifactColor[3];
        for (int i = 0; i < 3; i++) {
            rotated[i] = laneColors[(i + offset) % 3];
        }
        return rotated;
    }
}
