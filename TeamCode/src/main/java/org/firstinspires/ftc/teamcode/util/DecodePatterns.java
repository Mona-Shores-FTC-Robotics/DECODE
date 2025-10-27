package org.firstinspires.ftc.teamcode.util;

/**
 * Central location for the Decode randomiser colour sequences.
 * The order matches the official PGG, GPG, and GGP patterns from the game manual.
 */
public final class DecodePatterns {
    private DecodePatterns() { }

    public static final ArtifactColor[][] PATTERNS = new ArtifactColor[][]{
            {ArtifactColor.PURPLE, ArtifactColor.GREEN, ArtifactColor.GREEN},
            {ArtifactColor.GREEN, ArtifactColor.PURPLE, ArtifactColor.GREEN},
            {ArtifactColor.GREEN, ArtifactColor.GREEN, ArtifactColor.PURPLE}
    };
}
