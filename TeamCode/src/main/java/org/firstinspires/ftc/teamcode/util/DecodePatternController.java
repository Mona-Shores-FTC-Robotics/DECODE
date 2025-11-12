package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.util.DecodePatterns;
import org.firstinspires.ftc.teamcode.util.ArtifactColor;

/**
 * Small state holder that rotates through the predefined decode patterns.
 */
public final class DecodePatternController {

    private int index = -1;
    private ArtifactColor[] current = new ArtifactColor[0];

    public ArtifactColor[] current() {
        return current;
    }

    public ArtifactColor[] cycleNext() {
        if (DecodePatterns.PATTERNS.length == 0) {
            current = new ArtifactColor[0];
            index = -1;
            return current;
        }
        index = (index + 1) % DecodePatterns.PATTERNS.length;
        current = DecodePatterns.PATTERNS[index];
        return current;
    }

    public ArtifactColor[] clear() {
        index = -1;
        current = new ArtifactColor[0];
        return current;
    }
}
