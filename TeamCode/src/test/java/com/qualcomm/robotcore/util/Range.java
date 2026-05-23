package com.qualcomm.robotcore.util;

/** Test stub — mirrors the FTC SDK Range utility used by PoseFusion. */
public class Range {
    public static double clip(double number, double min, double max) {
        if (Double.isNaN(number)) return min;
        return Math.max(min, Math.min(max, number));
    }
}
