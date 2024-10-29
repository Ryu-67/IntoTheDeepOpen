package org.firstinspires.ftc.teamcode.rr;

import kotlin.Pair;

public class Utils {

    public static boolean valInThresh(double val, double val2, double tol) {
        return Math.abs(val - val2) > tol;
    }


    public static Pair<Double, Boolean> increment(boolean dflag, boolean uflag, double input, double lLim, double uLim, boolean locked) {
        double augmented = 0;

        if (dflag && !locked && input > lLim) {
            augmented = input - 1;
            locked = true;
        } else if (uflag && !locked && input < uLim) {
            augmented = input + 1;
            locked = true;
        } else if (!dflag && !uflag) {
            locked = false;
        }

        return new Pair<>(augmented, locked);
    }
}
