package frc.robot;

/**
 * Utilility functions.
 */
public final class Utils {

    /* Check doubles are equal to within epsilon preceision */
    public static boolean isDoubleEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

}