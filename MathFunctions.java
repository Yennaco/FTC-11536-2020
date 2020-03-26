package org.firstinspires.ftc.teamcode;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
public class MathFunctions {

    /**
     * Method which takes in the current angle you are facing in radians and makes sure it is between -PI and PI
     * @param currentAngle Your supplied heading between -2PI and 2PI
     * @return The wrapped heading between -PI and PI
     */
    public static double angleWrap(double currentAngle){
        if(currentAngle < -Math.PI)
        {
            return (Math.PI * 2) + currentAngle;
        }
        else if (currentAngle > Math.PI)
        {
            return (-Math.PI * 2) + currentAngle;
        }
        else
        {
            return currentAngle;
        }
    }

    /**
     * Method to find the shortest angle between your current angle and your target angle
     * @param currentAngle Supplied current angle of bot
     * @param targetAngle Target angle
     * @return The shortest difference between current angle and target angle
     */
    public static double smallestAngleDiff(double currentAngle, double targetAngle){
        targetAngle -= currentAngle;
        return angleWrap(targetAngle) + currentAngle;
    }

    public static double distance(double x1, double y1, double x2, double y2){
        return Math.sqrt( Math.pow((x2-x1), 2) + Math.pow((y2-y1), 2));
    }
    public static double clip(double val, double high, double low) {
        if (val > high)
        {
            val = high;
        }
        if (val < low)
        {
            val = low;
        }
        return val;
    }

    public static double quadratic1(double a, double b, double c){
        if (Math.sqrt(Math.pow(b, 2) - (4 * a * c)) < 0) //Checks to make sure there are solutions
        {
            return 0;
        }
        return (-b + Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    public static double quadratic2(double a, double b, double c){
        if (Math.sqrt(Math.pow(b, 2) - (4 * a * c)) < 0) //Checks to make sure there are solutions
        {
            return 0;
        }
        return (-b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
    }

    public static boolean quadraticChecker(double a, double b, double c){
        return Math.sqrt(Math.pow(b, 2) - (4 * a * c)) >= 0;
    }

}
