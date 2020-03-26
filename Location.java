package org.firstinspires.ftc.teamcode;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
public class Location {


    private static double locationX; //cm
    private static double locationY; //cm
    private static double orientation; //degrees

    public Location(double x, double y, double a){
        locationX = x;
        locationY = y;
        orientation = MathFunctions.angleWrap(a);
    }

    public Location(Location loc)
    {
        locationX = loc.getX();
        locationY = loc.getY();
        orientation = loc.getAngle();
    }

    public void update(double x, double y, double a){
        locationX = x;
        locationY = y;
        orientation = MathFunctions.angleWrap(a);

    }

    public void update(Location loc){
        locationX = loc.getX();
        locationY = loc.getY();
        orientation = MathFunctions.angleWrap(loc.getAngle());
    }

    public void updateDelta(double dx, double dy, double da){
        locationX += dx;
        locationY += dy;
        orientation += da;
        orientation = MathFunctions.angleWrap(orientation);
    }

    public void updateDeltaRobotCentric(double dx, double dy, double da) {

        double distTraveled = Math.sqrt(Math.pow(dy, 2) + Math.pow(dx, 2));
        double averageAngle = orientation + (da/2);
        double fieldDeltaX = distTraveled * Math.sin(averageAngle + (Math.PI / 2));
        double fieldDeltaY = distTraveled * Math.cos(averageAngle + (Math.PI / 2));

        locationX += fieldDeltaX;
        locationY += fieldDeltaY;
        orientation += da;
        orientation = MathFunctions.angleWrap(orientation);
    }

    public double getX(){
        return locationX;
    }

    public double getY(){
        return locationY;
    }

    public double getAngle(){
        return orientation;
    }

}
