package org.firstinspires.ftc.teamcode;

public class MecanumRobot {

    public static double FLPower;
    public static double FRPower;
    public static double BLPower;
    public static double BRPower;

    public static double locationX;
    public static double locationY;
    public static double locationA;

    public static double velocityX;
    public static double velocityY;
    public static double velocityA;

    public static double accelerationX;
    public static double accelerationY;
    public static double accelerationA;

    public static double leftEncoder;
    public static double rightEncoder;
    public static double middleEncoder;

    public MecanumRobot()
    {
        FLPower = 0;
        FRPower = 0;
        BLPower = 0;
        BRPower = 0;

        locationX = 0;
        locationY = 0;
        locationA = 0;

        leftEncoder = 0;
        rightEncoder = 0;
        middleEncoder = 0;
    }

    public void setLocation(double x, double y, double a)
    {
        locationX = x;
        locationY = y;
        locationA = MathFunctions.angleWrap(a);

        velocityX = 0;
        velocityY = 0;
        velocityA = 0;

        accelerationX = 0;
        accelerationY = 0;
        accelerationA = 0;
    }

    public void updateLocation(double dx, double dy, double da)
    {
        double lastLX = locationX;
        double lastLY = locationY;
        double lastLA = locationA;

        double lastVX = velocityX;
        double lastVY = velocityY;
        double lastVA = velocityA;

        locationX += dx;
        locationY += dy;
        locationA += da;
        locationA = MathFunctions.angleWrap(locationA);

        velocityX = (locationX-lastLX)/RobotBase.time.seconds();
        velocityY = (locationY-lastLY)/RobotBase.time.seconds();
        velocityA = (locationA-lastLA)/RobotBase.time.seconds();

        accelerationX = (velocityX-lastVX)/RobotBase.time.seconds();
        accelerationY = (velocityY-lastVY)/RobotBase.time.seconds();
        accelerationA = (velocityA-lastVA)/RobotBase.time.seconds();
    }

    public void updateLocationRobotCentric(double dx, double dy, double da)
    {
        double lastLX = locationX;
        double lastLY = locationY;
        double lastLA = locationA;

        double lastVX = velocityX;
        double lastVY = velocityY;
        double lastVA = velocityA;

        double distTraveled = Math.sqrt(Math.pow(dy, 2) + Math.pow(dx, 2));
        double averageAngle = locationA + (da/2);
        double fieldDeltaX = distTraveled * Math.sin(averageAngle + (Math.PI / 2));
        double fieldDeltaY = distTraveled * Math.cos(averageAngle + (Math.PI / 2));

        locationX += fieldDeltaX;
        locationY += fieldDeltaY;
        locationA += da;
        locationA = MathFunctions.angleWrap(locationA);

        velocityX = (locationX-lastLX)/RobotBase.time.seconds();
        velocityY = (locationY-lastLY)/RobotBase.time.seconds();
        velocityA = (locationA-lastLA)/RobotBase.time.seconds();

        accelerationX = (velocityX-lastVX)/RobotBase.time.seconds();
        accelerationY = (velocityY-lastVY)/RobotBase.time.seconds();
        accelerationA = (velocityA-lastVA)/RobotBase.time.seconds();
    }

}
