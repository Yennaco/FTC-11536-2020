package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
public class Odometry {

    //Important constants to be used in calculations
    final static double LEFT_ENCODER_DIST = -20; //20 cm left of com
    final static double RIGHT_ENCODER_DIST = 20; //20 cm right of com
    final static double CENTER_ENCODER_DIST = -20; //20 cm behind com
    final static double ENCODER_DISTANCE = RIGHT_ENCODER_DIST - LEFT_ENCODER_DIST;
    final static double WHEEL_DIAMETER = 6; //REV 60 mm omni wheel
    final static double TICKS_PER_REV = 8192; //REV through bore encoder
    final static double GEAR_RATIO = 1; //Wheel to encoder

    //Primitive method with the goal of updating the position based on encoder values. This is the basis of odometry
    public static void update(double left, double right, double center){
        //Raw encoder values imputed into the method
        RobotBase.robot.leftEncoder = left;
        RobotBase.robot.rightEncoder = right;
        RobotBase.robot.middleEncoder = center;

        double lengthLeft = RobotBase.robot.leftEncoder * GEAR_RATIO * WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
        double lengthRight = RobotBase.robot.rightEncoder * GEAR_RATIO * WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;
        double lengthCenter = RobotBase.robot.middleEncoder * GEAR_RATIO * WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;

        //The values used to calculate the change in position
        double radiusLeft = 0;
        double radiusRight = 0;
        double radiusCenter = 0;
        double angle = 0;

        //The actual change in position values
        double dY = 0;
        double dX = 0;
        double dA = 0;

        if (RobotBase.robot.leftEncoder < RobotBase.robot.rightEncoder)     //if the left encoder value is less than the right, we do these calculations to find the chang in x, y, and angle
        {
            //Calculation to find the angle swept out by the arc
            angle = (lengthRight - lengthLeft) / (2 * RIGHT_ENCODER_DIST);

            radiusLeft = lengthLeft / angle;
            radiusCenter = radiusLeft + RIGHT_ENCODER_DIST;

            //This is the calculated distance moved by the center encoder if it does not strafe
            double expectedCenterEncoderMovement = CENTER_ENCODER_DIST * dA;

            //This is the distance the center encoder actually traveled minus the amount expected for the robot to not have strafed to find how much it actually strafed
            double adjustedCenterEncoder = lengthCenter - expectedCenterEncoderMovement;

            //Calculations to find the x and y displacement based on the angle swept out as well as the center radius
            dX = (radiusCenter * (1-Math.cos(angle))) + (adjustedCenterEncoder * Math.cos(angle / 2));
            dY = (radiusCenter * Math.sin(angle)) + (adjustedCenterEncoder * Math.sin(angle / 2));
            dA = angle/2; //May not be divided by 2
        }
        else if (RobotBase.robot.rightEncoder < RobotBase.robot.leftEncoder)    //if the right encoder value is less than the left, we do these calculations to find the chang in x, y, and angle
        {
            //Calculation to find the angle swept out by the arc
            angle = -(lengthLeft - lengthRight) / (2 * RIGHT_ENCODER_DIST);

            radiusRight = lengthRight / angle;
            radiusCenter = radiusRight + RIGHT_ENCODER_DIST;

            //This is the calculated distance moved by the center encoder if it does not strafe
            double expectedCenterEncoderMovement = CENTER_ENCODER_DIST * dA;

            //This is the distance the center encoder actually traveled minus the amount expected for the robot to not have strafed to find how much it actually strafed
            double adjustedCenterEncoder = lengthCenter - expectedCenterEncoderMovement;

            //Calculations to find the x and y displacement based on the angle swept out as well as the center radius
            dX = (radiusCenter * (1-Math.cos(angle))) + (adjustedCenterEncoder * Math.cos(angle / 2));
            dY = (radiusCenter * Math.sin(angle)) + (adjustedCenterEncoder * Math.sin(angle / 2));
            dA = angle/2; //May not be divided by 2
        }
        else                                    //if the encoder values are the exact same, the angle doesn't change so the calculation is very simple
        {
            dX = lengthCenter;
            dY = (lengthRight+lengthLeft)/2;
            dA = 0;
        }


        RobotBase.robot.updateLocationRobotCentric(dX, dY, dA);
    }

}
