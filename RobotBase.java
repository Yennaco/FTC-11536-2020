package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.NoRouteToHostException;
import java.util.ArrayList;

/**
 * @author Zack Horton
 * If you're reading this that means you're cool, I like you! I am Zack Horton from team 11536 and I first made this class in 2020 during my ISP (Independent Senior Project)
 * Well really before my ISP because #coronacation - this comment probably didn't age well
 * Anyways, the point of this class was to be a jumping off point for teams in the future to use to learn about coding the robot
 * As well as hopefully to be used (or to use parts of this) in comp
 * Now, if you know who I am, that's awesome! Reach out to me, I miss you :(
 * If you don't know who I am, that's even cooler! That means this lived on much longer than I expected or that it got shared with some people from another team or something
 * So if you do see this and don't know me, I'd love to hear from you, my instagram is probably still 'zack_horton' give me a follow and DM me about how the season is going
 * Ok, its 1 AM while I'm writing this so I should probably go to bed but before I do I want to start something. If you're working on this, add your name to the author list so other people know how cool you are
 * @version 1.0
 * @since 1.0
 */
public abstract class RobotBase extends OpMode{


    public static ElapsedTime time = new ElapsedTime();
    //Motors
    private DcMotor motorFL = null;
    private DcMotor motorBL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBR = null;

    //Encoders
    private double leftEncoder;
    private double rightEncoder;
    private double centerEncoder;

    //IMU
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    public double imuRoll;
    public double oldImuRoll;

    //Odometry and location objects which I created
    public static MecanumRobot robot;

    public ArrayList<Location> TargetPoints;

    public void initTime() {
        time.reset();
    }

    //May want to set zero power behavior to brake, not quite sure
    /**
     * Simple method to initialize the motors to how they will likely want to be
     */
    public void initMotors(){
        telemetry.addData("Initializing Motors", "Started");
        telemetry.update();

        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Initializing Motors", "Done");
        telemetry.update();
    }

    /**
     * Simple method to initialize the imu. Most of this stuff is taken directly from the imu sample op mode
     */
    public void initIMU(){
        telemetry.addData("Initializing IMU", "Started");
        telemetry.update();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Initializing IMU", "Done");
        telemetry.update();
    }

    public void initTargets(){
        TargetPoints = new ArrayList<Location>();
    }

    public void drive()
    {
        motorFL.setPower(robot.FLPower);
        motorFR.setPower(robot.FRPower);
        motorBL.setPower(robot.BLPower);
        motorBR.setPower(robot.BRPower);
    }

    /**
     * Method for updating the imu values and returns the current yaw orientation of the robot
     * @return yaw orientation based on imu
     */
    public double imuValues(){
        oldImuRoll = imuRoll;
        imuRoll = imu.getAngularOrientation().firstAngle;
        return imuRoll;
    }

    /**
     * Simple method which updates the odometry and location objects
     */
    public void runOdometry(){
        Odometry.update(motorFL.getCurrentPosition(), motorFR.getCurrentPosition(), motorBL.getCurrentPosition());
    }

    /**
     * Simple method to add a given location to the array list of locations for the robot to follow
     * @param loc is the location you want to add to the list
     */
    public void addTargetPoint(Location loc){
        TargetPoints.add(loc);
    }

    /**
     * Simple method to add a given location based on the x and y coordinates to the array list of locations for the robot to follow
     * @param x is the x coordinate of the location to be added
     * @param y is the y coordinate of the location to be added
     */
    public void addTargetPoint(double x, double y){
        TargetPoints.add(new Location(x, y, 0));
    }

    //****************DRIVE FUNCTIONS********************

    /**
     * Basic drive method to drive with forward, strafe, and turn vectors. They are to be between -1 and 1 to easily be used with a gamepad
     * @param forwardVector the vector to move the robot forwards and backwards
     * @param strafeVector the vector to move the robot left and right
     * @param turnVector the vector to turn the robot
     */
    public void driveWithVectors(double forwardVector, double strafeVector, double turnVector) {
        double driveVector = Math.sqrt(Math.pow(forwardVector, 2) + Math.pow(strafeVector, 2));
        double driveAngle = Math.tan(forwardVector/strafeVector);

        RobotBase.robot.FLPower = MathFunctions.clip((driveVector * Math.sin(driveAngle) + turnVector), 1, -1);
        RobotBase.robot.FRPower = MathFunctions.clip((driveVector * Math.cos(driveAngle) - turnVector), 1, -1);
        RobotBase.robot.BLPower = MathFunctions.clip((driveVector * Math.cos(driveAngle) + turnVector), 1, -1);
        RobotBase.robot.BRPower = MathFunctions.clip((driveVector * Math.sin(driveAngle) - turnVector), 1, -1);
        drive();
    }

    /**
     * PID based method to turn the robot to a specified angle
     * @param targetAngle is the target angle
     * @param error is the error with which the angle must be within for a small amount of time to stop the robot
     * @param timeout is the time at which the robot will stop no matter what
     */
    public void turnToAngle(double targetAngle, double error, double timeout) {

        PIDFunctions anglePID = new PIDFunctions(RobotBase.robot.locationA, targetAngle, 0.001, 0.00001, -0.0001, error, 5); //The number 5 is the amount of cycles the angle must be within the error for it to stop. This may need to be adjusted
        double startTime = time.seconds();

        while((anglePID.output() > 0) && (time.seconds() - startTime > timeout))
        {
            anglePID.setCurrent(RobotBase.robot.locationA);
            driveWithVectors(0, 0, anglePID.output());
        }

        driveWithVectors(0,0,0);
    }

    /**
     * PID Based method to drive to to a specified point without turning
     * @param targetX target point x coordinate
     * @param targetY target point y coordinate
     * @param error error with in what the robot will stop
     * @param timeout time in which the robot will stop what it is doing
     */
    public void simpleDriveToPosition(double targetX, double targetY, double error, double timeout) {
        PIDFunctions xPID = new PIDFunctions(RobotBase.robot.locationX, targetX, 0.001, 0.00001, -0.0001, error, 5); //The number 5 is the amount of cycles the position must be within the error for it to stop. This may need to be adjusted
        PIDFunctions yPID = new PIDFunctions(RobotBase.robot.locationY, targetY, 0.001, 0.00001, -0.0001, error, 5);//The number 5 is the amount of cycles the position must be within the error for it to stop. This may need to be adjusted
        double startTime = time.seconds();

        while(((xPID.output() > 0) || (yPID.output() > 0)) && (time.seconds() - startTime > timeout))
        {
            xPID.setCurrent(RobotBase.robot.locationX);
            yPID.setCurrent(RobotBase.robot.locationY);
            driveWithVectors(yPID.output(), xPID.output(), 0);
        }
        driveWithVectors(0,0,0);
    }

    /**
     * PID Based method to drive to to a specified point with turning
     * @param targetX target point x coordinate
     * @param targetY target point y coordinate
     * @param targetA target angle
     * @param errorPos error of the position with in what the robot will stop
     * @param errorAngle error of the angle with in what the robot will stop
     * @param timeout time in which the robot will stop what it is doing
     */
    public void pathFollowToPosition(double targetX, double targetY, double targetA, double errorPos, double errorAngle, double timeout){
        PIDFunctions xPID = new PIDFunctions(RobotBase.robot.locationX, targetX, 0.001, 0.00001, -0.0001, errorPos, 5); //The number 5 is the amount of cycles the position must be within the error for it to stop. This may need to be adjusted
        PIDFunctions yPID = new PIDFunctions(RobotBase.robot.locationY, targetY, 0.001, 0.00001, -0.0001, errorPos, 5); //The number 5 is the amount of cycles the position must be within the error for it to stop. This may need to be adjusted
        PIDFunctions anglePID = new PIDFunctions(RobotBase.robot.locationA, targetA, 0.001, 0.00001, -0.0001, errorAngle, 5); //The number 5 is the amount of cycles the angle must be within the error for it to stop. This may need to be adjusted
        double startTime = time.seconds();

        while(((xPID.output() > 0) || (yPID.output() > 0) || (anglePID.output() > 0)) && (time.seconds() - startTime > timeout))
        {
            xPID.setCurrent(RobotBase.robot.locationX);
            yPID.setCurrent(RobotBase.robot.locationY);
            anglePID.setCurrent(RobotBase.robot.locationA);
            driveWithVectors(yPID.output(), xPID.output(), anglePID.output());
        }
        driveWithVectors(0,0,0);
    }

    /**
     * Quite complex method to line follow based on given input points used to construct a series of lines which the robot will follow
     * Much of this code is based on the "Pure Pursuit" code created by team 11115 Gluten Free (The FTC wizards may they go down in history)
     * I'm not exactly sure if this is what they did but it is what I took from their explanation
     * I didn't copy their code tho, I tried to make it for myself based on their description
     * If you're reading this (hi Sam) I would reccomend reading through this method cause I'm pretty proud of it (so long as it works)
     * I would also highly reccomend watching the video series made by GF to explain pure pursuit.
     * Here is the first video: https://youtu.be/3l7ZNJ21wMo
     * @param circleRadius the radius of the circle the robot draws around itself to find the line to follow
     * @param timeout the time at which the robot will stop
     */
    public void lineFollow(double circleRadius, double timeout) {

        //End tolerance is how far away from the end point the robot will start at
        double endTolerance = 4; //measured in ...

        //Location in path tells which line in the path that the robot is currently following
        int locationInPath = 1;

        //If there are less than 2 points in the path, a line can't be made so the method cuts short and there is some telemetry that says so
        if (TargetPoints.size() < 2){
            telemetry.addData("Not enough target points : ", TargetPoints.size());
            telemetry.update();
            return;
        }

        //Driving loop
        while(((Math.abs(RobotBase.robot.locationX - TargetPoints.get(TargetPoints.size()-1).getX()) > endTolerance) || (Math.abs(RobotBase.robot.locationY - TargetPoints.get(TargetPoints.size()-1).getY()) > endTolerance)) && (locationInPath < TargetPoints.size())) {
            runOdometry();

            //The two points that are chosen based on the location in path are defined using this code
            Location start = new Location(TargetPoints.get(locationInPath-1));
            Location end = new Location(TargetPoints.get(locationInPath));

            //If the two x-coordinates of the target points are equal to each other, we add a small value to one of them to avoid a vertical line and dividing by zero
            if(start.getX() == end.getX())
            {
                end.updateDelta(.0001, 0, 0);
            }

            //If the end point is in the radius around the robot and the location in path is not the maximum based on the size of the path, the location in the path is added to
            //May want this to go first....? If there are issues with corner rounding, this is one thing to try
            if((MathFunctions.distance(end.getX(), end.getY(), RobotBase.robot.locationX, RobotBase.robot.locationY) < circleRadius) && (locationInPath < TargetPoints.size())){locationInPath++;}

            //Slope and y-intercept of current line to follow calculated based on the current target points
            double m = (end.getY() - start.getY()) / (end.getX() - start.getX());
            double b = start.getY() - (m * start.getX());

            //This is not necessary but it is just done for ease and k and h are chosen because of the labeling convention for the equation of a circle
            double h = RobotBase.robot.locationX;
            double k = RobotBase.robot.locationY;

            //The A B and C constants for a quadratic equation are found which are the solutions to the intersections of the circle and line
            double quadA = (1 + Math.pow(m, 2));
            double quadB = ((2 * m * b) + (2 * m * k) + (2 * h));
            double quadC = (Math.pow(b, 2) + Math.pow(k, 2) - Math.pow(circleRadius, 2) - (2 * b * k));

            //Initialize the 2 possible x solutions for the circle and line intersections to zero
            double xSolution1 = 0;
            double xSolution2 = 0;

            //Initialize the best solution to the equations to zero (best being closest to the end point)
            double xSolution = 0;

            if (MathFunctions.quadraticChecker(quadA, quadB, quadC)) //Checks to make sure that the circle intersects with the line
            {
                //Sets the two possible solutions to the solution of the quadratic formula to the x solutions
                xSolution1 = MathFunctions.quadratic1(quadA, quadB, quadC);
                xSolution2 = MathFunctions.quadratic2(quadA, quadB, quadC);

                //Chooses the best solution to be the actual solution. Best being closest to the end point
                if (start.getX() < end.getX()) {
                    xSolution = Math.max(xSolution1, xSolution2);
                } else {
                    xSolution = Math.min(xSolution1, xSolution2);
                }
            } else        //If the circle doesn't intersect with the line, go straight towards the line perpendicularly to it
            {
                double b2 = RobotBase.robot.locationY + (RobotBase.robot.locationX / m);

                xSolution = (b2 - b) / (m - (1 / m));
            }

            //If the end point is within the circle radius then the target point is the end point
            if(MathFunctions.distance(end.getX(), end.getY(), RobotBase.robot.locationX, RobotBase.robot.locationY) < circleRadius)
            {
                xSolution = end.getX();
            }

            //Find the y coordinate of the solution based on the x coordinate and the line
            double ySolution = (m * xSolution) + b;

            //Drive towards the best solution with the vector of the distance from the solution divided by the radius. This makes sure the vector doesn't exceed 1 but it may if the line is not within the radius but that is dealt with by the driving function which clips the value to be within 1 and -1
            driveWithVectors((robot.locationX - xSolution)/circleRadius, (robot.locationY - ySolution)/circleRadius, 0);
        }

        driveWithVectors(0,0,0);

    }



}
