package org.firstinspires.ftc.teamcode;

/**
 * @author Zack Horton
 * @version 1.0
 * @since 1.0
 */
//@TeleOp(name="TeleOp")
public class TeleOp extends RobotBase {

    @Override
    public void init() {
        initMotors();
        initIMU();
        initTime();
    }

    @Override
    public void loop() {
        runOdometry();

        if(gamepad1.a)
        {
            turnToAngle(0, 2, 3);
            drive();
            telemetry.addData("Turning to", 0);
        }
        else if(gamepad1.b)
        {
            simpleDriveToPosition(0, 0, .05, 4);
            drive();
            telemetry.addData("Distance Error", "origin");
        }
        else if(gamepad1.x)
        {
            pathFollowToPosition(0, 0, 0, .05, 2, 5);
            drive();
            telemetry.addData("Driving to", "origin");
        }
        else{
            driveWithVectors(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }


        telemetry.addData("X", RobotBase.robot.locationX);
        telemetry.addData("Y", RobotBase.robot.locationY);
        telemetry.addData("A", RobotBase.robot.locationA);
        telemetry.update();
    }


}
