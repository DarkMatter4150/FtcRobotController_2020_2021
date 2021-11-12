package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

@Autonomous(name = "Diff Swerve Test Auto 1", group = "Linear Opmode")

public class TestAutoNoT265 extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this, true);
        robot.initIMU();

        waitForStart();
        long startTime = System.currentTimeMillis();
        long subtractTime = 0;

        robot.driveController.rotateModules(Vector2d.LEFT, false, 2000, this);
        //simple sequence to demonstrate the three main autonomous primitives

        //rotate modules to face to the right
        //robot.driveController.rotateModules(Vector2d.RIGHT, false, 10000, this);

        //drive 20 cm to the right (while facing forward)
        //robot.driveController.drive(Vector2d.FORWARD, 100 , 1, this);
        //robot.driveController.rotateRobot(Angle.RIGHT, this);
        /*
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.LEFT, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.BACKWARD, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.RIGHT, false, 2000, this);
        sleep(500);
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this);
        robot.driveController.drive(Vector2d.FORWARD, 50 , .5, this);
        robot.driveController.drive(Vector2d.UNIT_CIRCLE_60, 50 , .5, this);
        robot.driveController.drive(Vector2d.RIGHT, 50 , .5, this);
        robot.driveController.drive(Vector2d.UNIT_CIRCLE_120, 50 , .5, this);
        robot.driveController.rotateModules(Vector2d.FORWARD, false, 2000, this); */

        //robot.driveController.driveToLocation(0,0,0,1,1);

        //robot.driveController.drive(Vector2d.RIGHT, 200, 1, this);
        //robot.driveController.updateUsingJoysticks(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0), false);
        //robot.driveController.updateAbsRotation(Vector2d.RIGHT.scale(Math.sqrt(2)), new Vector2d(0,0).scale(Math.sqrt(2)), 0.7);

//        robot.driveController.resetDistanceTraveled();
//        while (robot.driveController.getDistanceTraveled() < cmDistance && opModeIsActive()) {
//            //slows down drive power in certain range
//            robot.driveController.updateTracking();
//            robot.driveController.update(direction.normalize(speed), 0);
//
//            telemetry.addData("Driving robot", "");
//            telemetry.addData("Right Orientation", robot.driveController.moduleRight.getCurrentOrientation().getAngle());
//            telemetry.addData("Distance", robot.driveController.moduleRight.getDistanceTraveled());
//            telemetry.addData("Encoder", robot.driveController.moduleRight.motor1.getCurrentPosition());
//            telemetry.update();
//        }
//        robot.driveController.update(Vector2d.ZERO, 0);


        //robot.driveController.moduleRight.updateTarget(Vector2d.BACKWARD, 0);
        //robot.driveController.moduleLeft.updateTarget(Vector2d.BACKWARD, 0);
        //sleep(2000);


        //turn to face robot right
        //robot.driveController.rotateRobot(Angle.RIGHT, this);
        Pose pose = new Pose(1,1,1);

    }
    public static void followCurvePath(Path path, double speed, double pvalue, LinearOpMode linearOpMode,
                                       Robot robot, Telemetry telemetry,
                                       Pose currentPose) {
        Path current_path = path;

        while (linearOpMode.opModeIsActive()) {

            if (!path.isComplete()) {
                //TODO: Update to remove dependencies on SLAM Camera
                    //updateSLAMNav(telemetry, dashboard, startingPose, currentPose);

                robot.updateBulkData();

                // Get our lookahead point
                path.update(currentPose);
                Pose lookahead_pose = path.getFollowPose();

                // Get the distance to our lookahead point
                double distance = Math.hypot(lookahead_pose.x - currentPose.x, lookahead_pose.y - currentPose.y);

                double speedFinal;
                // Find our drive speed based on distance
                if (distance < current_path.getFollowCircle().radius) {
                    speedFinal = Range.clip((distance / 8) + 0.1, 0, 1) * speed;
                } else {
                    speedFinal = speed;
                }

                // Find our turn speed based on angle difference
                double headingg = path.getHeadingGoal(Path.HeadingMethod.CONSTANT_ANGLE, currentPose);
                double turn_speed = Range.clip(Math.abs(currentPose.angle - lookahead_pose.angle) / (Math.PI / 4) + 0.1, 0, 1);

                // Drive towards the lookahead point
                driveUsingPose(lookahead_pose, speedFinal, turn_speed, pvalue, linearOpMode, robot, telemetry, currentPose);
            } else {
                robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);
                break;
            }

        }
        robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);

    }


    public static void driveUsingPose(Pose pose, double drive_speed, double turn_speed, double pvalue, LinearOpMode linearOpMode,
                                             Robot robot, Telemetry telemetry,
                                             Pose currentPose) {

        // Find the angle to the pose
        Vector2d directionPP = new Vector2d((pose.x - currentPose.x), (pose.y - currentPose.y)).normalize(drive_speed);

        // Find movement vector to drive towards that point
        double mvmt_a = -Math.signum(Range.clip((pose.angle - currentPose.angle - (Math.PI / 2)), -1, 1)) * turn_speed;
        mvmt_a = mvmt_a * pvalue;//- pidController(path.getHeadingGoal(path.heading_method,pose), currentPose.angle,.0000000000004,0,0);


        // Update actual motor powers with our movement vector
        robot.updateBulkData();
        //TODO: Fix Feedback to get rid of SLAM
            //updateSLAMNav(telemetry, dashboard, startingPose, currentPose);
        robot.driveController.updateTracking();
        linearOpMode.telemetry.addData("Driving robot", "");
        linearOpMode.telemetry.addData("Current X: ", currentPose.x);
        linearOpMode.telemetry.addData("Current Y: ", currentPose.y);
        linearOpMode.telemetry.addData("Current Theta: ", currentPose.angle);
        linearOpMode.telemetry.update();

        robot.driveController.updatePositionTracking(telemetry); //update position tracking
        robot.driveController.update(directionPP, mvmt_a);

    }

}
