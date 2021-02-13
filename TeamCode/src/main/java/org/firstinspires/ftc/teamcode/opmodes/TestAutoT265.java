package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.RobotUtil;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.opmodes.TestCameraT265.slamra;

@Autonomous(name = "T265 Test Auto", group = "Linear Opmode")

public class TestAutoT265 extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double startingX = 0;
    double startingY = 0;
    double startingTheta = 0;

    double currentX = 0;
    double currentY = 0;
    double currentTheta = 0;
    Pose currentPose = new Pose(startingX, startingY, startingTheta);
    public Path current_path;
    @Override
    public void runOpMode() throws InterruptedException {

        //Init
        robot = new Robot(this, true);
        robot.initIMU();


        //Beginning
        waitForStart();

        //Actual OpMode
        driveToPosition(30,30,.5,true,false,2,this);


        while (opModeIsActive()) {
            updateSLAMNav();
            robot.updateBulkData();
        }



    }


    public void drive(Vector2d direction, double cmDistance, double speed, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
        cmDistance = cmDistance; //BAD :(
        double initalSpeed = speed;

        alignModules = true;

        //turns modules to correct positions for straight driving
        if (alignModules)
            robot.driveController.rotateModules(direction, false, robot.driveController.DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) robot.driveController.setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode

        robot.driveController.resetDistanceTraveled();
        robot.driveController.updateTracking(); //ADDED

        while (robot.driveController.getDistanceTraveled() < cmDistance && /*System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && */linearOpMode.opModeIsActive()) {
            robot.updateBulkData();
            //updateSLAMNav();
            robot.driveController.updateTracking();
            //slows down drive power in certain range
            if (cmDistance - robot.driveController.getDistanceTraveled() < robot.driveController.START_DRIVE_SLOWDOWN_AT_CM) {
                speed = RobotUtil.scaleVal(cmDistance - robot.driveController.getDistanceTraveled(), 0, robot.driveController.START_DRIVE_SLOWDOWN_AT_CM, 0.3, initalSpeed);
                linearOpMode.telemetry.addData("speed: ", speed);
            }
            //updateTracking(); //WAS MOVED ABOVE
            robot.driveController.update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING

            linearOpMode.telemetry.addData("Driving robot", "");
            linearOpMode.telemetry.addData("Distance Traveled", robot.driveController.getDistanceTraveled());
            linearOpMode.telemetry.addData("CM Distance", cmDistance);
            //linearOpMode.telemetry.addData("SLAM Pos", translationSLAM);
            //linearOpMode.telemetry.addData("SLAM Rot", rotationSLAM);
            linearOpMode.telemetry.update();

            robot.driveController.updatePositionTracking(telemetry); //update position tracking
        }
        robot.driveController.update(Vector2d.ZERO, 0);
        robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    public void driveToPosition(double X, double Y, double speed, boolean fixModules, boolean alignModules, double maxError, LinearOpMode linearOpMode) {
        updateSLAMNav();
        double denominatorMath = ((X-currentX)*(X-currentX)+(Y-currentY)*(Y-currentY));
        double yPos = (Y-currentY)/Math.sqrt(denominatorMath);
        double xPos = (X-currentX)/Math.sqrt(denominatorMath);
        Vector2d direction = new Vector2d(xPos,yPos);
        double initalSpeed = speed;


        alignModules = true;

        //turns modules to correct positions for straight driving
        if (alignModules)
            robot.driveController.rotateModules(direction, false, robot.driveController.DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);

        //sets a flag in modules so that they will not try to correct rotation while driving
        if (fixModules) robot.driveController.setRotateModuleMode(DO_NOT_ROTATE_MODULES);
        else robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode

        robot.driveController.resetDistanceTraveled();
        robot.driveController.updateTracking(); //ADDED

        while (linearOpMode.opModeIsActive()) {
            if (!(between(currentX, X - maxError, X + maxError))) {
                    robot.updateBulkData();
                    updateSLAMNav();
                    robot.driveController.updateTracking();
                    //slows down drive power in certain range
                    //updateTracking(); //WAS MOVED ABOVE
                    denominatorMath = ((X-currentX)*(X-currentX)+(Y-currentY)*(Y-currentY));
                    yPos = (Y-currentY)/Math.sqrt(denominatorMath);
                    xPos = (X-currentX)/Math.sqrt(denominatorMath);
                    direction = new Vector2d(xPos,yPos);

                    linearOpMode.telemetry.addData("Driving robot", "");
                    linearOpMode.telemetry.addData("Current X: ", currentX);
                    linearOpMode.telemetry.addData("Current Y: ", currentY);
                    linearOpMode.telemetry.addData("Current Theta: ", currentTheta);
                    linearOpMode.telemetry.update();
                    robot.driveController.updatePositionTracking(telemetry); //update position tracking
                    robot.driveController.update(direction, 0);

                }
            else {
                if (!(between(currentY,Y-maxError,Y+maxError))) {
                    robot.updateBulkData();
                    updateSLAMNav();
                    robot.driveController.updateTracking();
                    //slows down drive power in certain range
                    //updateTracking(); //WAS MOVED ABOVE
                    denominatorMath = ((X-currentX)*(X-currentX)+(Y-currentY)*(Y-currentY));
                    yPos = (Y-currentY)/Math.sqrt(denominatorMath);
                    xPos = (X-currentX)/Math.sqrt(denominatorMath);
                    direction = new Vector2d(xPos,yPos);

                    linearOpMode.telemetry.addData("Driving robot", "");
                    linearOpMode.telemetry.addData("Current X: ", currentX);
                    linearOpMode.telemetry.addData("Current Y: ", currentY);
                    linearOpMode.telemetry.addData("Current Theta: ", currentTheta);
                    linearOpMode.telemetry.update();

                    robot.driveController.updatePositionTracking(telemetry); //update position tracking
                    robot.driveController.update(direction, 0);
                }
                else {
                    break;
                }

            }
        }
        robot.driveController.update(Vector2d.ZERO, 0);
        robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
    }

    public static boolean between(double i, double minValueInclusive, double maxValueInclusive) {
        if (i >= minValueInclusive && i <= maxValueInclusive)
            return true;
        else
            return false;
    }

    public void updateSLAMNav() {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        currentX = startingX + -translation.getY();
        currentY = startingY + translation.getX();
        currentTheta = startingTheta + rotation.getDegrees();
        currentPose = new Pose(currentX, currentY, currentTheta);

        telemetry.addData("X", currentX);
        telemetry.addData("Y", currentY);
        telemetry.addData("Rotation", currentTheta);
        telemetry.update();
    }


    public void followCurvePath(Path path, LinearOpMode linearOpMode) {
        this.current_path = path;


        while (linearOpMode.opModeIsActive()) {

            updateSLAMNav();
            robot.updateBulkData();

            // Get our lookahead point
            path.update(currentPose);
            Pose lookahead_pose = path.getFollowPose();

            // Get the distance to our lookahead point
            double distance = Math.hypot(lookahead_pose.x-currentPose.x, lookahead_pose.y-currentPose.y);
 
            double speed;
            // Find our drive speed based on distance
            if (distance < current_path.getFollowCircle().radius) {
                speed = Range.clip((distance / 8) + 0.1, 0, 1);
            } else {
                speed = 1;
            }

            // Find our turn speed based on angle difference
            double turn_speed = Range.clip(Math.abs(currentPose.angle -lookahead_pose.angle) / (Math.PI/4) + 0.1, 0, 1);

            // Drive towards the lookahead point
            driveUsingPurePursuit(lookahead_pose, speed, turn_speed, linearOpMode);
        }

    }

    public void driveUsingPurePursuit(Pose pose, double drive_speed, double turn_speed, LinearOpMode linearOpMode) {

        // Find the angle to the pose
        Vector2d directionPP = new Vector2d((pose.x-currentPose.x), (pose.y-currentPose.y)).scale(drive_speed);

        // Find movement vector to drive towards that point
        double mvmt_a = -Math.signum(Range.clip((pose.angle - currentPose.angle - (Math.PI/2)), -1, 1)) * turn_speed;

        // Update actual motor powers with our movement vector

        robot.updateBulkData();
        updateSLAMNav();
        robot.driveController.updateTracking();
        linearOpMode.telemetry.addData("Driving robot", "");
        linearOpMode.telemetry.addData("Current X: ", currentX);
        linearOpMode.telemetry.addData("Current Y: ", currentY);
        linearOpMode.telemetry.addData("Current Theta: ", currentTheta);
        linearOpMode.telemetry.update();

        robot.driveController.updatePositionTracking(telemetry); //update position tracking
        robot.driveController.update(directionPP, mvmt_a);

    }


}
