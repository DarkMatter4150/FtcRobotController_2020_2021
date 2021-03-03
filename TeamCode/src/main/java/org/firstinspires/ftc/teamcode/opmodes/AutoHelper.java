package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.RobotUtil;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.opmodes.TestCameraT265.slamra;

public class AutoHelper {

//    public void drive(Vector2d direction, double cmDistance, double speed, boolean fixModules, boolean alignModules, LinearOpMode linearOpMode) {
//        cmDistance = cmDistance; //BAD :(
//        double initalSpeed = speed;
//
//        alignModules = true;
//
//        //turns modules to correct positions for straight driving
//        if (alignModules)
//            robot.driveController.rotateModules(direction, false, robot.driveController.DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);
//
//        //sets a flag in modules so that they will not try to correct rotation while driving
//        if (fixModules) robot.driveController.setRotateModuleMode(DO_NOT_ROTATE_MODULES);
//        else robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
//
//        robot.driveController.resetDistanceTraveled();
//        robot.driveController.updateTracking(); //ADDED
//
//        while (robot.driveController.getDistanceTraveled() < cmDistance && /*System.currentTimeMillis() - startTime < DRIVE_TIMEOUT && */linearOpMode.opModeIsActive()) {
//            robot.updateBulkData();
//            //updateSLAMNav();
//            robot.driveController.updateTracking();
//            //slows down drive power in certain range
//            if (cmDistance - robot.driveController.getDistanceTraveled() < robot.driveController.START_DRIVE_SLOWDOWN_AT_CM) {
//                speed = RobotUtil.scaleVal(cmDistance - robot.driveController.getDistanceTraveled(), 0, robot.driveController.START_DRIVE_SLOWDOWN_AT_CM, 0.3, initalSpeed);
//                linearOpMode.telemetry.addData("speed: ", speed);
//            }
//            //updateTracking(); //WAS MOVED ABOVE
//            robot.driveController.update(direction.normalize(Math.abs(speed)), 0); //added ABS for DEBUGGING
//
//            linearOpMode.telemetry.addData("Driving robot", "");
//            linearOpMode.telemetry.addData("Distance Traveled", robot.driveController.getDistanceTraveled());
//            linearOpMode.telemetry.addData("CM Distance", cmDistance);
//            //linearOpMode.telemetry.addData("SLAM Pos", translationSLAM);
//            //linearOpMode.telemetry.addData("SLAM Rot", rotationSLAM);
//            linearOpMode.telemetry.update();
//
//            robot.driveController.updatePositionTracking(telemetry); //update position tracking
//        }
//        robot.driveController.update(Vector2d.ZERO, 0);
//        robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
//    }

//    public void driveToPosition(double X, double Y, double speed, boolean fixModules, boolean alignModules, double maxError, LinearOpMode linearOpMode) {
//        updateSLAMNav();
//        double denominatorMath = ((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));
//        double yPos = (Y - currentY) / Math.sqrt(denominatorMath);
//        double xPos = (X - currentX) / Math.sqrt(denominatorMath);
//        Vector2d direction = new Vector2d(xPos, yPos);
//        double initalSpeed = speed;
//
//
//        alignModules = true;
//
//        //turns modules to correct positions for straight driving
//        if (alignModules)
//            robot.driveController.rotateModules(direction, false, robot.driveController.DEFAULT_TIMEOUT_ROT_MODULES, linearOpMode);
//
//        //sets a flag in modules so that they will not try to correct rotation while driving
//        if (fixModules) robot.driveController.setRotateModuleMode(DO_NOT_ROTATE_MODULES);
//        else robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
//
//        robot.driveController.resetDistanceTraveled();
//        robot.driveController.updateTracking(); //ADDED
//        robot.driveController.update(Vector2d.FORWARD, 0);
//        robot.driveController.setRotateModuleMode(ROTATE_MODULES);
//
//        while (linearOpMode.opModeIsActive()) {
//            if (!(between(currentX, X - maxError, X + maxError))) {
//                robot.updateBulkData();
//                updateSLAMNav();
//                robot.driveController.updateTracking();
//                //slows down drive power in certain range
//                //updateTracking(); //WAS MOVED ABOVE
//                denominatorMath = ((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));
//                yPos = (Y - currentY) / Math.sqrt(denominatorMath);
//                xPos = (X - currentX) / Math.sqrt(denominatorMath);
//                direction = new Vector2d(xPos, yPos);
//
//                linearOpMode.telemetry.addData("Driving robot", "");
//                linearOpMode.telemetry.addData("Current X: ", currentX);
//                linearOpMode.telemetry.addData("Current Y: ", currentY);
//                linearOpMode.telemetry.addData("Current Theta: ", currentTheta);
//                linearOpMode.telemetry.update();
//                robot.driveController.updatePositionTracking(telemetry); //update position tracking
//                robot.driveController.update(direction, 0);
//
//            } else {
//                if (!(between(currentY, Y - maxError, Y + maxError))) {
//                    robot.updateBulkData();
//                    updateSLAMNav();
//                    robot.driveController.updateTracking();
//                    //slows down drive power in certain range
//                    //updateTracking(); //WAS MOVED ABOVE
//                    denominatorMath = ((X - currentX) * (X - currentX) + (Y - currentY) * (Y - currentY));
//                    yPos = (Y - currentY) / Math.sqrt(denominatorMath);
//                    xPos = (X - currentX) / Math.sqrt(denominatorMath);
//                    direction = new Vector2d(xPos, yPos);
//
//                    linearOpMode.telemetry.addData("Driving robot", "");
//                    linearOpMode.telemetry.addData("Current X: ", currentX);
//                    linearOpMode.telemetry.addData("Current Y: ", currentY);
//                    linearOpMode.telemetry.addData("Current Theta: ", currentTheta);
//                    linearOpMode.telemetry.update();
//
//                    robot.driveController.updatePositionTracking(telemetry); //update position tracking
//                    robot.driveController.update(direction, 0);
//                } else {
//                    robot.driveController.update(Vector2d.ZERO, 0);
//                    robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
//                    break;
//                }
//
//            }
//        }
//        robot.driveController.update(Vector2d.ZERO, 0);
//        robot.driveController.setRotateModuleMode(ROTATE_MODULES); //reset mode
//    }

    public static boolean between(double i, double minValueInclusive, double maxValueInclusive) {
        if (i >= minValueInclusive && i <= maxValueInclusive)
            return true;
        else
            return false;
    }

    public static void updateSLAMNav(Telemetry telemetry, FtcDashboard dashboard,
                                     Pose startingPose,
                                     Pose currentPose) {
        final int robotRadius = 9; // inches

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        currentPose.x = startingPose.x + -translation.getY();
        currentPose.y = startingPose.y + translation.getX();
        currentPose.angle = startingPose.angle - rotation.getDegrees();

        telemetry.addData("X", currentPose.x);
        telemetry.addData("Y", currentPose.y);
        telemetry.addData("Rotation", currentPose.angle);
        telemetry.update();
    }


    public static void followCurvePath(Path path, double speed, double pvalue, LinearOpMode linearOpMode,
                                       Robot robot, Telemetry telemetry, FtcDashboard dashboard,
                                       Pose startingPose,
                                       Pose currentPose) {
        Path current_path = path;

        while (linearOpMode.opModeIsActive()) {

            if (!path.isComplete()) {
                updateSLAMNav(telemetry, dashboard, startingPose, currentPose);
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
                driveUsingPurePursuit(lookahead_pose, path, speedFinal, turn_speed, pvalue, linearOpMode, robot, telemetry, dashboard, startingPose, currentPose);
            } else {
                robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);
                break;
            }

        }
        robot.driveController.update(Vector2d.ZERO, 0);//mvmt_a);

    }

    public static void driveUsingPurePursuit(Pose pose, Path path, double drive_speed, double turn_speed, double pvalue, LinearOpMode linearOpMode,
                                             Robot robot, Telemetry telemetry, FtcDashboard dashboard,
                                             Pose startingPose,
                                             Pose currentPose) {

        // Find the angle to the pose
        Vector2d directionPP = new Vector2d((pose.x - currentPose.x), (pose.y - currentPose.y)).normalize(drive_speed);

        // Find movement vector to drive towards that point
        double mvmt_a = -Math.signum(Range.clip((pose.angle - currentPose.angle - (Math.PI / 2)), -1, 1)) * turn_speed;
        mvmt_a = mvmt_a * pvalue;//- pidController(path.getHeadingGoal(path.heading_method,pose), currentPose.angle,.0000000000004,0,0);


        // Update actual motor powers with our movement vector

        robot.updateBulkData();
        updateSLAMNav(telemetry, dashboard, startingPose, currentPose);
        robot.driveController.updateTracking();
        linearOpMode.telemetry.addData("Driving robot", "");
        linearOpMode.telemetry.addData("Current X: ", currentPose.x);
        linearOpMode.telemetry.addData("Current Y: ", currentPose.y);
        linearOpMode.telemetry.addData("Current Theta: ", currentPose.angle);
        linearOpMode.telemetry.update();

        robot.driveController.updatePositionTracking(telemetry); //update position tracking
        robot.driveController.update(directionPP, mvmt_a);

    }


//    public double pidController(double target, double current, double Kp, double Ki, double Kd) {
//        error = target - current;
//        errorChange = error - lastError;
//        errorSum += error;
//        double correction = Kp * error + Ki * errorSum + Kd * errorChange;
//        lastError = error;
//        TelemetryPacket packet = new TelemetryPacket();
//        packet.put("Correction", correction);
//        dashboard.sendTelemetryPacket(packet);
//        return correction;
//    }

//    public void fourRingAuto() {
//        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 70)).addPoint(new PathPoint(25, 108)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-100, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(27, 30)).headingMethod(Path.HeadingMethod.AWAY_FROM_PATH_END);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(1000);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(35, 90)).addPoint(new PathPoint(25, 108)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(-10, 53)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(250);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//    }
//
//    public void oneRingAuto() {
//        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 70)).addPoint(new PathPoint(0, 88)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-100, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(27, 30)).headingMethod(Path.HeadingMethod.AWAY_FROM_PATH_END);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(1000);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(35, 55)).addPoint(new PathPoint(0, 88)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(-10, 53)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(250);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//    }
//
//    public void zeroRingAuto() {
//        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(25, 68)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-100, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(27, 30)).headingMethod(Path.HeadingMethod.AWAY_FROM_PATH_END);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(1000);
//
//        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(35, 55)).addPoint(new PathPoint(25, 68)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//
//        path = new Path().addPoint(new PathPoint(-10, 53)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
//        followCurvePath(path, .8 * Math.sqrt(2), 0.08, this);
//        sleep(250);
//        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING), .5, this);
//    }

}
