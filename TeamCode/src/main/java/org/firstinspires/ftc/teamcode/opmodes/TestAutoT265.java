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
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.RobotUtil;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.ROTATE_MODULES;

@Autonomous(name = "Diff Swerve Test Auto", group = "Linear Opmode")

public class TestAutoT265 extends LinearOpMode {
    Robot robot;
    public boolean willResetIMU = true;

    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double currentX = 0;
    double currentY = 0;
    double currentTheta = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init
        robot = new Robot(this, true);
        robot.initIMU();
        slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);


        //Beginning
        waitForStart();
        slamra.start();

        //Actual OpMode



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

        while (!(between(currentX,currentX-maxError,currentX+maxError)) && !(between(currentY,currentY-maxError,currentY+maxError)) && linearOpMode.opModeIsActive()) {
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
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);

        currentX = currentX + translation.getX();
        currentY = currentY + translation.getY();
        currentTheta = currentTheta + rotation.getDegrees();
    }

}
