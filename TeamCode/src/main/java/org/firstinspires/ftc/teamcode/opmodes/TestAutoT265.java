 package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.firstinspires.ftc.teamcode.drivecontrol.RobotUtil;
import org.firstinspires.ftc.teamcode.drivecontrol.Vector2d;

import java.util.Vector;

import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.DO_NOT_ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.drivecontrol.DriveModule.RotateModuleMode.ROTATE_MODULES;
import static org.firstinspires.ftc.teamcode.opmodes.TestCameraT265.slamra;
@Config
@Autonomous(name = "T265 Test Auto", group = "Linear Opmode")

public class TestAutoT265 extends LinearOpMode {

    public static double SPEED = .5;
    public static double PVALUE = .05;
    Robot robot;
    public boolean willResetIMU = true;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double startingX = 0;
    double startingY = 0;
    double startingTheta = 0;

    Double currentX = 0.0;
    Double currentY = 0.0;
    Double currentTheta = 0.0;

    double setPoint = 0;
    double error = 0;
    double lastError = 0;
    double errorSum = 0;
    double errorChange = 0;

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
        //driveToPosition2(75,72,.5,true,false,5,this);

        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10,55)).addPoint(new PathPoint(25,68)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, .8*Math.sqrt(2), 0.08, this, robot, telemetry, dashboard, startingX, startingY, startingTheta, currentX, currentY, currentTheta, currentPose);
        robot.driveController.rotateRobot(new Angle(-100, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);
        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);

        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(27, 30)).headingMethod(Path.HeadingMethod.AWAY_FROM_PATH_END);
        AutoHelper.followCurvePath(path, .8*Math.sqrt(2), 0.08, this, robot, telemetry, dashboard, startingX, startingY, startingTheta, currentX, currentY, currentTheta, currentPose);
        sleep(1000);

        path = new Path().addPoint(new PathPoint(35, 35)).addPoint(new PathPoint(35, 55)).addPoint(new PathPoint(25, 68)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, .8*Math.sqrt(2), 0.08, this, robot, telemetry, dashboard, startingX, startingY, startingTheta, currentX, currentY, currentTheta, currentPose);
        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);
        robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);

        path = new Path().addPoint(new PathPoint(-10, 53)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, .8*Math.sqrt(2), 0.08, this, robot, telemetry, dashboard, startingX, startingY, startingTheta, currentX, currentY, currentTheta, currentPose);
        sleep(250);
        robot.driveController.rotateRobot(new Angle(-90, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);
        //robot.driveController.rotateRobot(new Angle(0, Angle.AngleType.NEG_180_TO_180_HEADING),.5, this);
        //robot.driveController.rotateRobot(new Angle(90,Angle.AngleType.NEG_180_TO_180_CARTESIAN),.5, this);
        //robot.driveController.rotateRobot(new Angle(0,Angle.AngleType.NEG_180_TO_180_CARTESIAN),.5, this);
        //robot.driveController.rotateModules(Vector2d.FORWARD,false, 2000, this);


        while (opModeIsActive()) {
            AutoHelper.updateSLAMNav(telemetry, dashboard, startingX, startingY, startingTheta, currentX, currentY, currentTheta, currentPose);
            robot.updateBulkData();
        }



    }

}
