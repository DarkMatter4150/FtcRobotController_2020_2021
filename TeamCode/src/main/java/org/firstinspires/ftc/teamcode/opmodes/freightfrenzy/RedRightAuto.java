package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.coyote.geometry.Pose;
import org.firstinspires.ftc.teamcode.coyote.path.Path;
import org.firstinspires.ftc.teamcode.coyote.path.PathPoint;
import org.firstinspires.ftc.teamcode.drivecontrol.Angle;
import org.firstinspires.ftc.teamcode.drivecontrol.AutoHelper;
import org.firstinspires.ftc.teamcode.drivecontrol.Robot;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name = "Red Right", group = "Linear Opmode")
public class RedRightAuto extends LinearOpMode {

    public static double SPEED = .5;
    public static double PVALUE = .05;
    public boolean willResetIMU = true;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    Pose startingPose = new Pose(0, 0, 0);
    Pose currentPose = new Pose().copy(startingPose);

    public Path current_path;
    OpenCvWebcam webcam;
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(this, true);
        robot.initIMU();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        FreightFrenzyPipeline pipeline = new FreightFrenzyPipeline(640, telemetry);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        robot.initIntakeServo();
        waitForStart();

        ElapsedTime autoTimer = new ElapsedTime();
        double average = 0;
        Integer reps = 0;
        while (opModeIsActive() && autoTimer.milliseconds() <= 2000)
        {
            telemetry.addData("Analysis: ", pipeline.getLocation());
            telemetry.addData("Avg: ", average/reps);
            telemetry.update();

            if (pipeline.getLocation() == FreightFrenzyPipeline.BarcodeLocation.LEFT) {
                average+=3;
                reps++;
            }
            else if (pipeline.getLocation() == FreightFrenzyPipeline.BarcodeLocation.MIDDLE) {
                average+=1;
                reps++;
            }
            else {
                average+=0;
            }
            sleep(50);
        }

        float location = (float) (average/reps);


        Path path = new Path().addPoint(new PathPoint(-17, 18)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, .5, 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);

        robot.driveController.rotateRobot(new Angle(-10, Angle.AngleType.NEG_180_TO_180_HEADING),1, this, 1000);

        if(location >= 2.5){
            //Left
            leftAuto();
        }else if (location < 2.5 && location >= 1.5){
            //Middle
            middleAuto();
        }else if (location < 1){
            //Right
           middleAuto();
        } else {
            //Right and Backup
            middleAuto();
        }

        /*

        path = new Path().addPoint(new PathPoint(-17, 5)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE).positionPrecision(3).headingPrecision(15);
        AutoHelper.followCurvePath(path, 1, 0.1, this, robot, telemetry, dashboard,  startingPose, currentPose);

        robot.driveController.rotateRobot(new Angle(90, Angle.AngleType.NEG_180_TO_180_HEADING),1, this, 4000);

        Path path2 = new Path().addPoint(new PathPoint(-30, 25)).addPoint(new PathPoint(-60, 14)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE).positionPrecision(2).headingPrecision(200).constantHeading(90);
        AutoHelper.followCurvePath(path2, 0.5, 0.1, this, robot, telemetry, dashboard,  startingPose, currentPose);

        robot.driveController.rotateRobot(new Angle(75, Angle.AngleType.NEG_180_TO_180_HEADING),1, this, 3000);


        ElapsedTime timer = new ElapsedTime();
        double duckTimer = 0;
        duckTimer = timer.milliseconds();
        while (timer.milliseconds() - duckTimer < 1200) {
            float speed = (float) (.0008*(timer.milliseconds() - duckTimer)+.1);
            robot.setDuckSpinnerPower(-speed);
        }
        while (timer.milliseconds() - duckTimer < 1600 && timer.milliseconds() - duckTimer >= 1100) {
            robot.setDuckSpinnerPower(-0.9F);
        }
        while (timer.milliseconds() - duckTimer < 1650 && timer.milliseconds() - duckTimer >= 1600) {
            robot.setDuckSpinnerPower((float)0.1);
        }
        robot.setDuckSpinnerPower(0);

        path2 = new Path().addPoint(new PathPoint(-62, 32)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE).positionPrecision(1).headingPrecision(10).constantHeading(90);
        AutoHelper.followCurvePath(path2, 0.5, 0.1, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.initIntakeServo();
        robot.driveController.rotateRobot(new Angle(90, Angle.AngleType.NEG_180_TO_180_HEADING),1, this, 3000);
        sleep(10000);

         */
    }

    public void leftAuto() {
        robot.setIntakeServo(true);

        Path path = new Path().addPoint(new PathPoint(-17, 15)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE).positionPrecision(2).headingPrecision(15);
        AutoHelper.followCurvePath(path, 1, 0.1, this, robot, telemetry, dashboard,  startingPose, currentPose);


        sleep(1500);
        robot.setIntakePower(-1F);
        sleep(125);
        robot.setIntakePower(0.5F);
        sleep(1500);
        robot.setIntakePower(0);
    }

    public void middleAuto() {
        robot.setLiftPower(-1);
        sleep(1600);
        robot.setLiftPower(0);
        robot.setIntakeServo(true);
        sleep(1500);
        robot.setIntakePower(-1F);
        sleep(125);
        robot.setIntakePower(0.5F);
        sleep(1000);
        robot.setIntakePower(0);
        robot.setIntakeServo(true);

        Path path = new Path().addPoint(new PathPoint(-17, 10)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE).positionPrecision(3).headingPrecision(15);
        AutoHelper.followCurvePath(path, 1, 0.1, this, robot, telemetry, dashboard,  startingPose, currentPose);

        robot.setLiftPower(1);
        sleep(1600);
        robot.setLiftPower(0);
    }

    public void rightAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,112)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1, 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this, 2000);
        robot.setIntakePower(1);
        sleep(1000);
    }
}