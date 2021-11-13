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
@Autonomous(name = "Freight Frenzy Auto", group = "Linear Opmode")
public class FreightFrenzyAuto extends LinearOpMode {

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

        FreightFrenzyPipeline pipeline = new FreightFrenzyPipeline(320, telemetry);
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
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        ElapsedTime autoTimer = new ElapsedTime();

        while (opModeIsActive() && autoTimer.milliseconds() <= 1000)
        {
            telemetry.addData("Analysis: ", pipeline.getLocation());
            telemetry.update();

        }

        FreightFrenzyPipeline.BarcodeLocation location = pipeline.getLocation();

        if(location == FreightFrenzyPipeline.BarcodeLocation.LEFT){
            leftAuto();
        }else if (location == FreightFrenzyPipeline.BarcodeLocation.MIDDLE){
            middleAuto();
        }else if (location == FreightFrenzyPipeline.BarcodeLocation.RIGHT){
            rightAuto();
        } else {
            telemetry.addLine("No Team Marker Detected");
            leftAuto();
        }

    }

    public void leftAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,112)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1, 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setIntakePower(1);
        sleep(1000);
    }

    public void middleAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,112)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1, 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setIntakePower(1);
        sleep(1000);
    }

    public void rightAuto() {
        Path path = new Path().addPoint(new PathPoint(-10, 10)).addPoint(new PathPoint(-10, 55)).addPoint(new PathPoint(18,112)).headingMethod(Path.HeadingMethod.CONSTANT_ANGLE);
        AutoHelper.followCurvePath(path, 1, 0.4, this, robot, telemetry, dashboard,  startingPose, currentPose);
        robot.driveController.rotateRobot(new Angle(45, Angle.AngleType.NEG_180_TO_180_HEADING),1, this);
        robot.setIntakePower(1);
        sleep(1000);
    }
}