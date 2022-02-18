package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.leaguechamps.compact;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;
import java.util.Objects;

@Disabled
@Autonomous(name = "P", preselectTeleOp = "Blue TeleOp")
public class ParkOnly extends BaseOpMode {
    //STARTING LOCATION
    //COORDINATES POSITIVE THETA IS CCW
    //-X is down
    //NEGATIVE Y is Right
    Pose2d startPose = new Pose2d(9, 63, Math.toRadians(-90));
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;
    public final long delay = 0;
    ElapsedTime timer = new ElapsedTime();

    /**
     * Runs when the OpMode initializes
     */

    @Override
    public  void setup() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        pipeline = new FreightFrenzyPipeline(640, telemetry);
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        telemetry.addLine("Waiting for start");
        telemetry.update();

        int initAvg = 0;
        int initReps = 0;
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Position", pipeline.getPosition());
            telemetry.addData("Avg", (double)initAvg / (double) initReps);
            telemetry.update();
            initReps++;
            initAvg+= pipeline.getPosition();
            sleep(100);
        }
    }

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);
        //RealsenseManager.slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(startPose.getX(),startPose.getY(),new Rotation2d(startPose.getHeading())));

        ElapsedTime autoTimer = new ElapsedTime();

        rightAuto();
    }


    /**
     * Main OpMode loop, automatically updates the robot
     */

    @Override
    public void runLoop() {
        Pose2d position = robot.drivetrain.getPoseEstimate();
        Pose2d velocity = Objects.requireNonNull(robot.drivetrain.getPoseVelocity());
        PositionUtil.set(position);
        // Print pose to telemetry
        //telemetry.addData("armAngle", Math.toDegrees(robot.jankArm.getAngle()));
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }

    public void rightAuto() {

        Trajectory toAllianceHub = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(), false, 10,5)
                .lineToLinearHeading(new Pose2d(3, 44, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub);


        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(48, 43))
                .turn(Math.toRadians(-100))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }
}