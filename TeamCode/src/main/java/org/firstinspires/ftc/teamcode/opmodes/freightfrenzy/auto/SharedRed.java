package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift2;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;
import java.util.Objects;
@Autonomous(name = "RedRight OpenCVTesting", preselectTeleOp = "Red TeleOp")
public class SharedRed extends BaseOpMode {
    Pose2d startPose = new Pose2d(9, -63, Math.toRadians(90));
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;

     /**
     * Runs when the OpMode initializes
     */

     @Override
     public  void setup() {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

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
     }

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);

        ElapsedTime autoTimer = new ElapsedTime();
        double average = 0;
        Integer reps = 0;
        while (opModeIsActive() && autoTimer.milliseconds() <= 2500)
        {
            telemetry.addData("Analysis: ", pipeline.getLocation());
            telemetry.addData("Avg: ", average/reps);
            telemetry.update();

            if (pipeline.getLocation() == FreightFrenzyPipeline.BarcodeLocation.RIGHT) {
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

        if (location >= 2.5) {
            rightAuto();
        }
        else if (location < 2.5 && location >= 1.5) {
            //middle
        }
        else {
            leftAuto();
        }
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

    public void leftAuto() {
        robot.bucket.setPosition(Bucket.Positions.FORWARD);

        Trajectory toAllianceHub = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12,-46, Math.toRadians(90)))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub);

        robot.intake.setPower(-0.75);
        sleep(750);
        robot.intake.setPower(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);


        TrajectorySequence toDuck = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-47, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-67,-51, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toDuck);

        robot.carousel.setPower(0.4);
        sleep(4000);
        robot.carousel.setPower(0);

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(3,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40,-43, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }

    public void rightAuto() {
        robot.bucket.setPosition(Bucket.Positions.FORWARD);

        Trajectory toAllianceHub = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-12,-46, Math.toRadians(90)))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub);
        robot.drivetrain.turn(Math.toRadians(190));

        robot.lift.setTarget(Lift2.Points.HIGH);
        robot.lift.update();
        sleep(250);
        robot.bucket.setPosition(Bucket.Positions.DUMP_HIGH);
        sleep(750);
        robot.intake.setPower(-0.75);
        sleep(750);
        robot.intake.setPower(0);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        robot.lift.setTarget(Lift2.Points.LOW);
        robot.lift.update();



        TrajectorySequence toDuck = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-47, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-67,-51, Math.toRadians(0)))
                .build();
        //robot.drivetrain.followTrajectorySequence(toDuck);

        //robot.carousel.setPower(0.4);
        //sleep(4000);
        //robot.carousel.setPower(0);

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(3,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40,-43, Math.toRadians(0)))
                .build();
        //robot.drivetrain.followTrajectorySequence(toWarehouse);
    }


}

