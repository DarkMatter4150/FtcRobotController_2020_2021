package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.leaguechamps;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
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

import java.util.Arrays;
import java.util.Locale;
import java.util.Objects;

@Disabled
@Autonomous(name = "Blue Left No Duck", preselectTeleOp = "Blue TeleOp")
public class BlueLeftNoDuck extends BaseOpMode {
    Pose2d startPose = new Pose2d(-33, -63, Math.toRadians(90));
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;
    public final long delay = 0;

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

        ElapsedTime autoTimer = new ElapsedTime();
        double average = 0;
        double reps = 0;
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

        double location = (average/reps);

        if (location >= 2.5) {
            sleep(delay);
            middleAuto();
        }
        else if (location < 1.5 && location >= 0.5) {
            //middleAuto();
            sleep(delay);
            leftAuto();
        }
        else {
            //leftAuto();
            sleep(delay);
            //middleAuto();
            rightAuto();
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

        robot.intake.setPower(-0.6);
        sleep(750);
        robot.intake.setPower(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);


        /*TrajectorySequence toDuck = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-47, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-67,-51, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toDuck);

        robot.carousel.setPower(0.4);
        sleep(4000);
        robot.carousel.setPower(0);*/

        double yNew = robot.drivetrain.getPoseEstimate().getY() - 5;

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(-21,yNew))
                .lineToLinearHeading(new Pose2d(-40,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-87,-43, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }

    public void rightAuto() {

        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-14,-47, Math.toRadians(90)))
                .turn(Math.toRadians(190))
                .build();
        robot.drivetrain.followTrajectorySequence(toAllianceHub);
        //robot.drivetrain.turn(Math.toRadians(190));

        double yNew = robot.drivetrain.getPoseEstimate().getY() + 5;

        Trajectory toAllianceHub2 = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(), true, 10, 6)
                .lineTo(new Vector2d(-14,yNew))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub2);

        TrajectorySequence toAllianceHub3 = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(-14,-30)).setReversed(true).setVelConstraint(new MinVelocityConstraint(Arrays.asList(
                        new AngularVelocityConstraint(3.0),
                        new MecanumVelocityConstraint(3.0, 12)
                ))).setAccelConstraint(new ProfileAccelerationConstraint(3.0))
                .build();
        //robot.drivetrain.followTrajectorySequence(toAllianceHub3);


        robot.lift.setTarget(Lift.Points.HIGH);
        robot.lift.update();
        sleep(250);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(2000);
        robot.bucket.setPosition(Bucket.Positions.AUTO_HIGH);
        sleep(1000);
        robot.intake.setPower(-0.55);
        sleep(750);
        robot.intake.setPower(0);
        sleep(1000);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        robot.lift.setTarget(Lift.Points.MIN);
        robot.lift.update();
        while (robot.lift.getCurrentPosition() > 100) {
            robot.lift.update();
        }
        robot.lift.setSpeed(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);



        TrajectorySequence toDuck = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-55, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-67,-60, Math.toRadians(0)))
                .build();
        //robot.drivetrain.followTrajectorySequence(toDuck);

        //robot.carousel.setPower(0.4);
        //sleep(4000);
        //robot.carousel.setPower(0);

        double yNew2 = robot.drivetrain.getPoseEstimate().getY() - 5;

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(-21,yNew2))
                .lineToLinearHeading(new Pose2d(-40,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-87,-43, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }
    public void middleAuto() {

        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-14,-47, Math.toRadians(90)))
                .turn(Math.toRadians(190))
                .build();
        robot.drivetrain.followTrajectorySequence(toAllianceHub);
        //robot.drivetrain.turn(Math.toRadians(190));

        double yNew = robot.drivetrain.getPoseEstimate().getY() + 5;

        Trajectory toAllianceHub2 = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(), true, 10, 6)
                .lineTo(new Vector2d(-14,yNew))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub2);



        robot.lift.setTarget(Lift.Points.MID);
        robot.lift.update();
        while (robot.lift.getCurrentPosition() < 29700) {
            robot.lift.update();
        }
        robot.lift.setSpeed(0);
        sleep(250);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(2000);
        robot.bucket.setPosition(Bucket.Positions.AUTO_LOW);
        sleep(1000);
        robot.intake.setPower(-0.55);
        sleep(750);
        robot.intake.setPower(0);
        sleep(1000);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        robot.lift.setTarget(Lift.Points.MIN);
        robot.lift.update();
        while (robot.lift.getCurrentPosition() > 100) {
            robot.lift.update();
        }
        robot.lift.setSpeed(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);



        TrajectorySequence toDuck = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-60,-55, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-67,-60, Math.toRadians(0)))
                .build();
        //robot.drivetrain.followTrajectorySequence(toDuck);

        //robot.carousel.setPower(0.4);
        //sleep(4000);
        //robot.carousel.setPower(0);

        double yNew2 = robot.drivetrain.getPoseEstimate().getY() - 5;

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(-21,yNew2))
                .lineToLinearHeading(new Pose2d(-40,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-87,-43, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }
}