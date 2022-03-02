package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.state;

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
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.RealsenseLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;
import java.util.Objects;

@Autonomous(name = "Odo Blue Compact Duck", preselectTeleOp = "Blue TeleOp")
public class BlueCompactDuckOdo extends BaseOpMode {
    //STARTING LOCATION
    //COORDINATES POSITIVE THETA IS CCW
    //-X is down
    //NEGATIVE Y is Right
    Pose2d startPose = new Pose2d(-41, 63, Math.toRadians(-90));
    ElapsedTime duck;
    OpenCvWebcam webcam;
    FreightFrenzyPipeline pipeline;
    public final long delay = 0;
    ElapsedTime timer = new ElapsedTime();

    /**
     * Runs when the OpMode initializes
     */

    @Override
    public void setup() {
        robot.drivetrain.resetEncoderValues(2);
        robot.drivetrain.setLocalizer(new TrackingWheelLocalizer(hardwareMap));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);

        pipeline = new FreightFrenzyPipeline(640, telemetry);
        webcam.setPipeline(pipeline);
        duck = new ElapsedTime();
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
            telemetry.addData("Avg", (double) initAvg / (double) initReps);
            telemetry.update();
            initReps++;
            initAvg += pipeline.getPosition();
            sleep(100);
        }
    }

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);
        //RealsenseManager.slamera.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(startPose.getX(),startPose.getY(),new Rotation2d(startPose.getHeading())));

        ElapsedTime autoTimer = new ElapsedTime();
        double average = 0;
        double reps = 0;
        while (opModeIsActive() && autoTimer.milliseconds() <= 2500) {
            //telemetry.addData("Analysis: ", pipeline.getLocation());
            telemetry.addData("Avg: ", (double) average / reps);
            telemetry.update();


            if (pipeline.getPosition() > 0) {
                average += pipeline.getPosition();
                reps++;
            } else {
                average += 0;
            }
            sleep(50);
        }

        if (reps == 0) {
            reps++;
        }

        double location = (average / reps);

        if (location >= 2.5) {
            telemetry.addData("Running Right Auto ", "1");
            telemetry.update();
            sleep(delay);
            middleAuto();
        }
        else if (location < 1.5 && location >= 0.5) {
            //middleAuto();
            telemetry.addData("Running Middle Auto ", "1");
            telemetry.update();
            sleep(delay);
            leftAuto();
        }
        else {
            //leftAuto();
            telemetry.addData("Running Left Auto ", "1");
            telemetry.update();
            sleep(delay);
            //middleAuto();
            rightAuto();
        }

        toStorage();
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
        telemetry.addData("runtime", String.format(Locale.ENGLISH, "%fs", getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }

    public void leftAuto() {

        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-34,45, Math.toRadians(0)))
                .strafeRight(23)
                .build();
        robot.drivetrain.followTrajectorySequence(toAllianceHub);

        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(500);
        robot.intake.setPower(-0.8);
        sleep(750);
        robot.intake.setPower(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);

        duckSpinnerMovement(true);
    }

    public void rightAuto() {

        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-35,44, Math.toRadians(180)))
                .strafeLeft(20.5)
                .back(4)
                .build();
        robot.drivetrain.followTrajectorySequence(toAllianceHub);

        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(500);

        robot.lift.setTarget(Lift.Points.HIGH);
        robot.lift.update();
        timer.reset();
        while (robot.lift.getCurrentPosition() < 33800 && timer.milliseconds() <= 2000) {
            robot.lift.update();
            telemetry.addData("Lift Height", robot.lift.getCurrentPosition());
            telemetry.update();
        }
        robot.lift.setSpeed(0);
        sleep(250);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(2000);
        robot.bucket.setPosition(Bucket.Positions.DUMP_HIGH);
        sleep(1000);
        robot.intake.setPower(-0.65);
        sleep(1500);
        robot.intake.setPower(0);
        sleep(1000);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        robot.lift.setTarget(Lift.Points.MIN);
        robot.lift.update();
        while (robot.lift.getCurrentPosition() > 100) {
            robot.lift.update();
            telemetry.addData("Lift Height", robot.lift.getCurrentPosition());
            telemetry.update();
        }
        robot.lift.setSpeed(0);
        robot.bucket.setPosition(Bucket.Positions.INIT);

        duckSpinnerMovement(false);
    }
    public void middleAuto() {

        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-35,42, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-33,22, Math.toRadians(180)))
                //.forward(1.5)
                .build();
        robot.drivetrain.followTrajectorySequence(toAllianceHub);

        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(500);

        robot.lift.setHeight(21600);
        robot.lift.update();
        timer.reset();
        while (robot.lift.getCurrentPosition() < 21198 && timer.milliseconds() <= 1000) {
            robot.lift.update();
            telemetry.addData("Lift Height", robot.lift.getCurrentPosition());
            telemetry.update();
        }
        //robot.lift.setHeight(robot.lift.getCurrentPosition());
        robot.lift.setSpeed(0.0001);
        robot.lift.update();
        sleep(250);
        robot.bucket.setPosition(Bucket.Positions.FORWARD);
        sleep(2000);
        robot.bucket.setPosition(Bucket.Positions.AUTO_LOW);
        sleep(1500);
        robot.intake.setPower(-0.65);
        sleep(2000);
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


        duckSpinnerMovement(false);
    }

    private void duckSpinnerMovement(boolean side)
    {
        if (side)
        {
            TrajectorySequence toDuckSpinner = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                    .strafeLeft(27)
                    .turn(Math.toRadians(-100))
                    //.lineToLinearHeading(new Pose2d(-39, -50, Math.toRadians(0)))
                    //.lineToLinearHeading(new Pose2d(-60 , -46, Math.toRadians(0)))
                    .build();
            robot.drivetrain.followTrajectorySequence(toDuckSpinner);
        }
        else
        {
            TrajectorySequence toDuckSpinner = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                    .strafeRight(25)
                    .turn(Math.toRadians(100))
                    //.lineToLinearHeading(new Pose2d(-39, -50, Math.toRadians(0)))
                    //.lineToLinearHeading(new Pose2d(-60 , -46, Math.toRadians(0)))
                    .build();
            robot.drivetrain.followTrajectorySequence(toDuckSpinner);
        }

        Trajectory toDuckSpinner2 = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(),true)
                //.lineToLinearHeading(new Pose2d(-41, -46, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-63 , 58, Math.toRadians(-90)))
                //.strafeRight(6)
                .build();
        robot.drivetrain.followTrajectory(toDuckSpinner2);

        robot.carousel.setPower(-.5);
        sleep(3000);
        robot.carousel.setPower(0);
        sleep(500);
    }

    public void toStorage()
    {
        TrajectorySequence toStorage = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-67, 35, Math.toRadians(-90)))
                .build();
        robot.drivetrain.followTrajectorySequence(toStorage);
    }

}