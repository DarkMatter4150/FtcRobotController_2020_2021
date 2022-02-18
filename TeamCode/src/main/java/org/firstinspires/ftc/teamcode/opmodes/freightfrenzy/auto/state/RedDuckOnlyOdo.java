package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Locale;
import java.util.Objects;

@Autonomous(name = "Odo Red Duck Only", preselectTeleOp = "Red TeleOp")
public class RedDuckOnlyOdo extends BaseOpMode {
    //STARTING LOCATION
    //COORDINATES POSITIVE THETA IS CCW
    //-X is down
    //NEGATIVE Y is Right
    Pose2d startPose = new Pose2d(-41, -63, Math.toRadians(90));
    public final long delay = 0;

    /**
     * Runs when the OpMode initializes
     */

    @Override
    public void setup() {
        robot.drivetrain.resetEncoderValues(2);
        robot.drivetrain.setLocalizer(new TrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);
        duckSpinnerMovement();
        TrajectorySequence toStorage = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, -30, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toStorage);
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
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime", String.format(Locale.ENGLISH, "%fs", getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }

    private void duckSpinnerMovement()
    {
        TrajectorySequence toDuckSpinner = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-39, -50, Math.toRadians(0)))
                //.lineToLinearHeading(new Pose2d(-60 , -46, Math.toRadians(0)))
                //.strafeRight(6)
                .build();
        robot.drivetrain.followTrajectorySequence(toDuckSpinner);

        Trajectory toDuckSpinner2 = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(),true)
                //.lineToLinearHeading(new Pose2d(-41, -46, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-62 , -55, Math.toRadians(17)))
                //.strafeRight(6)
                .build();
        robot.drivetrain.followTrajectory(toDuckSpinner2);

        robot.carousel.setPower(.5);
        sleep(3000);
        robot.carousel.setPower(0);
        sleep(500);
    }

}