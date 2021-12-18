package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;

/*
 * This is a simple routine to test translational drive capabilities.
 */

@SuppressWarnings("unused")
@Config
@Autonomous(group = "drive")
public class StrafeTest extends LinearOpMode {
    public static double DISTANCE = 60; // in

    @SuppressWarnings("StatementWithEmptyBody")
    @Override
    public void runOpMode() throws InterruptedException {
        CheckmateRobot robot = new CheckmateRobot(hardwareMap);

        Trajectory trajectory = robot.drivetrain.trajectoryBuilder(new Pose2d())
                .strafeRight(DISTANCE)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        robot.drivetrain.followTrajectory(trajectory);

        Pose2d poseEstimate = robot.drivetrain.getPoseEstimate();
        telemetry.addData("finalX", poseEstimate.getX());
        telemetry.addData("finalY", poseEstimate.getY());
        telemetry.addData("finalHeading", poseEstimate.getHeading());
        telemetry.update();

        while (!isStopRequested() && opModeIsActive()) ;
        robot.cleanup();
    }
}
