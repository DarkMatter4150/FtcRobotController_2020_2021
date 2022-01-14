package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.other;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket;
import org.firstinspires.ftc.teamcode.robot.subsystems.FreightFrenzyPipeline;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DarkMatterMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.robot.util.PoseUtil;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.DriveConstants.MAX_ACCEL;

import java.util.Locale;
import java.util.Objects;
@Disabled
@Autonomous(preselectTeleOp = "Red TeleOp")
public class RedLeft2 extends BaseOpMode {
    Pose2d startPose = new Pose2d(9, -63, Math.toRadians(90));

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);


        leftAuto();



        //sleep(5000);


        Trajectory returnHome = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(startPose)
                .build();
        //robot.drivetrain.followTrajectory(returnHome);


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
        Trajectory toAllianceHub = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate())
                .lineTo(new Vector2d(0,-58))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub);

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(3,-43, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(40,-43, Math.toRadians(0)))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);
    }


}

