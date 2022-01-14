//package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;
//import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
//
//
///*
// * This is an example of a more complex path to really test the tuning.
// */
//
//
//@SuppressWarnings("unused")
//@Autonomous(group = "drive")
//public class RedRightAbsolute extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        CheckmateRobot robot = new CheckmateRobot(hardwareMap);
//        Pose2d startPose = new Pose2d(9, -63, Math.toRadians(90));
//        PositionUtil.set(startPose);
//        robot.drivetrain.setPoseEstimate(startPose);
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        sleep(3000);
//
//        TrajectorySequence toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(new Pose2d(12,-53, Math.toRadians(-30)))
//                .lineToConstantHeading(new Vector2d(-5,-47))
//                .build();
//
//        robot.drivetrain.followTrajectorySequence(toAllianceHub);
//
//
//        Pose2d pose = FieldLocations.RED_ALLIANCE_SHIPPING_HUB_STEP_2.pose;
//        TrajectorySequence toalliancehub = robot.drivetrain.trajectorySequenceBuilder(startPose)
//                .lineToLinearHeading(FieldLocations.RED_ALLIANCE_SHIPPING_HUB_STEP_1.pose)
//                .lineToConstantHeading(new Vector2d(pose.getX(), pose.getY()))
//                .build();*//*
//
//
//
//        sleep(3000);
//
//        TrajectorySequence toSpinnerSeq = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
//                .lineToLinearHeading(new Pose2d(-46, -45, Math.toRadians(0)))
//                .lineToConstantHeading(new Vector2d(-46, -60))
//                .build();
//
//        robot.drivetrain.followTrajectorySequence(toSpinnerSeq);
//
//        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
//                .lineToConstantHeading(new Vector2d(-40, -45))
//                .lineToConstantHeading(new Vector2d(50,-45))
//                .build();
//        robot.drivetrain.followTrajectorySequence(toWarehouse);
//
//        sleep(3000);
//
//        Trajectory returnHome = robot.drivetrain.trajectoryBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
//                .lineToLinearHeading(startPose)
//                .build();
//        robot.drivetrain.followTrajectory(returnHome);
//        robot.cleanup();
//    }
//
//    public double getX(CheckmateRobot robot) {
//        return robot.drivetrain.getPoseEstimate().getX();
//    }
//    public double getY(CheckmateRobot robot) {
//        return robot.drivetrain.getPoseEstimate().getY();
//    }
//    public double getRot(CheckmateRobot robot) {
//        return robot.drivetrain.getPoseEstimate().getHeading();
//    }
//}
//*/
