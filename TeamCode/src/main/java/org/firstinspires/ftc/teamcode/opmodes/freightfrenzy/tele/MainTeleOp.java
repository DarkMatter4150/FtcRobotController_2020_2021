package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;//package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.Range;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
//import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
//import org.firstinspires.ftc.teamcode.robot.subsystems.LinearSlides;
//import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;
//
//import java.util.Locale;
//
//@Config
//@TeleOp(name = "TeleOp")
//public class MainTeleOp extends BaseOpMode {
//    public static double liftChangeSpeed = 0.2;
//
//    @Override
//    public void setup(){
//
//        double duckTimer = 0;
//        ElapsedTime timer = new ElapsedTime();
//
//
//        // Retrieve our pose from the PoseStorage.currentPose static field
//        //robot.drivetrain.setPoseEstimate(PositionUtil.get());
//
//        //y button moves duck spinner
//        gp2.y.onActivate = () -> robot.carousel.spinner(duckTimer, timer, gp2);
//
//        // Left stick Y axis runs the arm
//        gp2.leftStickY.setActivationThreshold(0.4);
//        gp2.leftStickY.whileActive = () -> robot.lift.setPower(1F);
//        gp2.leftStickY.whileActiveNeg = () -> robot.lift.setPower(-1F);
//
//        // Dpad does set points
//        //gp2.dpadUp.onActivate = () -> robot.lift.setTarget(Lift.Points.HIGH);
//        //gp2.dpadRight.onActivate = () -> robot.lift.setTarget(Lift.Points.LOW);
//        //gp2.dpadLeft.onActivate = () -> robot.lift.setTarget(Lift.Points.LOW);
//        //gp2.dpadDown.onActivate = () -> robot.lift.setTarget(Lift.Points.MIN);
//
//        // X wiggles the bucket
//        gp2.x.onActivate = () -> robot.intake.setPosition(Intake.Positions.REST);
//        gp2.x.onDeactivate = () -> robot.intake.setPosition(Intake.Positions.INTAKE);
//
//        // B dumps the bucket
//        gp2.b.onActivate = () -> robot.intake.setPosition(Intake.Positions.DUMP);
//        gp2.b.onDeactivate = () -> robot.intake.setPosition(Intake.Positions.REST);
//
//        // Triggers run the intake
//        gp2.rightTrigger.onActivate = () -> robot.intake.setPower(-1);
//        gp2.rightTrigger.onDeactivate = () -> robot.intake.setPower(0);
//        gp2.leftTrigger.onActivate = () -> robot.intake.setPower(1);
//        gp2.leftTrigger.onDeactivate = () -> robot.intake.setPower(0);
//
//    }
//
//    @Override
//    public void run_loop(){
//        //updatePosition();
//        // Moves the robot based on the GP1 left stick
//        robot.drivetrain.setWeightedDrivePower(
//                new Pose2d(
//                        // left stick X
//                        -gp1.leftStickY.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
//                        // left sick Y
//                        -gp1.leftStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
//                        // right stick X (rotation)
//                        -gp1.rightStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1)
//                )
//        );
//        switch (opModeType){
//            case TeleOp:
//                // Moves the robot based on the GP1 left stick
//                robot.drivetrain.setWeightedDrivePower(
//                        new Pose2d(
//                                // left stick X
//                                -gp1.leftStickY.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
//                                // left sick Y
//                                -gp1.leftStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1),
//                                // right stick X (rotation)
//                                -gp1.rightStickX.getCorrectedValue() * Range.scale(gp1.rightTrigger.getCorrectedValue(), -1, 1, 0, 1)
//                        )
//                );
//                break;
//
//            case Autonomous:
//                // Replace false here with a check to cancel the sequence
//                //noinspection ConstantConditions
//                if (false) robot.drivetrain.cancelSequence();
//                if (!robot.drivetrain.isBusy()) opModeType = OpModeType.TeleOp;
//                break;
//            default:
//                // If we end up here, something went horribly wrong.
//                // Generally, the best plan of action is to ignore
//                //  it and move on.
//                opModeType = OpModeType.TeleOp;
//                // Mission accomplished.
//                break;
//        }
//    }
//
///*    private void updatePosition() {
//        Pose2d position = robot.drivetrain.getPoseEstimate();
//        Pose2d velocity = robot.drivetrain.getPoseVelocity();
//        PositionUtil.set(position);
//        // Print pose to telemetry
//        telemetry.addData("liftHeight", robot.lift.getHeight());
//        telemetry.addData("x", position.getX());
//        telemetry.addData("y", position.getY());
//        telemetry.addData("h", Math.toDegrees(position.getHeading()));
//        telemetry.addData("runtime",String.format(Locale.ENGLISH,"%fs",getRuntime()));
//        if (velocity != null) {
//            telemetry.addData("vX", velocity.getX());
//            telemetry.addData("vY", velocity.getY());
//            telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
//        }
//        telemetry.update();
//    }*/
//}