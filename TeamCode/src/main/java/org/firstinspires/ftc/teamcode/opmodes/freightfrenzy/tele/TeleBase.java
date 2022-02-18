package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;
import java.util.Objects;

@Disabled
@TeleOp
public class TeleBase extends BaseOpMode {

    ElapsedTime elapsedTime;
    @Override
    public void setup(){
        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.drivetrain.setPoseEstimate(PositionUtil.get());

        // A opens the jank hand
        gp2.y.onActivate = () -> robot.carousel.spinner(elapsedTime, gp2, "blue");
        gp2.y.onDeactivate = () -> robot.carousel.setPower(0);
        // B closes the jank hand
        //gp2.b.onActivate = () -> robot.jankHand.open();

        // Right bumper runs the carousel
        //gp2.rightBumper.onActivate = () -> robot.carousel.setPower(1);
        //gp2.rightBumper.onDeactivate = () -> robot.carousel.setPower(0);

        // Left bumper runs the carousel the other way
        //gp2.leftBumper.onActivate = () -> robot.carousel.setPower(-1);
        //gp2.leftBumper.onDeactivate = () -> robot.carousel.setPower(0);

        // Left stick Y axis runs the arm
        gp2.leftStickY.setActivationThreshold(0.4);
        //gp2.leftStickY.whileActive = () -> robot.jankArm.setAngle(robot.jankArm.getAngle() - .01);
        //gp2.leftStickY.whileActiveNeg = () -> robot.jankArm.setAngle(robot.jankArm.getAngle() + .01);
    }

    @Override
    public void preRunLoop() {
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void runLoop(){
        updatePosition();

        switch (opModeType){
            case TeleOp:
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.setWeightedDrivePower(
                        new Pose2d(
                                // left stick X
                                -gp1.leftStickY.getCorrectedValue() * Range.scale((gp1.rightTrigger.getCorrectedValue()), -1, 1, 0, 1),
                                // left sick Y
                                -gp1.leftStickX.getCorrectedValue() * Range.scale((gp1.rightTrigger.getCorrectedValue()), -1, 1, 0, 1),
                                 //right stick X (rotation)
                                -gp1.rightStickX.getCorrectedValue() * Range.scale((gp1.rightTrigger.getCorrectedValue()), -1, 1, 0, 1)
                        )
                );
                break;

            case Autonomous:
                // Replace false here with a check to cancel the sequence
                //noinspection ConstantConditions
                if (false) robot.drivetrain.cancelSequence();
                if (!robot.drivetrain.isBusy()) opModeType = OpModeType.TeleOp;
                break;
            default:
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp;
                // Mission accomplished.
                break;
        }
    }

    private void updatePosition() {
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
}
