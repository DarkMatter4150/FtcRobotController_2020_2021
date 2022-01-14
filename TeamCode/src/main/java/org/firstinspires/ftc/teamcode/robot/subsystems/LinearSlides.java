package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;


public class LinearSlides implements AbstractSubsystem {

    public enum SlidePositions
    {
        // Position when the bucket is initialized
        DOWN(0.0),
        // Position when the bucket is dumping
        LOW(3.0),
        // The normal position
        MIDDLE(15.0),
        // The position when the bucket is all the way up
        TOP(20.0);

        public double pos;

        SlidePositions(Double pos) {
            this.pos = pos;
        }
    }

    private final DcMotorEx linearSlide1;
    private final DcMotorEx linearSlide2;


    private double targetHeight; // inches
    public SlidePositions currentLayer;

    private double targetPower = 0;
    private int direction = 1;

    // Constants to manage the height of the lift - NEEDS TO BE TWEAKED ONCE LINEAR SLIDES ARE DONE
    private static final double spoolDiameter = 1.5; // inches
    private static final double encoderTicksPerRev = 537.7;
    private static final double maxHeight = 21.26; // inches

    // Math for going from height to revolutions to geared ticks to ticks:
    /*
    targetHeight * encoderTicksPerRev
    ---------------------------------
          spoolDiameter * Math.PI
     */

    private static final double ticksPerInch = encoderTicksPerRev / (spoolDiameter * Math.PI);

    public LinearSlides(HardwareMap hardwareMap) {
        // Initialize the motor
        linearSlide1 = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT1.name);
        linearSlide2 = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT2.name);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.Motors.LIFT1.reverse) {
            linearSlide1.setDirection(DcMotorEx.Direction.REVERSE);
        }
        if (HardwareNames.Motors.LIFT2.reverse) {
            linearSlide1.setDirection(DcMotorEx.Direction.REVERSE);
        }
        // Set it to run to a target position and hold the position
        linearSlide1.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        init();
    }

    @Override
    public void update() {

        if (targetHeight > maxHeight) {
            targetHeight = maxHeight;
        } else if (targetHeight < 0) {
            targetHeight = 0;
        }

        moveToTargetPos(targetHeight);
    }

    public void moveToTargetPos(double target)
    {
        linearSlide1.setTargetPosition((int)round(target * ticksPerInch));
        linearSlide2.setTargetPosition((int)round(target * ticksPerInch));

        if (getCurrentHeightInches("all") > target) {
            direction = -1;
        }
        else if (getCurrentHeightInches("all") < target) {
            direction = 1;
        }
        targetPower = 0.1+(Math.abs(getTargetHeight() - getCurrentHeightInches("all"))/maxHeight);
        targetPower*=direction;
        setPower(targetPower);

    }

    /**
     * Sets the target height of the lift in inches
     * @param target Target height in inches
     */
    public void setHeight(SlidePositions target) {

        targetHeight = target.pos;
        currentLayer = target;
    }

    public void setPower(double power) {
        targetPower = Range.clip(power, -1, 1);
        linearSlide2.setPower(power);
        linearSlide2.setPower(power);
    }

    public double getCurrentHeightInches(String slide)
    {
        if (slide.equalsIgnoreCase("lift1")) {
            return linearSlide1.getCurrentPosition() / ticksPerInch;
        }
        else if (slide.equalsIgnoreCase("lift2")) {
            return linearSlide2.getCurrentPosition() / ticksPerInch;
        }
        else {
            return (Math.abs(linearSlide2.getCurrentPosition()) + Math.abs(linearSlide1.getCurrentPosition())) / ticksPerInch;
        }
    }
    public double getCurrentHeightTicks(String slide)
    {
        if (slide.equalsIgnoreCase("lift1")) {
            return linearSlide1.getCurrentPosition();
        }
        else if (slide.equalsIgnoreCase("lift2")) {
            return linearSlide2.getCurrentPosition();
        }
        else {
            return (Math.abs(linearSlide2.getCurrentPosition()) + Math.abs(linearSlide1.getCurrentPosition()));
        }
    }


    /**
     * Gets the target height of the lift in inches (not the actual height)
     * @return Target height in inches
     */
    public double getTargetHeight() {
        return targetHeight;
    }


    //gets shipping layer
    public SlidePositions getLayer()
    {
        return currentLayer;
    }


    /**
     * Initializes the lift subsystem
     */
    public void init()
    {
        resetEncoders();
        targetHeight = SlidePositions.DOWN.pos;

        linearSlide1.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
    }
    public void setTolerance(int ticks)
    {
        linearSlide1.setTargetPositionTolerance(ticks);
        linearSlide2.setTargetPositionTolerance(ticks);
    }

    public void resetEncoders()
    {
        linearSlide1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    @Override
    public void cleanup() {
        init();
    }
}
