package org.firstinspires.ftc.teamcode.robot.subsystems;

import static java.lang.Math.max;
import static java.lang.Math.round;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.HardwareNames;
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem;

public class LinearSlides implements AbstractSubsystem {

    private final DcMotorEx linearSlide1;
    private final DcMotorEx linearSlide2;

    //set the shipping unit layers in inches, should have 3 layers total
    private double[] layerHeights = {0, 0, 0};

    private double targetHeight; // inches

    // Constants to manage the height of the lift - NEEDS TO BE TWEAKED ONCE LINEAR SLIDES ARE DONE
    private static final double spoolDiameter = 1.5; // inches
    private static final double encoderTicksPerRev = 751.8;
    private static final double maxHeight = 20.0; // inches

    // Math for going from height to revolutions to geared ticks to ticks:
    /*
    targetHeight * encoderTicksPerRev
    ---------------------------------
          spoolDiameter * Math.PI
     */

    private static final double ticksPerInch = encoderTicksPerRev / (spoolDiameter * Math.PI);

    public LinearSlides(HardwareMap hardwareMap) {
        // Initialize the motor
        linearSlide1 = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT.name);
        linearSlide2 = hardwareMap.get(DcMotorEx.class, HardwareNames.Motors.LIFT.name);
        linearSlide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearSlide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Reverse the motor if we set it that way in the config
        if (HardwareNames.Motors.LIFT.reverse) {
            linearSlide1.setDirection(DcMotorEx.Direction.REVERSE);
        }
        // Set it to run to a target position and hold the position
        linearSlide1.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
        linearSlide1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize the target height
        targetHeight = 0;
    }

    @Override
    public void update() {
        linearSlide2.setPower(1);
        linearSlide2.setPower(1);

        if (targetHeight > maxHeight) {
            targetHeight = maxHeight;
        } else if (targetHeight < 0) {
            targetHeight = 0;
        }

        moveToTargetPos();
    }

    @Override
    public void cleanup() {
        resetPos();
    }

    //gets shipping layer
    public int getLayer()
    {
        //based on dpad return index for the layer height array
        //dpad up should return top layer(index: 2)
        //dpad right should return middle layer(index: 1)
        //dpad bottom should return bottom layer(index: 0)

        return 0;
    }

    public double getCurrentHeight()
    {
        return linearSlide1.getCurrentPosition() / ticksPerInch;
    }

    public void moveToMaxHeight()
    {
        targetHeight = maxHeight;
        moveToTargetPos();
    }

    public void moveToTargetPos()
    {
        linearSlide1.setTargetPosition((int)round(targetHeight * ticksPerInch));
        linearSlide2.setTargetPosition((int)round(targetHeight * ticksPerInch));
    }

    public void resetPos()
    {
        targetHeight = 0;

        linearSlide1.setTargetPosition(0);
        linearSlide2.setTargetPosition(0);
    }



}
