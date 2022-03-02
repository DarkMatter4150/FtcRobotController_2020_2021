package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

/**
 * Class to control the arm and capstone grabber. There is no direct access to the servo position for a reason.
 */
class Capper(hardwareMap: HardwareMap) : AbstractSubsystem {
    private var armServo: Servo
    private var clawServo: Servo

    /**
     * Set positions for the am and claw.
     *
     * All of these values are derived from dinking around.
     *  DO NOT TOUCH THEM OR SIMON WILL TAKE YOUR KNEECAPS.
     */
    enum class ArmPositions(@JvmField val position: Double) {
        // Position when the arm is at rest on the slides
        REST(0.0),
        // The Position to grab the Team Scoring Element
        DOWN(0.9),
        //The position to cap
        UP(0.7),

        // The position when the arm is initialized
        INIT(0.15);
    }

    enum class ClawPositions(@JvmField val position: Double) {
        // Position when the claw is in a neutral position
        REST(0.0),
        // The position where the claw is all the way open
        OPEN(0.9),
        //The position where the claw is all the way closed
        CLOSE(0.63),
        // The position when the claw is initialized
        INIT(0.6);
    }

    /**
     * The position the bucket is at
     *
     * These aren't raw double values, they're values on the enum so that programmers don't break
     *  things
     */
    var armPosition: ArmPositions = ArmPositions.REST
        set(value) {
            field = value
            armServo.position = value.position
        }

    var clawPosition: ClawPositions = ClawPositions.REST
        set(value) {
            field = value
            clawServo.position = value.position
        }

    init {
        // Grab the servo from the hardware map
        armServo = hardwareMap.get(Servo::class.java, Servos.ARM.name)
        clawServo = hardwareMap.get(Servo::class.java, Servos.CLAW.name)

        // If the servo is reversed in the config, reverse it
        if (Servos.ARM.reversed) {
            armServo.direction = Servo.Direction.REVERSE
        }
        if (Servos.CLAW.reversed) {
            clawServo.direction = Servo.Direction.REVERSE
        }

        // Set the position to the rest position
        armServo.position = ArmPositions.INIT.position
        clawServo.position = ClawPositions.INIT.position
    }
}