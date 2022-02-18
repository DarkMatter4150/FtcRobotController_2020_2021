package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

/**
 * Class to control the arm and capstone grabber. There is no direct access to the servo position for a reason.
 */
class OdometryDeploy(hardwareMap: HardwareMap) : AbstractSubsystem {
    private var leftServo: Servo
    private var rightServo: Servo
    private var backServo: Servo

    /**
     * Set positions for the am and claw.
     *
     * All of these values are derived from dinking around.
     *  DO NOT TOUCH THEM OR SIMON WILL TAKE YOUR KNEECAPS.
     */
    enum class ServoPositions(@JvmField val position: Double) {
        // Positions where the Odo pods are retracted
        RIGHT_UP(0.0),
        LEFT_UP(1.0),
        BACK_UP(0.0),
        // Positions where the Odo pods are extended to track
        RIGHT_DOWN(1.0),
        LEFT_DOWN(-1.0),
        BACK_DOWN(1.0);
    }

    /**
     * The position the bucket is at
     *
     * These aren't raw double values, they're values on the enum so that programmers don't break
     *  things
     */
    var rightPosition: ServoPositions = ServoPositions.RIGHT_DOWN
        set(value) {
            field = value
            rightServo.position = value.position
        }

    var leftPosition: ServoPositions = ServoPositions.LEFT_DOWN
        set(value) {
            field = value
            leftServo.position = value.position
        }

    var backPosition: ServoPositions = ServoPositions.BACK_DOWN
        set(value) {
            field = value
            backServo.position = value.position
        }

    init {
        // Grab the servo from the hardware map
        leftServo = hardwareMap.get(Servo::class.java, Servos.LEFT.name)
        rightServo = hardwareMap.get(Servo::class.java, Servos.RIGHT.name)
        backServo = hardwareMap.get(Servo::class.java, Servos.BACK.name)

        // If the servo is reversed in the config, reverse it
        if (Servos.LEFT.reversed) {
            leftServo.direction = Servo.Direction.REVERSE
        }
        if (Servos.RIGHT.reversed) {
            rightServo.direction = Servo.Direction.REVERSE
        }
        if (Servos.BACK.reversed) {
            backServo.direction = Servo.Direction.REVERSE
        }

        // Set the position to the rest position
        down()
        /*
        leftServo.position = ServoPositions.LEFT_DOWN.position
        rightServo.position = ServoPositions.RIGHT_DOWN.position
        backServo.position = ServoPositions.BACK_UP.position
         */
    }

    fun up() {
        rightPosition = ServoPositions.RIGHT_UP
        leftPosition = ServoPositions.LEFT_UP
        backPosition = ServoPositions.BACK_UP
    }

    fun down() {
        rightPosition = ServoPositions.RIGHT_DOWN
        leftPosition = ServoPositions.LEFT_DOWN
        backPosition = ServoPositions.BACK_DOWN
    }
}