package org.firstinspires.ftc.teamcode.robot.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Servos
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

/**
 * Bucket thingy on the lift. There is no direct access to the servo position for a reason.
 */
class Bucket(hardwareMap: HardwareMap) : AbstractSubsystem {
    private var bucketServo: Servo

    /**
     * Set positions for the bucket.
     *
     * All of these values are derived from dinking around.
     *  DO NOT TOUCH THEM OR SIMON WILL TAKE YOUR KNEECAPS.
     */
    enum class Positions(@JvmField val position: Double) {
        // Position when the bucket is dumping
        REST(0.0),
        // The vertical position
        FORWARD(0.4),
        BACKWARD(0.55),
        // The position when the bucket is all the way down
        DUMP(1.0),
        DUMP_HIGH(0.75),
        // The position when the bucket is initialized
        INIT(0.7);
    }

    /**
     * The position the bucket is at
     *
     * These aren't raw double values, they're values on the enum so that programmers don't break
     *  things
     */
    var position: Positions = Positions.REST
        set(value) {
            field = value
            bucketServo.position = value.position
        }

    init {
        // Grab the servo from the hardware map
        bucketServo = hardwareMap.get(Servo::class.java, Servos.BUCKET.name)

        // If the servo is reversed in the config, reverse it
        if (Servos.BUCKET.reversed) {
            bucketServo.direction = Servo.Direction.REVERSE
        }

        // Set the position to the rest position
        bucketServo.position = Positions.INIT.position
    }
}