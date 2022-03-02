package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem

@Config
class Intake(hardwareMap: HardwareMap) : AbstractSubsystem {
    // This subsystem has two motors because each motor is useless without the other
    private val intakeMotor: DcMotorEx


    companion object{
        @JvmField var coefficient = .7
    }

    var power: Double
        /**
         * Positive values push stuff out, negative values pull stuff in (intake)
         */
        set(value) {
            intakeMotor.power = Range.clip(value, -1.0, 1.0) * coefficient
        }
        get() {
            return intakeMotor.power / coefficient
        }

    init {
        // Set up the intake motor
        intakeMotor = hardwareMap.get(DcMotorEx::class.java, Motors.INTAKE.name)
        intakeMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        intakeMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        intakeMotor.power = 0.0

    }
}