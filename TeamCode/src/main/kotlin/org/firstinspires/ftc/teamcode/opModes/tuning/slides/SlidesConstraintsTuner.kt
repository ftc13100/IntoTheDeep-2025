import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.teamcode.constants.ControlBoard
import org.firstinspires.ftc.teamcode.constants.SlidesConstants
import org.firstinspires.ftc.teamcode.subsystems.slides.OpenElevatorSubsystem

@Autonomous
class SlidesConstraintsTuner : LinearOpMode() {
    private lateinit var flipperServo: Servo
    private lateinit var leftMotor: Motor
    private lateinit var rightMotor: Motor

    private lateinit var limit: TouchSensor

    private lateinit var elevator: OpenElevatorSubsystem

    private var lastPos = 0.0
    private var lastVel = 0.0

    private var maxVel = 0.0
    private var maxAccel = 0.0

    private val timer = ElapsedTime(ElapsedTime.Resolution.SECONDS)
    override fun runOpMode() {
        leftMotor = Motor(hardwareMap, ControlBoard.SLIDES_LEFT.deviceName)
        rightMotor = Motor(hardwareMap, ControlBoard.SLIDES_RIGHT.deviceName)

        elevator = OpenElevatorSubsystem(leftMotor, rightMotor)
        waitForStart()

        var dt = 0L
        var dv = 0.0

        timer.reset()
        while (opModeIsActive()) {
            while (timer.seconds() < TIME_TO_RUN) {
                elevator.up()

                dt = timer.nanoseconds() - dt
                dv = elevator.currentVel - dv / dt

                maxVel = elevator.currentVel.coerceAtLeast(maxVel)
                maxAccel = dv.coerceAtLeast(maxAccel)

                telemetry.addData("Max Velocity", maxVel)
                telemetry.addData("Max Acceleration", maxAccel)
                telemetry.update()


                if (isStopRequested) break
            }

            telemetry.addData("Final Max Velocity", maxVel)
            telemetry.addData("Final Max Acceleration", maxAccel)
            telemetry.update()
        }
    }

    companion object {
        @JvmField
        var TIME_TO_RUN = 2.0
    }
}