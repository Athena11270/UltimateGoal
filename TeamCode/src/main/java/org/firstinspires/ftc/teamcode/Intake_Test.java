package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Intake Test", group="Linear Opmode")
public class Intake_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotor LR = null;
    //private DcMotor LL = null;
    private DcMotor OT = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //LR = hardwareMap.get(DcMotor.class, "LR");
        //LL = hardwareMap.get(DcMotor.class, "LL");
        OT = hardwareMap.get(DcMotor.class, "OT");
        //yoo are ___
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        //LR.setDirection(DcMotor.Direction.FORWARD);
        //LL.setDirection(DcMotor.Direction.REVERSE);
        OT.setDirection(DcMotorSimple.Direction.FORWARD);
        OT.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double IPower = gamepad1.left_stick_y;

            //LL.setPower(IPower);
            //LR.setPower(IPower);

            OT.setPower(gamepad1.right_stick_y);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //telemetry.addData("Motors", "IL (%.2f), IR (%.2f), OT (%.2f),", LL.getPower(), LR.getPower(), OT.getPower());
            telemetry.addData("Motor", "OT (%.2f",OT.getPower());
            telemetry.update();
        }
    }
}
