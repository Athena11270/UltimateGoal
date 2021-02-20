package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="input test", group="Linear Opmode")
public class inputtest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("A", gamepad1.a + "; B: " + gamepad1.b + "; X: " + gamepad1.x + "; Y: " + gamepad1.y);
            telemetry.addData("Up", gamepad1.dpad_up + "; Left: " + gamepad1.dpad_left + "; Right: " + gamepad1.dpad_right + "; Down: " + gamepad1.dpad_down);
            telemetry.addData("AS", "LS[" + gamepad1.left_stick_x + ", " + gamepad1.left_stick_y + "]; RS[" + gamepad1.right_stick_x + ", " + gamepad1.right_stick_y + "]");
            telemetry.addData("BP", "L: " + gamepad1.left_bumper + "; R: " + gamepad1.right_bumper);
            telemetry.addData("TR", "L: " + gamepad1.left_trigger + "; R: " + gamepad1.right_trigger);
            telemetry.update();
        }
    }
}
