package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

// currently TeleOp, but will really only be used in Autonomous
@Disabled
@TeleOp(name="MKTest", group="")
public class MKTest extends LinearOpMode {

    /* Declare OpMode members. */
    MKTestHardware robot   = new MKTestHardware(this);

    @Override
    public void runOpMode() {

        robot.initialize();
        robot.InitCamera();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // detect stack size
        String stackSize = robot.DetectStackSize();
        robot.CloseCamera();
        telemetry.addData("*** STACK SIZE: ", stackSize);
        telemetry.update();
        sleep(2000); // JUST SO WE CAN SEE THE RESULT - DO NOT DO IN COMPETITION CODE

        // call method to drive pre-loaded wobble goal to correct place

        // maybe shoot three pre-loaded rings into score thingy

        // maybe go get other wobble goal, bring to correct place

        // park on line
    }
}

