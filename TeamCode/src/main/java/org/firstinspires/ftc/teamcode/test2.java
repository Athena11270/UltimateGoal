package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="test2", group="Linear Opmode")
public class test2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor OT = null;

    @Override
    public void runOpMode() {
        OT = hardwareMap.get(DcMotor.class, "OT");
        OT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        runtime.reset();
        double power = 1;
        double tpr = 28;

        OT.setPower(power);
        sleep(1000);
        double prevTick = OT.getCurrentPosition();
        double deltaTick = 0;
        sleep(50);
        double prps = 100;

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive()) {

            deltaTick = OT.getCurrentPosition() - prevTick;
            prevTick = OT.getCurrentPosition();


            double rps = (deltaTick/tpr)/timer.time();
            timer.reset();
            double rpsfinal = (rps + prps)/2;
            prps = rps;

            power += gamepad1.left_stick_y/5000;
            OT.setPower(power);

            telemetry.addData("Power", power);
            telemetry.addData("RPS", rpsfinal);
            telemetry.update();
        }
    }
}
