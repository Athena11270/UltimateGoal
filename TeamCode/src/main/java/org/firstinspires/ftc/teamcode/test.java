package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="test", group="Linear Opmode")
public class test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor OT = null;

    @Override
    public void runOpMode() {
        OT = hardwareMap.get(DcMotor.class, "OT");
        OT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();
        runtime.reset();
        double target = 85;
        double power = 1;
        double tpr = 28;

        OT.setPower(power);
        sleep(1000);
        double prevTick = OT.getCurrentPosition();
        double deltaTick = 0;
        sleep(50);
        double prps = target;
        while (opModeIsActive()) {

            deltaTick = OT.getCurrentPosition() - prevTick;
            prevTick = OT.getCurrentPosition();

            double rps = (deltaTick/tpr)*10;
            double rpsfinal = (rps + prps)/2;
            prps = rps;

            power = power * Math.sqrt(target/rps);
            OT.setPower(power);

            telemetry.addData("Power", power);
            telemetry.addData("RPS", rpsfinal);
            telemetry.addData("Target", target);
            telemetry.update();
            sleep(100);
        }
    }
}
