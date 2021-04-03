package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="auto test again :)", group="Linear Opmode")
public class auto_test_again extends LinearOpMode{
    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        seven.InitCamera();
        seven.closeClaw();
        seven.OT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        double speed1 = 1;
        double speed2 = 0.5;
//        double speed1 = 0.375;
//        double speed2 = 0.25;


        //move to be able to see the stack size
        seven.closeClaw();
        //seven.armUp();
        seven.bbDrive(-4, speed1);
        seven.bbDrive(-34, speed1);
        seven.bbDrive(4, speed1);
        seven.bbDrive(34, speed1);
    }
}
