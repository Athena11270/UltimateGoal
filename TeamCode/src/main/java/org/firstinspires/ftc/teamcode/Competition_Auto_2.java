package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Autonomous 2", group="Linear Opmode")
public class Competition_Auto_2 extends LinearOpMode{
    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        seven.InitCamera();
        seven.closeClaw();
        seven.OT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

//        double speed1 = 0.75;
//        double speed2 = 0.5;
//        double speed1 = 0.375;
//        double speed2 = 0.25;
        double speed1 = 0.6;
        double speed2 = 0.4;


        //move to be able to see the stack size
        seven.closeClaw();
        //seven.armUp();
        seven.drive(-38, speed1);


        //read the stack
        String stackSize = seven.DetectStackSize();
        telemetry.addData("Stack Size", stackSize);
        telemetry.update();
        seven.CloseCamera();

        seven.drive(-18, speed1);
//        seven.strafeL(16, speed2); //strafe right to align cam w/ rings
//        seven.strafeR(3, speed2);

        int wait = 400;

        seven.strafeR(18, speed1/2);

        // spin up
        seven.launcherMono(1);
        sleep(wait*3);

        // fire all three times
        for (int n = 0; n <= 3; n++) {
            seven.bumper(true);
            sleep(wait);
            seven.bumper(false);
            sleep(wait);
        }
        // power back down
        seven.launcherMono(0);

        seven.turn(-90, speed1);
        seven.turn(-90, speed1);

        int strafeAdd = 24;
        int forwardAdd = 0;
        if(stackSize.equals(SevenTheRobot.RINGS_FOUR)) {
            forwardAdd = 48;
        }

        else if(stackSize.equals(SevenTheRobot.RINGS_ONE)) {
            strafeAdd = 0;
            forwardAdd = 24;
        }

        seven.strafeL(strafeAdd, speed2);
        seven.drive(forwardAdd, speed1);
        seven.armUp();
        sleep(wait);
        seven.armDown();
        sleep(wait*2);
        seven.openClaw();
        sleep(wait);
        seven.turn(-30, speed1);
        seven.armUp();
        sleep(wait);
        seven.turn(30, speed1);
        seven.drive(-forwardAdd, 1);
        seven.strafeR(strafeAdd, 1);
        seven.drive(12, 1);
        seven.Arm.setTargetPosition(0);
        sleep(1000);

        //reminder: we're driving backwards, probably have to turn to drop in C
//        if (stackSize.equals(seven.RINGS_FOUR)){
//            //move to zone C, drop wobble (15 points), move to shooting position
//            seven.drive(-24, 0.5);
//            //turn 180 :)
//            seven.drive(72, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.drive(-40, 0.5);
//            //turn 180 again :)
//            seven.strafeL(12, 0.5); //get to shot zone
//
//            //now to blast it
//            seven.launcherMono(1);
//            //bonk ring1 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//            //reposition
//            //bonk ring 2 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//            //reposition
//            //bonk ring 3 & reset
//            seven.BP.setPosition(0);
//            sleep(100);
//            seven.BP.setPosition(0.55);
//
//            //scoot to launch line
//
//        }
//        else if (stackSize.equals(seven.RINGS_ONE)){`
//            //move to zone B, drop wobble (15 points), move to shooting position
//            seven.drive(-56, 0.5);
//            seven.strafeL(20, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.drive(32, 0.5);
//            seven.strafeL(12, 0.5);
//        }
//        else if (stackSize.equals(seven.RINGS_NONE)){
//            //move to zone A, drop wobble (15 points), move to shooting position
//            seven.drive(-24, 0.5);
//            seven.armDown();
//            seven.openClaw();//lower arm, drop wobble
//            seven.strafeL(32, 0.5); //move laterally to power shot zone
//        }
//        else {
//            //uh idk I mean there's an error if we got here,
//            //so we should just do the easiest goal and pray for the best I guess
//        }

        //either we do the high goal and run the same thing thrice (12/24/36 points)
        //or we shoot & adjust thrice to hit the power shots (15/30/45 points)
        //move to park on line (5 points)
        //fully optimized, that gives us 65 points (assuming power shots).
    }
}
