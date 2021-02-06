package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.SevenTheRobotVision.RINGS_FOUR;
import static org.firstinspires.ftc.teamcode.SevenTheRobotVision.RINGS_NONE;
import static org.firstinspires.ftc.teamcode.SevenTheRobotVision.RINGS_ONE;

@Autonomous(name="PsuedoAuto", group="Linear Opmode")
public class PsuedoAuto extends LinearOpMode{
    @Override
    public void runOpMode() {
        SevenTheRobot seven = new SevenTheRobot(this);
        seven.initialize();
        //seven.InitCamera();
        waitForStart();

        //move to be able to see the stack size

        //read the stack
        /*String stackSize = seven.DetectStackSize();
        seven.CloseCamera();*/

        //if (stackSize.equals(RINGS_FOUR)){
            //move to zone C, drop wobble (15 points), move to shooting position
        //}
        //else if (stackSize.equals(RINGS_ONE)){
            //move to zone B, drop wobble (15 points), move to shooting position
        //}
        //else if (stackSize.equals(RINGS_NONE)){
            //move to zone C, drop wobble (15 points), move to shooting position
        //}
        //else {
            //uh idk I mean there's an error if we got here,
            //so we should just do the easiest goal and pray for the best I guess
        //}

        //either we do the high goal and run the same thing thrice (12/24/36 points)
        //or we shoot & adjust thrice to hit the power shots (15/30/45 points)
        //move to park on line (5 points)
        //fully optimized, that gives us 65 points (assuming power shots).
    }
}
