package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Blue Foundation Block", group="Autonomous")
public class BlueFoundationBlock extends LinearOpMode {




    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;

    final int wall_buffer = 2100;//distance from wall after you back up, applies to all times after
    //you have grabbed the black, hopefully will allow the robot to avoid the other robot
    final int line_dist_from_start = 2000; //distance from start to the line under the skybridge
    final int block_distance = 1700; //Distance from the middle block to either the rigth or left block
    final int block_distance_x = 780;
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 1600; //Distance to move foward while grabbing the block
    final int extra_dist = 2850;
    final int dir = 1;          //direction to move after grabbing the foundation
    final int move_dist = 200; //distance to move to the dir after grabbing the foundation




    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof Robot.StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }
    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap, gamepad1, gamepad2);
        //Robot.pistonHome(); //coudl include this?

        Robot r = new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        Robot.pistonHome();
        while (!isStarted()) {

            //wait

        }
        Robot.resetTime();
        Robot.reset_pow();
        auto.moveDirMax(0,-1,0,1800,3000,0,0,-.8); //run downwards
            //the speed should be based on how long it takes to get there
        auto.moveDirMax(1,0,0,400,500,0,0,-.6);
        auto.orientToFoundation(0,600);
        auto.moveFoundation(1500,-1);
        //let go of foundation
        {

            Robot.resetTime();
            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1000) {
                Robot.runServoUp();
            }
            Robot.resetTime();
            Robot.reset_pow();
        }
        auto.moveDirMax(dir,0,0,move_dist,1000); //no need, in good spot
        auto.moveDirMax(0,-1,0,(int) (line_dist_from_start+block_distance_x*4.3),8000,-1,0,0);
        //should be about the right distance
        auto.turnOrient(-1,0,1000);
        //auto.moveDirMax(0,1,0,2000,1000,-1,0);
        //auto.recognize_block(); //not enough time currently
        //auto.grabBlock(move_from_wall,block_distance,block_forward_dist);
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.6, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!auto.checkDone(3000)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake(.80); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
        }
        auto.moveToFoundationY(1,(int) (line_dist_from_start+block_distance_x*4.3),extra_dist);


        //drop o


    }



}
