package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Block Foundation Rewrite", group="Autonomous")
public class BlueBlockFoundation2 extends LinearOpMode {




    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;

    final int wall_buffer = 2300;//distance from wall after you back up, applies to all times after
    //was previously 500 when it worked
    //you have grabbed the black, hopefully will allow the robot to avoid the other robot
    final int line_dist_from_start = 1200; //distance from start to the line under the skybridge
    final int block_distance = 1700; //Distance from the middle block to either the rigth or left block
    final int block_distance_x = 780; //this needs to be different on both sides
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 1600; //Distance to move foward while grabbing the block
    final int foundation_dist = 4450;
    final int foundation_forward =2750-wall_buffer;
    final int foundation_side = 200;
    final int foundation_buff = 850; //buffer for moving it outwards
    final int foundation_back = 450;
    final int foundation_pull = 3600;
    final int x_dir = 1;



    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }

    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);

        Robot.pistonHome();
        Robot r = new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        auto.wall_buffer = 1800; //will be much closer
        while(!isStarted()) { //needs a godo way to exit out of the loop
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings
            auto.recognize_block();



        }
        auto.start(); //still need to check if these are correct


        telemetry.addData("pos guess",most_recent_position);
        telemetry.update();

        auto.grabBlockFastColor(move_from_wall,block_distance,block_forward_dist);
        //moveDirMax(0,1,0,(int) (block_forward_dist+(block_distance/Math.sqrt(2))-wall_buffer),4000);

        int extra_dist = (2-most_recent_position) * 600;
        //actually, we coudl turn here if we wanted to
        auto.turnOrient(0,x_dir,800);//doesnt need to be that long
        //turnOrient(0,1,2500,5500); //possibly faster
        //I could gain a lot of time if I was turning while moving I believe

        auto.moveDirMax(0,-1,0, (line_dist_from_start ),2000);
        //only if directoin is off by a certan amount
        //unused currently
        /*
        double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
        double ang_rot = Math.atan2(0, x_dir) - Math.PI / 2.0; //angle your game stick is facing

        double goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //

         */
        auto.turnOrient(0, x_dir, 800);//doesnt need to be that long
        //otherwise, skip this part

        //now we are at the line, now we want to be able ot place the block, should be pointed
        if (auto.most_recent_position == 2) {
            auto.moveDirMax(0,-1,0,foundation_dist+1600,5000,1,0);
        }
        else if(auto.most_recent_position == 1 ) {
            auto.moveDirMax(0,-1,0,foundation_dist+800,5000,1,0);
        }
        else {
            auto.moveDirMax(0,-1,0,foundation_dist,5000,1,0);
        }

        auto.turnOrient(-1,0,1000,0,0,0); //make this go a little faster
        Robot.resetTime();
        Robot.reset_pow();
        auto.moveDir(0,-.75,0,1000,3000,0,0,0); // a little bit more on the slow side, to ensur eit doesn't push it too much

        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.4, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 900) {
                Robot.runServoDown();
                Robot.setMotors(powers, 1);
                Robot.outtake();
            }
            Robot.resetTime();
            Robot.reset_pow();
        }
        auto.moveDirMax(0,1,0,foundation_back,3000,0,0,0);
        auto.turnOrient(1,0,2000,0,0,0);
        //moveDirMax(1,0,0,foundation_side,3000); //not sure if this is in the right direction
        //moveDirMax(1,0,0,2000,3000); //2000?

        auto.moveDir(0,.75,0,foundation_back+foundation_buff,4500,0,0,0);
        //this doesn't seem to work?
        //not exactly sure wha tto do here,
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1400 && !auto.timePast()  ) { //one second less, since running it earlier
                Robot.runServoDown();
            }
            Robot.reset_pow();
        }
        telemetry.addData("runtime",auto.extra_funcs.getRuntime());
        telemetry.update();
        //should it go max speed, or should it like ramp up, or something like that?
        auto.moveDirMax(0,-1,0,foundation_pull,3000,0,0,0);
        auto.turnOrient(0,-x_dir,3000,0,0,0);
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1500 && !auto.timePast()  ) { //one second less, since running it earlier
                Robot.runServoUp();
            }
            Robot.resetTime();
            Robot.reset_pow();
        }
        auto.moveDirMax(1,0,0,100,800,0,0,1);
        auto.moveDirMax(x_dir*.4,-1,0,2500,3000,0,0,1); //this line needs testing
        //need a bit of a strafe to ensure it gets into the corner


    }



    public class StopException extends RuntimeException {
        public StopException() {super();}
    }





}
