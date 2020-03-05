package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import android.content.Context;
import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.stream.Stream;

//TODO: Besidses getting everythign working, another task is to get the rest of the modes finished and
//TODO: to get the string to java file thing working
//TODO: that implicates that I get workin g a general function, maybe I can make a Robot Iterative taht works for a litenear op mode, maybe continue updating until
//TODO: Its reached its done stare. I will also need that watchdog 28.5 second timer I I deide to do that
//TODO: I could also check the drive lines, becuase those likely have problems, but its probably easer to debug if I can test
//TODO: Also, it important that I test he ramp up driving, mayb eadd it to an autonomous and see how consistant or inconsistant it ends up driving our robot
//

@TeleOp(name="Autonomous Creation", group="Iterative Opmode")
public class AutoCreation extends OpMode {

    //creates an autonomous mode easier, and saves data to a file
    //should have an interface to be easy to use
    @Override
    public void init() {


        Robot.init_teleOp(telemetry, hardwareMap,gamepad1,gamepad2);
        previousButton = opNoDistModes();
        previousControl = controlButtons();
        telemetry.addData("Note:","Entering Autonomous Creation Op Mode");
        //information on bindings go here, add all the data about it the telop mode
        telemetry.update();
    }
    String[] modes =new String[]{"Record Movement","Debug Motion"};
    //debug motion should test a bunch of things, such as moving with ramp up vs moving without ramp upd
    //should do some motion, and see if it ends on the same spot
    //Maybe test odemetry, strafe back and forth
    //diagnals, and some random angle there and back
    //maybe also test out turn while moving
    String loading = "|/-";

    static String line_seperator = "&";
    static String item_seperator = ",";
    int file = 0;

    String log = "";

    int mode = -1;
        int submode = -1;
            int submode2 = -1;
                int submode3 = -1;
    int frame = 0;
    //mode: -1
        int pntr = 0;
    //mode: 0
        boolean start = false;
        boolean running = false;
        boolean file_written = false;
        double previous_angle = 0;
        double[] previousJoyStick = new double[4];
        boolean[] previousButton;
        double[] previousControl;
        opNoDist op= new opNoDist();
        ArrayList<String> saveData = new ArrayList<>(100);
        int mode_path = 0;
        RobotIterative robot_move = null;
        int task_id = 1;
        boolean[] mode_buttons = new boolean[]{true, false, false}; //default mode
        double previous_ang = 0;
        int sides = 4;

    @Override
    public void loop() {
        frame++;
        updateButtonInfo();
        if (mode == -1) { //menu
            {
                if (dpad_right) {
                    pntr += 1;
                }
                else if (dpad_left){
                    pntr -= 1;
                }

                if (pntr < 0) {
                    pntr = 0;
                }
                pntr %= modes.length;
            }
            String str = "";
            for (int i = 0; modes.length > i; i++) {
                if (pntr == i) {
                    str += ">";
                }
                str += modes[i];
                str += "    ";
            }
            telemetry.addData("Which mode would you like to enter?", str);
            if (gamepad1.a) {
                mode = pntr; //set mode to pointer, and now get ready to enter that mdoe
                telemetry.update();
                telemetry.addData("Entered mode:", modes[pntr]);
                telemetry.update();
                //sleep(2500); //wait, so the user can read the message
                pntr = 0;
            }
            telemetry.update();
        }
        else if (mode == 0) { //Record Movement
            if (submode == -1) {
                submode_neg_1();
            }
            if(submode == 0) { //Joystick Input
                if (!start) {
                    zero_nostart();
                }
                else {
                    //now we can start recording the joystick
                    //we also have to simulate robot movement
                    if (running) {
                        running_loop();
                    }
                    else {
                        if (!file_written) {
                           saveFiles();
                        }
                        else {
                            int mode = -1;
                            if (submode2 == -1) {
                                mode = modeInterface(new String[]{ "Run path backwards","Continue path"});
                            }
                            if(mode != -1) {
                                submode2 = mode;
                            }
                            if(submode2 != -1) {
                                if(submode2 == 0) { //Run backwards
                                    runBackLoop();
                                    if(frame % 10 == 0) {
                                        saveLog();
                                    }
                                }
                                else if (submode2 == 1) { //COntinue path

                                }
                            }

                            //now in another menu for chosing what to do

                        }

                        //now, we can prompt the user whether it wants to run the path backwards, or continue the path further
                        //also, whether to append or to write a new file
                        //
                    }

                    //now that the robot is running, we have to analyze the data
                }
            }
            if(submode == 1) { //Numerical Input

            }

            //A couple options within this
                //Record motion spline (This will proabably be the main one)
                    //Spline using: Holonomic Algorithm
                    //Spline using: NonHolonmic Algorithm
                    //Spline using: General Search Algorithm (For this, I'll probably have to work with Jeremy Cole to figure out something
                //Record User motion
                    //Joystick Input motion--using the joystick, record all motoins, and replay them back
                    //Numerical Input motion--Use an interface to type in numbers and test those out
            //Note:
                //make sure to include a reset positions, reset encoders, reset gyro, etc
                    //Also, save files are SUPER important, need to be easily accessible and exportable
        }
        else if( mode == 1) { //Debug motion
            if(submode2 == -1) {
                int tmp_mode = -1;
                tmp_mode = modeInterface(new String[]{"Polygon Motion"});
                if (tmp_mode != -1) {
                    submode2 = tmp_mode;
                }
            }
            if(submode2 == 0) {
                //then we test the different directions
                if(submode3 == -1) {
                    int tmp_mode = -1;
                    tmp_mode = modeInterface(new String[]{"Use Ramp Up","Don't use Ramp Up"});
                    if (tmp_mode != -1) {
                        submode3 = tmp_mode;
                        robot_move = null; //make robot_move null

                    }
                }
                if(submode3 == 0 || submode == 1) {
                    if(!polygonTestLoop(submode3==0,sides)) {

                    }
                    else{
                        robot_move = null;
                        sides++;
                        Robot.reset_pow();
                        Robot.resetTime();
                        sleep(1000); //wait a little bit between running
                    }

                    //then use ramp up to test
                    //Could run a square test
                    //Turn test
                    //N gon test, draws out a polygon with a certain number of sides

                }


            }
            //lets tackle this one first:
            //Move in a bunch of directions, record data in a calibration file
            //Maybe I can test using ramp up vs not using ramp up

        }





    }
    public static String headerProgram() {
        String res = "package us.ftcteam11574.teamcode2019;\n" +
                "\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.OpMode;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.TeleOp;\n" +
                "import com.qualcomm.robotcore.util.ElapsedTime;\n";
        res += "import com.qualcomm.robotcore.eventloop.opmode.Autonomous;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.Disabled;\n" +
                "import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;\n\n\n";
        res += "@Autonomous";
        res += "public class NAME extends LinearOpMode {\n";
        res +="@Override\n" +
                "    public void runOpMode() {\n";
        res +="boolean[] mode_buttons = new boolean[]{true, false, false}; //default mode\n";
        //note: one short on brackets here


        //then need to extend all the requiered functions


        //import statements at the top

        return res;
    }
    public static String dataToProgram(ArrayList<String> saveData) {
        //The header section is here
        //TODO: include an argument that takes in tabs so I can make everythign auto formatted
        String res = headerProgram();
        for (int i = 0; saveData.size() > i; i++) {
            RobotIterative robot_move = new RobotIterative(saveData.get(i),item_seperator,line_seperator);
            res += robot_move.dataToJavaFile();
            //add this part of the thing to the java file
            //that should be everythign required
        }
        res += "}\n}";
        return res;
    }

    int modeInterface(String[] modes) {
        int out = -1;
        String str = "";
        for (int i = 0; modes.length > i; i++) {
            if (pntr == i) {
                str += ">";
            }
            str += modes[i];
            str += "    ";
        }
        telemetry.addData("Which mode would you like to enter?", str);
        if (gamepad1.a) {
            out = pntr; //set mode to pointer, and now get ready to enter that mdoe
            telemetry.update();
            telemetry.addData("Entered mode:", modes[pntr]);
            telemetry.update();
            //sleep(2500); //wait, so the user can read the message
            pntr = 0;
        }
        telemetry.update();
        {
            if (dpad_right) {
                pntr += 1;
            }
            else if (dpad_left){
                pntr -= 1;
            }

            if (pntr < 0) {
                pntr = 0;
            }
            pntr %= modes.length;
        }
        return out;
    }


    boolean dpad_left = false;
    boolean dpad_right = false;
    boolean a_cur = false;
    public void updateButtonInfo() {

        if (Robot.gamepad1.dpad_left){
            if ( !dpad_left ) {
                dpad_left = true;

            }
        }
        else {
            if (dpad_left) {
                dpad_left = false;
            }
        }
        if (Robot.gamepad1.dpad_right){
            if ( !dpad_right ) {
                dpad_right = true;

            }
        }
        else {
            if (dpad_right) {
                dpad_right = false;
            }
        }
        if (Robot.gamepad1.a){
            if ( !a_cur ) {
                a_cur = true;

            }
        }
        else {
            if (a_cur) {
                a_cur = false;
            }
        }


    }
    public boolean[] opNoDistModes() {
        boolean[] buttons = new boolean[3];
        buttons[0] = op.max_mode;
        buttons[1] = op.orientation_mode;
        buttons[2] = op.slow_intake;
        return buttons;
    }
    public double[] controlButtons() {
        double[] button_info = new double[5];
        button_info[0] = Robot.gamepad1.left_trigger;
        button_info[1] = Robot.gamepad1.right_trigger;
        if(gamepad1.right_bumper) {
            button_info[2] = -1;
        }
        else if(gamepad1.left_bumper) {
            button_info[2] = 1;
        }
        else {
            button_info[2] = 0;
        }
        button_info[3] = op.pistonPos;
        if(Robot.gamepad1.dpad_down) {
            button_info[4] = -1;
        }
        else if (Robot.gamepad1.dpad_up) {
            button_info[4] = 1;
        }
        else {
            button_info[4] = 0;
        }
        //probbly want to also include control for the foundation grabber
        //button_info[4] =
        return button_info;
    }




    //create some interface functions


    private void writeToFile(String content, String name) {
        try {
            File file = new File(Environment.getExternalStorageDirectory() + "/" + name + ".txt");

            if (!file.exists()) {
                file.createNewFile();
            }
            FileWriter writer = new FileWriter(file);
            writer.append(content);
            writer.flush();
            writer.close();
        } catch (IOException e) {
            telemetry.addData("failed to upload file","");
            telemetry.update();
            sleep(2000);
        }
    }
    void sleep(int max_time) {
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < max_time) {
            //nothing
        }
        Robot.resetTime();
    }
    public boolean double_eq(double[] a, double[] b) {
        if(a.length != b.length) {
            return false;
        }
        for (int i = 0; a.length > i; i++) {
            if (a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }
    public boolean boolean_eq(boolean[] a, boolean[] b) {
        if(a.length != b.length) {
            return false;
        }
        for (int i = 0; a.length > i; i++) {
            if (a[i] != b[i]) {
                return false;
            }
        }
        return true;
    }
    public void submode_neg_1() {
        String[] modes = new String[]{"Joystick Input", "Numerical Input"};
        String str = "";
        {
            if (dpad_right) {
                pntr += 1;
            }
            else if (dpad_left){
                pntr -= 1;
            }

            if (pntr < 0) {
                pntr = 0;
            }
            pntr %= modes.length;
        }
        for (int i = 0; modes.length > i; i++) {
            if (pntr == i) {
                str += ">";
            }
            str += modes[i];
            str += "    ";
        }
        telemetry.addData("Which mode would you like to enter? (gamepad b to select)", str);
        if (gamepad1.b) {
            submode = pntr; //set mode to pointer, and now get ready to enter that mdoe
            telemetry.update();
            telemetry.addData("Entered mode:", modes[pntr]);
            telemetry.update();
            pntr = 0;

            //sleep(2500); //wait, so the user can read the message
        }
        telemetry.update();
    }
    public void zero_nostart() {
        telemetry.addData("Press x to start recording, and x again to stop (on gamepad2)", loading.charAt((frame / 50) % loading.length()));
        telemetry.update();
        if (gamepad1.x || gamepad2.x) {
            start = true;
            running = true;
            opNoDist op = new opNoDist();
            //sleep(1000);
            op.init(telemetry, hardwareMap,gamepad1,gamepad2);
            op.start();
            Robot.reset_pow();
            Robot.update_position();

            //Robot.

        }
    }
    public void running_loop() {
        file_written = false;
        if (gamepad2.x || gamepad1.x) { //TODO: change this to only read for if gamepad2.x is pressed, but for now, jsut testing to make sure it works, since gamepad2 isn't working
            running = false;
        }
        double[] gamepad = Robot.scaleJoyStick();
        telemetry.addData("vx",previousJoyStick[0]);
        telemetry.addData("vy",previousJoyStick[1]);
        telemetry.addData("current angle",Robot.angle());
        double d = 0;
        for (int i = 0; Robot.positions.length > i; i++) {
            d += Math.abs(Robot.motors[i].getCurrentPosition() - Robot.positions[i]); //absolute vlaue to ensure they don't cancel each other out
        }
        telemetry.addData("current distance",d);
        boolean[] button = opNoDistModes();
        double[] control = controlButtons();
        op.run(); //I think this will work to run everything
        if (!double_eq(gamepad, previousJoyStick) || !double_eq(previousControl, control)) { //if not equal, then we save all the previous data into the file, or if control buttons change
            //double[] gamepad_input = previousJoyStick; //Do I want to save the joystick or the x,y,r info
            double angle = Robot.angle(); //same robot.angle
            //maybe make it so it checks after or something
            //previous_angle = Robot.angle(); //actually, want the angle after, so not previous angle
            double distance = 0;
            for (int i = 0; Robot.positions.length > i; i++) {
                distance += Math.abs(Robot.motors[i].getCurrentPosition() - Robot.positions[i]); //absolute vlaue to ensure they don't cancel each other out
            }
            distance /= 4.;

            String write = "M" + item_seperator; //for movement
            for (int i = 0; previousJoyStick.length > i; i++) {

                write += previousJoyStick[i]; //
                write += item_seperator; //line seperator
            }
            write += distance + item_seperator;
            write += angle + item_seperator;
            //also write the data from button control
            for (int i = 0; previousControl.length > i; i++) {
                write += previousControl[i] + item_seperator; //write all the button values to the system
            }
            write += line_seperator;
            saveData.add(write);//write the data to the file
            Robot.update_position(); //now, the current position is the zero position for the encoders
            previous_angle = Robot.angle();
            //TODO
            //include a play back button, that attempts to go back to where it starts
            //This should actually include also readings for the pantagraph, intake, and foundation grabber at the same time


        }
        if (!boolean_eq(button, previousButton)) {//Then we save the updated info of the modes
            boolean[] button_info = button;
            String write = "B" +item_seperator; //For button
            for (int i = 0; button_info.length > i; i++) {
                write += button[i] ? "T" : "F"; //True or false
                write += item_seperator;
            }
            write += line_seperator; //n
            saveData.add(write);
            //This updates to show that the modes have changed
        }


        previousButton = button;
        previousJoyStick = gamepad;
        previousControl = control;
    }
    public void runBackLoop() {
        telemetry.addData("testA",saveData.size());

        if(robot_move == null) {
            //note, starting from the back, sine want to do the most recent item backwards first, and finally, the last item
            if(task_id+1 < saveData.size()) {
                robot_move = new RobotIterative(saveData.get(saveData.size() - task_id++), item_seperator, line_seperator);
                robot_move.update();
                robot_move.backwards = true;
                robot_move.set_mode(mode_buttons);
            }
            //intialize the robot if its not already initialized
        }
        else if(!robot_move.done()) { //instatly getting done for some reason
            robot_move.update();

            telemetry.addData("cur ",saveData.size() - task_id);


            telemetry.addData("Value of input:" ,saveData.get(task_id));
            telemetry.addData("max dist distance:" ,robot_move.max_dist);
            if(!robot_move.angle_done) { //only show this if in teh angle done section
                log += "\nGOAL:" + robot_move.angle;
                log += "\nPRE ERROR:" + robot_move.angDiff();
                log += "\nPRE ANGLE:" + Robot.angle();
                log += "\nPRE ANGLE:" + Math.abs(Robot.motors[0].getPower() + Robot.motors[1].getPower() + Robot.motors[2].getPower() + Robot.motors[3].getPower());
                telemetry.addData("angle error:", robot_move.angDiff());
            }
            else {
                telemetry.addData("angle done--rotate dif:", robot_move.angDiff());
                log += "\nTURNING ERROR:" + Robot.distanceTurn(Robot.angle(),(new MoveStruct(saveData.get(saveData.size() - task_id),item_seperator,line_seperator)).angle);
                log += "\nWITH ROT:" + robot_move.rot;
                log += "\nGOAL TURN:" + (new MoveStruct(saveData.get(saveData.size() - task_id),item_seperator,line_seperator)).angle;
                log += "\nPost ANGLE:" + Robot.angle();
            }
            telemetry.update();

        }
        else {
            telemetry.addData("done:","done");
            if(task_id+1 < saveData.size()) {
                mode_buttons = robot_move.return_mode();
                robot_move = new RobotIterative(saveData.get(saveData.size()-task_id), item_seperator,line_seperator);
                task_id += 1;
                robot_move.start();
                robot_move.backwards = true;
                robot_move.set_mode(mode_buttons);
                Robot.reset_pow(); //maybe test if this works, seems weird
                log += "\n----------";
            }
            else {
                telemetry.addData("done:","with auto thing");
                Robot.reset_pow();
                submode = -1;

            }
            telemetry.update();
            //after this, then it should be do
        }
    }
    public void saveFiles() {
        String write_to_file = "";
        for (int i = 0; saveData.size() > i; i++) {
            write_to_file += saveData.get(i);
        }

        writeToFile(write_to_file, "testSaveFile" + file++);
        writeToFile(dataToProgram(saveData), "testSaveFileJava" + file++); //writes also a java program to a file

        telemetry.addData("Succesfully saved data", "");
        telemetry.update();
        file_written = true;
        robot_move = null;
    }
    public void saveLog() {
        writeToFile(log,"logFile");
    }
    public boolean polygonTestLoop(boolean ramp_up, int sides) {

        if(robot_move == null) {
            task_id = 0;
            initRPoly(ramp_up, sides, task_id);
        }
        else if(!updateRobot()) {

        }
        else {
            //then done
            if(task_id+1 < sides) {
                initRPoly(ramp_up, sides, (++task_id)); //incremeent id
            }
            else {
                return true;
            }
        }
        return false;






    }
    public void initRPoly(boolean ramp_up, int sides, int id) {
        double ang_diff =(Math.PI*2)/sides;
        double cur_ang = ang_diff*id;
        double vx = Math.cos(cur_ang);
        double vy = Math.sin(cur_ang);
        double rot = -0;
        robot_move = new RobotIterative(vx,vy,rot,0,500,0,1,1500,3500,0,0,0,0);
        robot_move.update();
        robot_move.angle_done=true; //ensure we don't do anything with checking angles
        robot_move.ramp_up=ramp_up;
    }
    public boolean updateRobot() {
        if(!robot_move.done()) { //instatly getting done for some reason
            robot_move.update();
        }
        return robot_move.done();
    }



}
