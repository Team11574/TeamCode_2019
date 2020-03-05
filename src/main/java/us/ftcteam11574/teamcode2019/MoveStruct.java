package us.ftcteam11574.teamcode2019;

public class MoveStruct {
    //structure for movement, tagged with "M"
    public double distance; public double angle;
    public double vx,vy,rot,rot2;
    public double[] control;
    MoveStruct(String line, String item_seperator,String line_seperator) {
        line = line.replace(line_seperator, "");
        String[] tokens = line.split(item_seperator);
        String first = tokens[0]; //coudl crash here, if there are no items
        if (first.equals("M")) {

            double[] gamepads = new double[4];
            for (int i = 0; 4 > i; i++) {
                gamepads[i] = Double.parseDouble(tokens[1 + i]);
                //read next 4, and parse to double
            }

            vx = gamepads[0];
            vy = gamepads[1];
            rot = gamepads[2];
            rot2 = gamepads[3];
            distance = Double.parseDouble(tokens[5]);
            angle = Double.parseDouble(tokens[6]);
            control = new double[5];
            for (int i = 0; 4 > i; i++) {
                control[i] = Double.parseDouble(tokens[7 + i]);
            }

        }
        else {
            //problem if this happens
            throw new RuntimeException("Wrong Argument inside of MoveStruct!!");
        }
    }



}
