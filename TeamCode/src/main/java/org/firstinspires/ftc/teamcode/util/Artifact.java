package org.firstinspires.ftc.teamcode.util;

public class Artifact {
    private String colour;
    private int position;

    public Artifact(String colour, int position) {
        this.colour = colour;
        this.position = position;
    }

    public String getColour(){
        return this.colour;
    }

    public int getPosition(){
        return this.position;
    }

}

