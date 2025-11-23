package org.firstinspires.ftc.teamcode.util;

public class Artifact {
    private String colour;
    private double position;

    public Artifact(String colour, double position) {
        this.colour = colour;
        this.position = position;
    }

    public String getColour(){
        return this.colour;
    }

    public double getPosition(){
        return this.position;
    }

}

