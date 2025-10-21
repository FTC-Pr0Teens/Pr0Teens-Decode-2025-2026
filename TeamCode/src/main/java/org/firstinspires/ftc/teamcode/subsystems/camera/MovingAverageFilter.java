package org.firstinspires.ftc.teamcode.subsystems.camera;


import java.util.LinkedList;
import java.util.Queue;

public class MovingAverageFilter {

    // A queue to store a history of angle readings
    private Queue<Double> angleHistory = new LinkedList<>();

    // The number of recent angle samples to include in the moving average
    private static final int HISTORY_SIZE = 150;

    // Applies a moving average filter to smooth out noisy angle readings
    public double filterAngle(double newAngle) {
        // Add the new angle to the end of the history queue
        angleHistory.add(newAngle);

        // If the history exceeds the desired size, remove the oldest entry
        if (angleHistory.size() > HISTORY_SIZE) {
            angleHistory.poll(); // Removes the oldest angle
        }

        // Compute the sum of all angles in the current history
        double sum = 0;
        for (double angle : angleHistory) {
            sum += angle;
        }

        // Return the average of the angles to produce a smoothed output
        return sum / angleHistory.size();
    }
}
