package org.firstinspires.ftc.teamcode.subsystems.camera;
public class Vec2 {

public double x;
public double y;

public double direction;

public double magnitude;


public Vec2(double a, double b) {
    // Assign components
    x = a;
    y = b;
    magnitude = Math.sqrt(x*x + y*y);

    direction = compassAtan(x, y);
}


double clamp(double value, double min, double max) {
    if (value < min) return min;
    return Math.min(value, max);
}


public double toCompassAngle(double polarDirection) {
    return (polarDirection + 360) % 360;
}

public double toPolarAngle(double compassDirection) {
    if (compassDirection >= 180) {
        return -(360 - compassDirection);
    } else {
        return compassDirection;
    }
}

/**
 * Custom atan calculation that returns angle in compass degrees (0-360).
 * Uses x and y components (not in the standard order of atan2).
 * Handles quadrant corrections manually.
 * @param xComponent x value
 * @param yComponent y value
 * @return angle in compass degrees from 0 to 360
 */
double compassAtan(double xComponent, double yComponent) {
    // Handle division by zero case (y=0)
    if (y == 0) { return 0; }
    // Calculate atan in degrees of x/y (note: swapped arguments compared to atan2)
    double value = Math.toDegrees(Math.atan(x/y));
    // Quadrant checks and adjustments for compass system
    if (x > 0 && y >= 0) { return value;}
    if (x > 0 && y < 0) { return 90 - value;}
    if (x < 0 && y < 0) { return 180 + value;}
    if (x < 0 && y >= 0) {return 360 + value;}
    return 0;
}

/**
 * Sets vector components from x and y components.
 * Updates magnitude and direction accordingly.
 * @param new_x new x component
 * @param new_y new y component
 */
public void fromComponent(double new_x, double new_y) {
    x = new_x;
    y = new_y;
    magnitude = Math.sqrt(x*x + y*y);
    direction = compassAtan(x, y);
}

/**
 * Sets vector components from magnitude and direction (polar coordinates).
 * Calculates x and y accordingly.
 * @param new_magnitude magnitude (length) of vector
 * @param new_degrees direction in degrees (compass degrees)
 */
void fromPolar(double new_magnitude, double new_degrees) {
    magnitude = new_magnitude;
    direction = new_degrees;
    // Calculate x and y from polar coords: x = sin(angle)*mag, y = cos(angle)*mag
    x = Math.sin(Math.toRadians(new_degrees)) * new_magnitude;
    y = Math.cos(Math.toRadians(new_degrees)) * new_magnitude;
}

/**
 * Adds another vector to this vector (component-wise addition).
 * Updates this vector's x and y, magnitude and direction are not updated here.
 * @param vector vector to add
 * @return this vector after addition
 */
public Vec2 add(Vec2 vector) {
    this.x += vector.x;
    this.y += vector.y;
    return this;
}

/**
 * Subtracts another vector from this vector (component-wise subtraction).
 * Updates this vector's x and y, magnitude and direction are not updated here.
 * @param vector vector to subtract
 * @return this vector after subtraction
 */
public Vec2 sub(Vec2 vector) {
    this.x -= vector.x;
    this.y -= vector.y;
    return this;
}

/**
 * Returns a new vector that is the normalized form (unit vector) of this vector.
 * Normalized vector has magnitude 1 and same direction.
 * @return normalized vector (new instance)
 */
Vec2 normalize() {
    return new Vec2(x / magnitude, y / magnitude);
}

/**
 * Divides this vector's components by a denominator (scales vector).
 * @param denominator value to divide x and y by
 * @return this vector after division
 */
public Vec2 divide(double denominator) {
    this.x /= denominator;
    this.y /= denominator;
    return this;
}
}
