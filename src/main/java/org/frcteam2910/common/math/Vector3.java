package org.frcteam2910.common.math;

import org.frcteam2910.common.math.MathUtils;

import java.text.DecimalFormat;
import java.util.Objects;

/**
 * A vector representing a point in 3d space
 */
public final class Vector3 {

    /**
     * A vector with a length of zero
     */
    public static final Vector3 ZERO = new Vector3(0, 0, 0);

    /**
     * The x coordinate of the vector
     */
    public final double x;

    /**
     * The y coordinate of the vector
     */
    public final double y;

    /**
     * The z coordinate of the vector
     */
    public final double z;

    /**
     * The length of the vector
     */
    public final double length;

    /**
     * Creates a new vector with the specified x, y and z components
     * @param x The x coordinate
     * @param y The y coordinate
     * @param z The z coordinate
     */
    public Vector3(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;

        length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    /**
     * Creates a new vector from a double array containing the x, y, and z components
     * @param vector The double array containing the x, y, and z components
     */
    public Vector3(double[] vector) {
        x = vector[0];
        y = vector[1];
        z = vector[2];

        length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
    }

    /**
     * Adds this vector and another vector together
     * @param vector The vector to add
     * @return A vector with the result of the addition
     */
    public Vector3 add(Vector3 vector) {
        return add(vector.x, vector.y, vector.z);
    }

    /**
     * Adds three scalar values to this vector
     * @param x The value to add to the x-coordinate
     * @param y The value to add to the y-coordinate
     * @param z The values to add to the z-coordinate
     * @return A vector with the result of the addition
     */
    public Vector3 add(double x, double y, double z) {
        return new Vector3(this.x + x, this.y + y, this.z + z);
    }

    /**
     * Subtracts a vector from this vector
     * @param vector
     * @return A vector with the result of the subtraction
     */
    public Vector3 subtract(Vector3 vector) {
        return subtract(vector.x, vector.y, vector.z);
    }

    /**
     * Subtracts three scalar values from this vector
     * @param x The value to subtract from the x-coordinate
     * @param y The value to subtract from the y-coordinate
     * @param z The value to subtract from the z-coordinate
     * @return A vector with the result of the subtraction
     */
    public Vector3 subtract(double x, double y, double z) {
        return new Vector3(this.x - x, this.y - y, this.z - z);
    }

    /**
     * Multiplies each component of the vector by a scalar values
     * @param scalar The scalar to multiply each component by
     * @return The vector scaled by the scalar
     */
    public Vector3 scale(double scalar) {
        return multiply(scalar, scalar, scalar);
    }

    /**
     * Performs a component-wise multiplication on this vector with another vector
     * @param scale The vector to multiply by
     * @return A vector with the result of the multiplication
     */
    public Vector3 mulitply(Vector3 scale) {
        return multiply(scale.x, scale.y, scale.z);
    }

    /**
     * Multiplies the components of this vector by the three scalar values
     * @param x A scalar to multiply the x-coordinate by
     * @param y A scalar to multiply the y-coordinate by
     * @param z A scalar to multiply the z-coordinate by
     * @return A vector with the result of the multiplication
     */
    public Vector3 multiply(double x, double y, double z) {
        return new Vector3(this.x * x, this.y * y, this.z * z);
    }

    /**
     * Neagtes the vector
     * @return A vector that when added to this vector would result in a zero vector
     */
    public Vector3 negate() {
        return new Vector3(-x, -y, -z);
    }

    /**
     * Calculates the normalized form of this vector
     *
     * @return A unit vector with the same angle as this vector
     */
    public Vector3 norm() {
        return new Vector3(x / length, y / length, z / length);
    }

    /**
     * Calculates the dot product of this vector and another vector
     * @param other The other vector to calculate the dot product with
     * @return The dot product of the two vectors
     */
    public double dot(Vector3 other) {
        return (this.x * other.x) + (this.y * other.y) + (this.z * other.z);
    }

    /**
     * Calculates the cross product of this vector and another vector
     * @param other The other vector to calculate the cross product with
     * @return The cross product of the two vectors
     */
    public Vector3 cross(Vector3 other) {
        double result_x = (this.y * other.z) - (this.z * other.y);
        double result_y = (this.z * other.x) - (this.x * other.z);
        double result_z = (this.x * other.y) - (this.y * other.x);
        return new Vector3(result_x, result_y, result_z);
    }

    /**
     * Rotates this vector by the specified rotation
     * @param rotation A Rotation3 containing the rotation matrix to rotate by
     * @return A vector rotated by the rotation matrix
     */
    public Vector3 rotate(Rotation3 rotation) {
        double[][] rotationMatrix = rotation.rotationMatrix;
        double r_x = x * rotationMatrix[0][0] + y * rotationMatrix[1][0] + z * rotationMatrix[2][0];
        double r_y = x * rotationMatrix[0][1] + y * rotationMatrix[1][1] + z * rotationMatrix[2][1];
        double r_z = x * rotationMatrix[0][2] + y * rotationMatrix[1][2] + z * rotationMatrix[2][2];
        return new Vector3(r_x, r_y, r_z);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public boolean equals(Object obj) {
        if (!(obj instanceof Vector3)) {
            return false;
        }

        return equals((Vector3) obj, MathUtils.EPSILON);
    }

    /**
     * Checks if the two vectors are equal with a default of 1e-9 error
     * @param other The other vector to compare with
     * @return True if the vectors are equals given the allowable error, false if not
     */
    public boolean equals(Vector3 other) {
        return equals(other, MathUtils.EPSILON);
    }

    /**
     * Checks if the two vectors are the same given the allowable error
     * @param other The other vector to compare with
     * @param allowableError
     * @return True if the vectors are equals given the allowable error, false if not
     */
    public boolean equals(Vector3 other, double allowableError) {
        return MathUtils.epsilonEquals(x, other.x, allowableError) &&
                MathUtils.epsilonEquals(y, other.y, allowableError) &&
                MathUtils.epsilonEquals(z, other.z, allowableError);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }

    /**
     * {@inheritDoc}
     */
    @Override
    public String toString() {
        DecimalFormat fmt = new DecimalFormat("#0.000");
        return '(' + fmt.format(x) + ", " + fmt.format(y) + ", " + fmt.format(z) + ')';
    }
}
