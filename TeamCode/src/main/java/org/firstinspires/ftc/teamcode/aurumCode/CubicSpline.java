package org.firstinspires.ftc.teamcode.aurumCode;

import java.util.Arrays;
public class CubicSpline {
    private final double[] x; //x coordinates (knots)
    private final double[] a; // x constant terms (y values)
    private final double[] b; // linear coefficient
    private final double[] c; //quadratic coefficient
    private final double[] d; // cubic coefficient

    /**
     * Constructor
     * @param x The x-coordinates of the data points (must be sorted in ascending order)
     * @param y The y coordinates
     */
    public CubicSpline(double[] x, double[] y){
        if(x == null || y == null || x.length != y.length || x.length < 2){
            throw new IllegalArgumentException("There must be at least two points and arrays must be same length");
        }

        int n = x.length - 1;
        this.x = x;
        this.a = y;
        this.b = new double[n];
        this.c = new double[n + 1]; // we need c[n] for the calculation, though the spline only goes to n - 1
        this.d = new double[n];

        //1. calculate step sizes (h)
        double[] h = new double[n];
        for(int i = 0; i < n; i++){
            h[i] = x[i + 1] - x[i];
            if(h[i] <= 0){
                throw new IllegalArgumentException("X must be sorted in ascending order");
            }
        }

        //2. create tridiagonal matrix (alpha)
        //solves for the second derivative related to coefficient c
        double[] alpha = new double[n];
        for(int i = 1; i < n; i++){
            alpha[i] = (3.0 / h[i]) * (a[i + 1] - a[i]) - (3.0 / h[i - 1]) * (a[i] - a[i - 1]);
        }

        //3. solve tridiagonal system using crout factorization
        double[] l = new double[n + 1];
        double[] mu = new double[n + 1];
        double[] z = new double[n + 1];

        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        for(int i = 1; i < n; i++){
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n] = 1.0;
        z[n] = 0.0;
        this.c[n] = 0.0; // natural boundary condition (second derivative = 0 at end)

        //4. back-substitution to find coefficients b, c and d
        for(int j = n - 1; j >= 0; j--){
            this.c[j] = z[j] - mu[j] * this.c[j + 1];
            this.b[j] = (this.a[j + 1] - this.a[j]) / h[j] - (h[j] * (this.c[j + 1] + 2.0 * this.c[j])) / 3.0;
            this.d[j] = (this.c[j + 1] - this.c[j]) / (3.0 * h[j]);
        }
    }

    /**
     *
     * @param val The x value to interpolate
     * @return The interpolated y value
     */
    public double interpolate(double val){
        int n = x.length - 1;

        //handle out of bounds
        //clamp to the nearest endpoint for safety
        if(val <= x[0]) return a[0];
        if(val>= x[n]) return a[n];

        //binary search to find correct interval i
        int i = Arrays.binarySearch(x, val);

        //if exact match found, return the known y
        if(i >= 0) return a[i];

        //binary search to index
        //binary search return (-(insertion point) - 1)
        i = -(i + 1) - 1;

        //ensure i is within bounds
        if(i < 0) i = 0;
        if(i >= n) i = n - 1;

        double dx = val - x[i];

        return a[i] + dx * (b[i] + dx * (c[i] + dx * d[i]));
    }
}
