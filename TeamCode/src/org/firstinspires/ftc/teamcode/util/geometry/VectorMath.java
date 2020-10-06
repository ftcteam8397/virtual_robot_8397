package org.firstinspires.ftc.teamcode.util.geometry;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class VectorMath {
    public static final int X = 0;
    public static final int Y = 1;
    public static final int Z = 2;

    // Function to find
    // cross product of two vector array.
    // https://www.geeksforgeeks.org/program-dot-product-cross-product-two-vector/
    public static VectorF crossProduct(VectorF vectA, VectorF vectB) {
        float x = vectA.get(Y) * vectB.get(Z) - vectA.get(Z) * vectB.get(Y);
        float y = vectA.get(Z) * vectB.get(X) - vectA.get(X) * vectB.get(Z);
        float z = vectA.get(X) * vectB.get(Y) - vectA.get(Y) * vectB.get(X);
        return new VectorF(x, y, z);
    }

    // Function that return
    // dot product of two vector array.
    public static float dotProduct(float[] vect_A, float[] vect_B) {

        float product = 0;

        // Loop for calculate cot product
        for (int i = 0; i < vect_A.length; i++)
            product = product + vect_A[i] * vect_B[i];
        return product;
    }

    public static float determinant2D(float[] vectA, float[] vectB) {
        return vectA[X] * vectB[Y] - vectB[X] * vectA[Y];
    }
}
