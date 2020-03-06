/*
 * Copyright (c) 2009-2017, Peter Abeles. All Rights Reserved.
 *
 * This file is part of Efficient Java Matrix Library (EJML).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.ejml.dense.block.linsol.qr;

import org.ejml.UtilEjml;
import org.ejml.data.DMatrixRBlock;
import org.ejml.dense.block.MatrixOps_DDRB;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.generic.GenericMatrixOps_F64;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;


/**
 * @author Peter Abeles
 */
public class TestQrHouseHolderSolver_DDRB {

    Random rand = new Random(23423);

    /**
     * Test positive examples against a variety of different inputs shapes.
     */
    @Test
    public void testPositiveSolve() {
        int r = 3;
        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();

        for( int i = 1; i <= r*3; i++ ) {
            for( int j = i; j <= r*3; j++ ) {
                for( int k = 1; k <= r*3; k++ ) {
//                    System.out.println("i = "+i+" j = "+j+" k = "+k);
                    DMatrixRBlock A = MatrixOps_DDRB.createRandom(j,i,-1,1,rand,r);
                    DMatrixRBlock X = MatrixOps_DDRB.createRandom(i,k,-1,1,rand,r);
                    DMatrixRBlock Y = new DMatrixRBlock(j,k,r);
                    DMatrixRBlock X_found = new DMatrixRBlock(i,k,r);

                    // compute the expected solution directly
                    MatrixOps_DDRB.mult(A,X,Y);

                    assertTrue(solver.setA(A.copy()));

                    solver.solve(Y,X_found);

                    assertTrue(MatrixOps_DDRB.isEquals(X,X_found, UtilEjml.TEST_F64));
                }
            }
        }
    }

    @Test
    public void testInvert() {
        int r = 3;
        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();

        for( int i = 1; i <= r*3; i++ ) {
            DMatrixRBlock A = MatrixOps_DDRB.createRandom(i,i,-1,1,rand,r);

            DMatrixRBlock A_orig = A.copy();
            DMatrixRBlock I = new DMatrixRBlock(i,i,r);

            assertTrue(solver.setA(A.copy()));

            solver.invert(A);

            // A times its inverse is an identity matrix
            MatrixOps_DDRB.mult(A,A_orig,I);

            assertTrue(GenericMatrixOps_F64.isIdentity(I,UtilEjml.TEST_F64));
        }
    }

    @Test
    public void testQuality() {
        DMatrixRBlock A = MatrixOps_DDRB.convert(CommonOps_DDRM.diag(4,3,2,1),3);
        DMatrixRBlock B = MatrixOps_DDRB.convert(CommonOps_DDRM.diag(4,3,2,0.1),3);

        // see if a matrix with smaller singular value has a worse quality
        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();
        assertTrue(solver.setA(A.copy()));
        double qualityA = (double)solver.quality();

        assertTrue(solver.setA(B.copy()));
        double qualityB = (double)solver.quality();

        assertTrue(qualityB<qualityA);
        assertEquals(qualityB*10.0,qualityA,UtilEjml.TEST_F64);
    }

    /**
     * Checks to see if quality is scale invariant.
     */
    @Test
    public void testQuality_scale() {
        DMatrixRBlock A = MatrixOps_DDRB.convert(CommonOps_DDRM.diag(4,3,2,1),3);
        DMatrixRBlock B = A.copy();
        CommonOps_DDRM.scale(2,B);

        // see if a matrix with smaller singular value has a worse quality
        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();
        assertTrue(solver.setA(A.copy()));
        double qualityA = (double)solver.quality();

        assertTrue(solver.setA(B.copy()));
        double qualityB = (double)solver.quality();

        assertEquals(qualityA,qualityB,UtilEjml.TEST_F64);
    }

    @Test
    public void modifiesA(){
        DMatrixRBlock A = MatrixOps_DDRB.createRandom(4,4,-1,1,rand,3);
        DMatrixRBlock A_orig = A.copy();

        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();

        assertTrue(solver.setA(A));

        boolean modified = !MatrixFeatures_DDRM.isEquals(A,A_orig);

        assertTrue(modified == solver.modifiesA());
    }

    @Test
    public void modifiesB(){
        DMatrixRBlock A = MatrixOps_DDRB.createRandom(4,4,-1,1,rand,3);

        QrHouseHolderSolver_DDRB solver = new QrHouseHolderSolver_DDRB();

        assertTrue(solver.setA(A));

        DMatrixRBlock B = MatrixOps_DDRB.createRandom(4,2,-1,1,rand,3);
        DMatrixRBlock B_orig = B.copy();
        DMatrixRBlock X = new DMatrixRBlock(A.numRows,B.numCols,3);

        solver.solve(B,X);

        boolean modified = !MatrixFeatures_DDRM.isEquals(B_orig,B);

        assertTrue(modified == solver.modifiesB());
    }

}
