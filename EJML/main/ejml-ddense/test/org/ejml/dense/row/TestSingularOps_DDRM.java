/*
 * Copyright (c) 2009-2018, Peter Abeles. All Rights Reserved.
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

package org.ejml.dense.row;

import org.ejml.UtilEjml;
import org.ejml.data.DGrowArray;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.equation.Equation;
import org.ejml.interfaces.decomposition.SingularValueDecomposition_F64;
import org.ejml.simple.SimpleMatrix;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;


/**
 * @author Peter Abeles
 */
public class TestSingularOps_DDRM {

    Random rand = new Random(234234);

    @Test
    public void singularValues() {
        for (int rows = 1; rows < 8; rows++) {
            for (int cols = 1; cols < 8; cols++) {
                DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rows,cols,rand);
                DMatrixRMaj A_orig = A.copy();

                double found[] = SingularOps_DDRM.singularValues(A);

                assertTrue( MatrixFeatures_DDRM.isIdentical(A_orig,A,0));

                int N = Math.min(rows,cols);
                assertEquals(N,found.length);
                for (int i = 1; i < N; i++) {
                    assertTrue(found[i-1]>found[i]);
                }
            }
        }
    }

    @Test
    public void rank_tol() {
        DMatrixRMaj A = CommonOps_DDRM.diag(1,1,2, 0.001 );
        assertEquals(4,SingularOps_DDRM.rank(A,UtilEjml.EPS ));
        assertEquals(3,SingularOps_DDRM.rank(A, 0.01 ));
    }

    @Test
    public void rank() {
        DMatrixRMaj A = CommonOps_DDRM.diag(1,1,2, 0.001 );
        assertEquals(4,SingularOps_DDRM.rank(A));
        A = CommonOps_DDRM.diag(1,1,2, (double)1e-24 );
        assertEquals(3,SingularOps_DDRM.rank(A));
    }

    @Test
    public void svd() {
        for (int rows = 1; rows < 8; rows++) {
            for (int cols = 1; cols < 8; cols++) {
                DMatrixRMaj A = RandomMatrices_DDRM.rectangle(rows,cols,rand);
                DMatrixRMaj A_orig = A.copy();

                DMatrixRMaj U = new DMatrixRMaj(1,1);
                DGrowArray sv = new DGrowArray();
                DMatrixRMaj Vt = new DMatrixRMaj(1,1);

                SingularOps_DDRM.svd(A,U,sv,Vt);

                assertTrue( MatrixFeatures_DDRM.isIdentical(A_orig,A,0));

                int N = Math.min(rows,cols);
                assertEquals(N,sv.length);
                for (int i = 1; i < N; i++) {
                    assertTrue(sv.data[i-1]>sv.data[i]);
                }

                DMatrixRMaj W = CommonOps_DDRM.diag(sv.data);

                DMatrixRMaj found =new Equation(U,"U",Vt,"Vt",W,"W").
                                process("A=U*W*Vt").lookupDDRM("A");

                assertTrue( MatrixFeatures_DDRM.isIdentical(A,found,UtilEjml.TEST_F64));
            }
        }
    }

    @Test
    public void descendingOrder() {
        // test different shapes of input matrices
        testDescendingOrder(3, 4, false,false);
        testDescendingOrder(4, 3, false,false);
        testDescendingOrder(3, 4, true,false);
        testDescendingOrder(4, 3, true,false);

        testDescendingInputTransposed(4,5,true,true,false);
    }


    @Test
    public void descendingOrder_array() {
        // test different shapes of input matrices
        testDescendingOrder(3, 4, false, true);
        testDescendingOrder(4, 3, false, true);
        testDescendingOrder(3, 4, true, true);
        testDescendingOrder(4, 3, true, true);

        testDescendingInputTransposed(4,5,true,true,true);
    }

    /**
     * Creates a random SVD that is highly unlikely to be in the correct order.  Adjust its order
     * and see if it produces the same matrix.
     */
    private void testDescendingOrder(int numRows, int numCols, boolean compact, boolean testArray ) {
        SimpleMatrix U,W,V;

        int minLength = Math.min(numRows,numCols);
        double singularValues[] = new double[minLength];

        if( compact ) {
            U = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numRows,minLength,rand));
            W = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(minLength,minLength,0,1,rand));
            V = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numCols,minLength,rand));
        } else {
            U = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numRows,numRows,rand));
            W = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(numRows,numCols,0,1,rand));
            V = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numCols,numCols,rand));
        }

        // Compute A
        SimpleMatrix A=U.mult(W).mult(V.transpose());

        // extract array of singular values
        for( int i = 0; i < singularValues.length; i++ )
            singularValues[i] = (double)W.get(i,i);
        
        // put into descending order
        if( testArray ) {
            SingularOps_DDRM.descendingOrder(U.getDDRM(),false,singularValues,minLength,V.getDDRM(),false);
            // put back into W
            for( int i = 0; i < singularValues.length; i++ )
                W.set(i,i,singularValues[i]);
        } else {
            SingularOps_DDRM.descendingOrder(U.getDDRM(),false,W.getDDRM(),V.getDDRM(),false);
        }

        // see if it changed the results
        SimpleMatrix A_found = U.mult(W).mult(V.transpose());

        assertTrue(A.isIdentical(A_found,UtilEjml.TEST_F64));

        // make sure singular values are descending
        if( testArray ) {
            for( int i = 1; i < minLength; i++ ) {
                assertTrue(singularValues[i-1] >= singularValues[i]);
            }
        } else {
            for( int i = 1; i < minLength; i++ ) {
                assertTrue(W.get(i-1,i-1) >= W.get(i,i));
            }
        }
    }

    /**
     * Use the transpose flags and see what happens
     */
    private void testDescendingInputTransposed(int numRows, int numCols,
                                               boolean tranU , boolean tranV , boolean testArray ) {
        SimpleMatrix U,S,V;

        int minLength = Math.min(numRows,numCols);
        double singularValues[] = new double[minLength];

        U = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numRows,minLength,rand));
        S = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(minLength,minLength,0,1,rand));
        V = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numCols,minLength,rand));

        // Compute A
        SimpleMatrix A=U.mult(S).mult(V.transpose());

        // extract array of singular values
        for( int i = 0; i < singularValues.length; i++ )
            singularValues[i] = (double)S.get(i,i);

        // put into ascending order
        if( tranU ) U = U.transpose();
        if( tranV ) V = V.transpose();

        // put into descending order
        if( testArray ) {
            SingularOps_DDRM.descendingOrder(U.getDDRM(),tranU,singularValues,minLength,V.getDDRM(),tranV);
            // put back into S
            for( int i = 0; i < singularValues.length; i++ )
                S.set(i,i,singularValues[i]);
        } else {
            SingularOps_DDRM.descendingOrder(U.getDDRM(),tranU,S.getDDRM(),V.getDDRM(),tranV);
        }

        // see if it changed the results
        if( tranU ) U = U.transpose();
        if( tranV ) V = V.transpose();
        SimpleMatrix A_found = U.mult(S).mult(V.transpose());

        assertTrue(A.isIdentical(A_found,UtilEjml.TEST_F64));

        // make sure singular values are descending
        if( testArray ) {
            for( int i = 1; i < minLength; i++ ) {
                assertTrue(singularValues[i-1] >= singularValues[i]);
            }
        } else {
            for( int i = 1; i < minLength; i++ ) {
                assertTrue(S.get(i-1,i-1) >= S.get(i,i));
            }
        }
    }

    /**
     * See if it blows up with uncountable numbers
     */
    @Test
    public void descendingOrder_NaN() {
        int numRows = 5;
        int numCols = 7;
        int minLength = Math.min(numRows,numCols);

        SimpleMatrix U,S,V;

        U = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numRows,minLength,rand));
        S = SimpleMatrix.wrap(RandomMatrices_DDRM.diagonal(minLength,minLength,0,1,rand));
        V = SimpleMatrix.wrap(RandomMatrices_DDRM.orthogonal(numCols,minLength,rand));

        // put in a NaN
        S.set(2,2,Double.NaN);

        SingularOps_DDRM.descendingOrder(U.getDDRM(),false,S.getDDRM(),V.getDDRM(),false);

        assertTrue( Double.isNaN((double)S.get(minLength-1,minLength-1)));

        // put in an Inf
        S.set(2,2,Double.POSITIVE_INFINITY);

        SingularOps_DDRM.descendingOrder(U.getDDRM(),false,S.getDDRM(),V.getDDRM(),false);

        assertTrue( Double.isInfinite((double)S.get(0,0)));
    }


    /**
     * Gives it correct input matrices and makes sure no exceptions are thrown.  All permutations
     * are tested.
     */
    @Test
    public void checkSvdMatrixSize_positive() {
        checkSvdMatrixSize_positive(4,5);
        checkSvdMatrixSize_positive(5,4);
    }

    /**
     * Checks a few of the many possible bad inputs
     */
    @Test
    public void checkSvdMatrixSize_negative() {
        checkSvdMatrixSize_negative(4,5);
        checkSvdMatrixSize_negative(5,4);
    }

    private void checkSvdMatrixSize_positive( int numRows , int numCols )
    {
        int s = Math.min(numRows,numCols);

        // create a none compact SVD
        DMatrixRMaj U = new DMatrixRMaj(numRows,numRows);
        DMatrixRMaj W = new DMatrixRMaj(numRows,numCols);
        DMatrixRMaj V = new DMatrixRMaj(numCols,numCols);

        SingularOps_DDRM.checkSvdMatrixSize(U,false,W,V,false);
        CommonOps_DDRM.transpose(U);
        CommonOps_DDRM.transpose(V);
        SingularOps_DDRM.checkSvdMatrixSize(U,true,W,V,true);

        // compact SVD
        U = new DMatrixRMaj(numRows,s);
        W = new DMatrixRMaj(s,s);
        V = new DMatrixRMaj(numCols,s);

        SingularOps_DDRM.checkSvdMatrixSize(U,false,W,V,false);
        CommonOps_DDRM.transpose(U);
        CommonOps_DDRM.transpose(V);
        SingularOps_DDRM.checkSvdMatrixSize(U,true,W,V,true);

        // see what happens if you throw in some null matrices
        SingularOps_DDRM.checkSvdMatrixSize(null,false,W,null,false);
        SingularOps_DDRM.checkSvdMatrixSize(null,true,W,V,true);
        SingularOps_DDRM.checkSvdMatrixSize(U,true,W,null,true);
    }

    private void checkSvdMatrixSize_negative( int numRows , int numCols )
    {
        int s = Math.min(numRows,numCols);

        // create a none compact SVD
        DMatrixRMaj U = new DMatrixRMaj(numRows,s);
        DMatrixRMaj W = new DMatrixRMaj(numRows,numCols);
        DMatrixRMaj V = new DMatrixRMaj(numCols,s);

        try {
            SingularOps_DDRM.checkSvdMatrixSize(U,false,W,V,false);
            fail("An exception should have been thrown");
        } catch( RuntimeException e) {}


        // compact SVD
        U = new DMatrixRMaj(numRows,s);
        W = new DMatrixRMaj(s,s);
        V = new DMatrixRMaj(numCols,s);

        try {
            SingularOps_DDRM.checkSvdMatrixSize(U,true,W,V,true);
            fail("An exception should have been thrown");
        } catch( RuntimeException e) {}
        CommonOps_DDRM.transpose(U);
        CommonOps_DDRM.transpose(V);
        try {
            SingularOps_DDRM.checkSvdMatrixSize(U,false,W,V,false);
            fail("An exception should have been thrown");
        } catch( RuntimeException e) {}
    }

    @Test
    public void nullVector() {
        for( int numRows = 2; numRows < 10; numRows++ ) {
            for( int numCols = 2; numCols < 10; numCols++ ) {
                // construct a matrix with a null space by decomposition a random matrix
                // and setting one of its singular values to zero
                SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(numRows,numCols,rand));

                SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(A.numRows(), A.numCols(),true,true,false);
                assertTrue(svd.decompose(A.getDDRM()));

                SimpleMatrix U = SimpleMatrix.wrap(svd.getU(null,false));
                SimpleMatrix S = SimpleMatrix.wrap(svd.getW(null));
                SimpleMatrix Vt = SimpleMatrix.wrap(svd.getV(null,true));

                // pick an element inconveniently in the middle to be the null space
                S.set(1,1,0);
                svd.getSingularValues()[1] = 0;

                A=U.mult(S).mult(Vt);

                // Find the right null space
                SimpleMatrix v = SimpleMatrix.wrap(SingularOps_DDRM.nullVector(svd, true , null));

                // see if the returned vector really is the null space
                SimpleMatrix ns = A.mult(v);

                for( int i = 0; i < ns.numRows(); i++ ) {
                    assertEquals(0,ns.get(i),UtilEjml.TEST_F64);
                }

                // Find the left null space
                v = SimpleMatrix.wrap(SingularOps_DDRM.nullVector(svd, false , null));

                // see if the returned vector really is the null space
                ns = v.transpose().mult(A);

                for( int i = 0; i < ns.numRows(); i++ ) {
                    assertEquals(0,ns.get(i),UtilEjml.TEST_F64);
                }
            }
        }
    }

    @Test
    public void nullSpace() {
        for( int numRows = 2; numRows < 5; numRows++ ) {
            for( int numCols = 2; numCols < 5; numCols++ ) {

                // construct a matrix with a null space by decomposition a random matrix
                // and setting one of its singular values to zero
                SimpleMatrix A = SimpleMatrix.wrap(RandomMatrices_DDRM.rectangle(numRows,numCols,rand));

                SingularValueDecomposition_F64<DMatrixRMaj> svd = DecompositionFactory_DDRM.svd(A.numRows(), A.numCols(),true,true,false);
                assertTrue(svd.decompose(A.getDDRM()));

                SimpleMatrix U = SimpleMatrix.wrap(svd.getU(null,false));
                SimpleMatrix S = SimpleMatrix.wrap(svd.getW(null));
                SimpleMatrix Vt = SimpleMatrix.wrap(svd.getV(null,true));

                // pick an element inconveniently in the middle to be the null space
                S.set(1,1,0);
                svd.getSingularValues()[1] = 0;

                A=U.mult(S).mult(Vt);

                // now find the null space
                SimpleMatrix ns = SimpleMatrix.wrap(SingularOps_DDRM.nullSpace(svd,null,UtilEjml.EPS));

                // make sure the null space is not all zero
                assertTrue( Math.abs(CommonOps_DDRM.elementMaxAbs(ns.getDDRM())) > 0 );

                // check the null space's size
                assertEquals(ns.numRows(),A.numCols());
                assertEquals(ns.numCols(),1+Math.max(numCols-numRows,0));

                // see if the results are null
                SimpleMatrix found = A.mult(ns);
                assertTrue( Math.abs(CommonOps_DDRM.elementMaxAbs(found.getDDRM())) <= 10*UtilEjml.EPS );
            }
        }
    }

    /**
     * Decompose a singular matrix and see if it produces the expected result
     */
    @Test
    public void rank_and_nullity(){
        DMatrixRMaj A = new DMatrixRMaj(3,3, true,
                -0.988228951897092, -1.086594333683141, -1.433160736952583,
                -3.190200029661606, 0.190459703263404, -6.475629910954768,
                1.400596416735888, 7.158603907761226, -0.778109120408813);
        rank_and_nullity(A,2,1);

        //wide matrix
        A = new DMatrixRMaj(1,3,true,1,0,0);
        rank_and_nullity(A,1,2);

        // tall matrix
        A = new DMatrixRMaj(3,1,true,1,0,0);
        rank_and_nullity(A,1,0);
    }

    public void rank_and_nullity(DMatrixRMaj A , int rank , int nullity ) {
        SingularValueDecomposition_F64<DMatrixRMaj> alg = DecompositionFactory_DDRM.svd(A.numRows,A.numCols,true,true,false);
        assertTrue(alg.decompose(A));

        assertEquals(rank, SingularOps_DDRM.rank(alg, UtilEjml.EPS));
        assertEquals(nullity, SingularOps_DDRM.nullity(alg, UtilEjml.EPS));
    }

    /**
     * Decompose a singular matrix and see if it produces the expected result
     */
    @Test
    public void rank_and_nullity_noArgument(){
        DMatrixRMaj A = new DMatrixRMaj(3,3, true,
                -0.988228951897092, -1.086594333683141, -1.433160736952583,
                -3.190200029661606, 0.190459703263404, -6.475629910954768,
                1.400596416735888, 7.158603907761226, -0.778109120408813);
        rank_and_nullity_noArgument(A, 2, 1);

        //wide matrix
        A = new DMatrixRMaj(1,3,true,1,0,0);
        rank_and_nullity_noArgument(A,1,2);

        // tall matrix
        A = new DMatrixRMaj(3,1,true,1,0,0);
        rank_and_nullity_noArgument(A,1,0);
    }

    public void rank_and_nullity_noArgument(DMatrixRMaj A , int rank , int nullity ) {
        SingularValueDecomposition_F64<DMatrixRMaj> alg = DecompositionFactory_DDRM.svd(A.numRows,A.numCols,true,true,false);
        assertTrue(alg.decompose(A));

        assertEquals(rank, SingularOps_DDRM.rank(alg));
        assertEquals(nullity, SingularOps_DDRM.nullity(alg));
    }
}
