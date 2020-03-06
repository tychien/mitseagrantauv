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

package org.ejml.dense.row.mult;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;

import java.util.Random;


/**
 *
 * @author Peter Abeles
 */
public class BenchmarkMatrixMatrixMultQuad {

    static Random rand = new Random(234234);

    static int TRIALS_MULT = 10000000;

    public static long mult1(DMatrixRMaj A , DMatrixRMaj B , DMatrixRMaj tmp,
                             DMatrixRMaj expected , int numTrials) {
        long prev = System.currentTimeMillis();

        for( int i = 0; i < numTrials; i++ ) {
            MatrixMatrixMult_DDRM.mult_small(A,B,tmp);
            MatrixMatrixMult_DDRM.multTransB(tmp, A, expected);
        }

        long curr = System.currentTimeMillis();
        return curr-prev;
    }

    public static long quad1(DMatrixRMaj A , DMatrixRMaj B ,
                             DMatrixRMaj expected , int numTrials) {
        long prev = System.currentTimeMillis();

        for( int i = 0; i < numTrials; i++ ) {
            MatrixMultQuad.multQuad1(A, B, expected);
        }

        long curr = System.currentTimeMillis();
        return curr-prev;
    }

    public static void performTests( int numRows , int numCols ,
                                     int numTrials ) {
        DMatrixRMaj A = RandomMatrices_DDRM.rectangle(numRows,numCols,rand);
        DMatrixRMaj B = RandomMatrices_DDRM.rectangle(numCols,numCols,rand);
        DMatrixRMaj out = RandomMatrices_DDRM.rectangle(numRows, numRows, rand);
        DMatrixRMaj tmp = new DMatrixRMaj(numRows,numCols);

        System.out.printf(numRows+"  "+numCols+"     Mult1: %7d  Quad1 %7d\n",
                mult1(A,B,tmp,out,numTrials),
                quad1(A,B,out,numTrials));
    }

    public static void main( String args[] ) {
        int size[] = new int[]{2,4,10,15,20,50,100,200,500,1000,2000,4000,10000};
        int count[] = new int[]{12000000,2000000,200000,60000,30000,1000,300,25,1,1,1,1,1};

        int N = size.length;

        for( int i = 0; i < N; i++ ) {
            System.out.println();

            performTests(size[i],size[i]*2,count[i]);
        }


    }
}