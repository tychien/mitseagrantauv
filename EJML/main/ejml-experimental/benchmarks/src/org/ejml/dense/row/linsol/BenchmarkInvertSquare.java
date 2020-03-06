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

package org.ejml.dense.row.linsol;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.misc.UnrolledInverseFromMinor_DDRM;
import org.ejml.interfaces.linsol.LinearSolverDense;

import java.util.Random;


/**
 * Compare the speed of various algorithms at inverting square matrices
 *
 * @author Peter Abeles
 */
// TODO generate random stuff per trial
public class BenchmarkInvertSquare {


    public static long invertBenchmark(LinearSolverDense solver , DMatrixRMaj orig , int numTrials ) {
        DMatrixRMaj A = new DMatrixRMaj(orig.numRows,orig.numCols);

        long prev = System.currentTimeMillis();

        solver.setA(orig);

        for( long i = 0; i < numTrials; i++ ) {
            solver.invert(A);
        }

        return System.currentTimeMillis() - prev;
    }

    public static long invertUnrolledBenchmark(DMatrixRMaj orig , int numTrials ) {
        DMatrixRMaj A = new DMatrixRMaj(orig.numRows,orig.numCols);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            UnrolledInverseFromMinor_DDRM.inv(orig,A);
        }

        return System.currentTimeMillis() - prev;
    }

    public static long invertOpsBenchmark(DMatrixRMaj orig , int numTrials ) {
        DMatrixRMaj A = new DMatrixRMaj(orig.numRows,orig.numCols);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            CommonOps_DDRM.invert(orig,A);
        }

        return System.currentTimeMillis() - prev;
    }

    private static void runAlgorithms(DMatrixRMaj mat , int numTrials )
    {
//        System.out.println("invert GJ No Pivot     = "+ invertBenchmark(
//                new GaussJordanNoPivot(),mat,numTrials));
//        System.out.println("invert GJ              = "+ invertBenchmark(
//                new GaussJordan(mat.numRows),mat,numTrials));
//        System.out.println("invert LU              = "+ invertBenchmark(
//                new LinearSolverLu(new LUDecompositionAlt_DDRM()),mat,numTrials));
//        System.out.println("invert LU  NR          = "+ invertBenchmark(
//                new LinearSolverLu(new LUDecompositionNR()),mat,numTrials));
//        System.out.println("invert Ops             = "+
//                invertOpsBenchmark(mat,numTrials));
        System.out.println("unrolled               = "+
                invertUnrolledBenchmark(mat,numTrials));
    }

    public static void main( String args [] ) {
        Random rand = new Random(23423);

        int size[] = new int[]{5,4,6,10,100,1000,2000,5000,10000};
        int trials[] = new int[]{(int)8e6,(int)5e6,(int)2e6,(int)1e6,1000,3,1,1,1};

        for( int i = 0; i < 1; i++ ) {
            int w = size[i];

            System.out.printf("Inverting size %3d for %12d trials\n",w,trials[i]);
            DMatrixRMaj mat = RandomMatrices_DDRM.rectangle(w,w,rand);

            runAlgorithms(mat,trials[i]);
        }

    }
}
