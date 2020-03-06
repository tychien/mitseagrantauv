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

package org.ejml.dense.row.decomposition.chol;

import org.ejml.EjmlParameters;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;

import java.util.Random;


/**
 * Compare the speed of various algorithms at inverting square matrices
 *
 * @author Peter Abeles
 */
public class BenchmarkCholeskyDecomposition {


    public static long choleskyL(DMatrixRMaj orig , int numTrials ) {

        CholeskyDecompositionInner_DDRM alg = new CholeskyDecompositionInner_DDRM(true);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig) ) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }

    public static long choleskyU(DMatrixRMaj orig , int numTrials ) {

        CholeskyDecompositionInner_DDRM alg = new CholeskyDecompositionInner_DDRM(false);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig) ) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }

    public static long choleskyL_block(DMatrixRMaj orig , int numTrials ) {

        CholeskyDecompositionBlock_DDRM alg = new CholeskyDecompositionBlock_DDRM(
                EjmlParameters.BLOCK_WIDTH_CHOL);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig) ) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }


    public static long choleskyBlockU(DMatrixRMaj orig , int numTrials ) {

        CholeskyDecomposition_DDRB_to_DDRM alg = new CholeskyDecomposition_DDRB_to_DDRM(false);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig) ) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }

    public static long choleskyBlockL(DMatrixRMaj orig , int numTrials ) {

        CholeskyDecomposition_DDRB_to_DDRM alg = new CholeskyDecomposition_DDRB_to_DDRM(true);

        long prev = System.currentTimeMillis();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig)) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }

    public static long choleskyLDL(DMatrixRMaj orig , int numTrials ) {

        long prev = System.currentTimeMillis();

        CholeskyDecompositionLDL_DDRM alg = new CholeskyDecompositionLDL_DDRM();

        for( long i = 0; i < numTrials; i++ ) {
            if( !DecompositionFactory_DDRM.decomposeSafe(alg,orig) ) {
                throw new RuntimeException("Bad matrix");
            }
        }

        return System.currentTimeMillis() - prev;
    }

    private static void runAlgorithms(DMatrixRMaj mat , int numTrials )
    {
        System.out.println("Lower            = "+ choleskyL(mat,numTrials));
//        System.out.println("Upper            = "+ choleskyU(mat,numTrials));
        System.out.println("Lower Block      = "+ choleskyL_block(mat,numTrials));
//        System.out.println("LDL              = "+ choleskyLDL(mat,numTrials));
//        System.out.println("Real Block U     = "+ choleskyBlockU(mat,numTrials));
        System.out.println("Real Block L     = "+ choleskyBlockL(mat,numTrials));
    }

    public static void main( String args [] ) {
        Random rand = new Random(23423);

        int size[] = new int[]{2,4,10,100,500,1000,2000,4000,10000};
        int trials[] = new int[]{(int)2e7,(int)5e6,(int)1e6,1000,40,3,1,1,1};

        // results vary significantly depending if it starts from a small or large matrix
        for( int i = 4; i < size.length; i++ ) {
            int w = size[i];

            System.out.printf("Decomposition size %3d for %12d trials\n",w,trials[i]);

            System.out.print("* Creating matrix ");
            DMatrixRMaj symMat = RandomMatrices_DDRM.symmetricPosDef(w,rand);
            System.out.println("  Done.");
            runAlgorithms(symMat,trials[i]);
        }
    }
}