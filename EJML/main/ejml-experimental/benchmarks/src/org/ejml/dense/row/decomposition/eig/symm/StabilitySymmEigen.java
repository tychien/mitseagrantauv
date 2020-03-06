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

package org.ejml.dense.row.decomposition.eig.symm;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.ejml.dense.row.decomposition.eig.SymmetricQRAlgorithmDecomposition_DDRM;
import org.ejml.dense.row.factory.DecompositionFactory_DDRM;
import org.ejml.interfaces.decomposition.EigenDecomposition_F64;
import org.ejml.interfaces.decomposition.TridiagonalSimilarDecomposition_F64;

import java.util.Random;


/**
 * Compare the speed of various algorithms at inverting square matrices
 *
 * @author Peter Abeles
 */
public class StabilitySymmEigen {


    public static double evaluate(EigenDecomposition_F64<DMatrixRMaj> alg , DMatrixRMaj orig ) {

        if( !alg.decompose(orig)) {
            return Double.NaN;
        }

        return DecompositionFactory_DDRM.quality(orig,alg);
    }

    private static void runAlgorithms( DMatrixRMaj mat  )
    {
        TridiagonalSimilarDecomposition_F64<DMatrixRMaj> decomp = DecompositionFactory_DDRM.tridiagonal(0);
        System.out.println("qr ult           = "+ evaluate(new SymmetricQRAlgorithmDecomposition_DDRM(decomp,true),mat));
    }

    public static void main( String args [] ) {
        Random rand = new Random(239454923);

        int size = 10;
        double scales[] = new double[]{1,0.1,1e-20,1e-100,1e-200,1e-300,1e-304,1e-308,1e-310,1e-312,1e-319,1e-320,1e-321,Double.MIN_VALUE};

        System.out.println("Square matrix");
        DMatrixRMaj orig = RandomMatrices_DDRM.symmetric(size,-1,1,rand);
        DMatrixRMaj mat = orig.copy();
        // results vary significantly depending if it starts from a small or large matrix
        for( int i = 0; i < scales.length; i++ ) {
            System.out.printf("Decomposition size %3d for %e scale\n",size,scales[i]);
            CommonOps_DDRM.scale(scales[i],orig,mat);
            runAlgorithms(mat);
        }

        System.out.println("  Done.");
    }
}