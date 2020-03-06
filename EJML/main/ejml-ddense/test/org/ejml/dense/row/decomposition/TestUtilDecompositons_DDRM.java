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

package org.ejml.dense.row.decomposition;

import org.ejml.UtilEjml;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.MatrixFeatures_DDRM;
import org.ejml.dense.row.RandomMatrices_DDRM;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestUtilDecompositons_DDRM {

    Random rand = new Random(234);

    @Test
    public void checkIdentity_null() {
        DMatrixRMaj A = UtilDecompositons_DDRM.checkIdentity(null,4,3);
        assertTrue(MatrixFeatures_DDRM.isIdentity(A, UtilEjml.TEST_F64));
    }

    @Test
    public void checkIdentity_random() {
        DMatrixRMaj orig = RandomMatrices_DDRM.rectangle(4,3,rand);
        DMatrixRMaj A = UtilDecompositons_DDRM.checkIdentity(orig,4,3);
        assertTrue(MatrixFeatures_DDRM.isIdentity(A, UtilEjml.TEST_F64));
        assertTrue(A==orig);
    }

    @Test
    public void checkZeros_null() {
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZeros(null,4,3);
        assertTrue(MatrixFeatures_DDRM.isZeros(A, UtilEjml.TEST_F64));
    }

    @Test
    public void checkZeros_random() {
        DMatrixRMaj orig = RandomMatrices_DDRM.rectangle(4,3,rand);
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZeros(orig,4,3);
        assertTrue(MatrixFeatures_DDRM.isZeros(A, UtilEjml.TEST_F64));
        assertTrue(A==orig);
    }

    @Test
    public void checkZerosLT_null() {
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZerosLT(null,4,3);
        assertTrue(MatrixFeatures_DDRM.isUpperTriangle(A,0, UtilEjml.TEST_F64));
    }

    @Test
    public void checkZerosLT_random() {
        DMatrixRMaj orig = RandomMatrices_DDRM.rectangle(4,3,rand);
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZerosLT(orig,4,3);
        assertTrue(MatrixFeatures_DDRM.isUpperTriangle(A, 0, UtilEjml.TEST_F64));
        assertTrue(A==orig);
    }

    @Test
    public void checkZerosUT_null() {
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZerosUT(null,4,3);
        assertTrue(MatrixFeatures_DDRM.isLowerTriangle(A,0, UtilEjml.TEST_F64));
    }

    @Test
    public void checkZerosUT_random() {
        DMatrixRMaj orig = RandomMatrices_DDRM.rectangle(4,3,rand);
        DMatrixRMaj A = UtilDecompositons_DDRM.checkZerosUT(orig,4,3);
        assertTrue(MatrixFeatures_DDRM.isLowerTriangle(A, 0, UtilEjml.TEST_F64));
        assertTrue(A==orig);
    }
}