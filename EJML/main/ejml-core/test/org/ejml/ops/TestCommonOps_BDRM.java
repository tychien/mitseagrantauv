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

package org.ejml.ops;

import org.ejml.data.BMatrixRMaj;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * @author Peter Abeles
 */
public class TestCommonOps_BDRM {
    @Test
    public void transposeSquare() {
        BMatrixRMaj A = new BMatrixRMaj(4,4);
        A.set(0,0,true);
        A.set(1,0,true);
        A.set(1,1,true);
        A.set(3,2,true);

        BMatrixRMaj B = A.copy();

        CommonOps_BDRM.transposeSquare(B);

        for( int y = 0; y < A.getNumRows(); y++ ){
            for( int x = 0; x < A.getNumCols(); x++ ) {
                assertEquals(A.get(y,x),B.get(x,y));
            }
        }
    }
}
