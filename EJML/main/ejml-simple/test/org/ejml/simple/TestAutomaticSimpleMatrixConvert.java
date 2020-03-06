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

package org.ejml.simple;

import org.ejml.data.DMatrixRMaj;
import org.ejml.data.FMatrixRMaj;
import org.ejml.data.MatrixType;
import org.junit.Test;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * @author Peter Abeles
 */
public class TestAutomaticSimpleMatrixConvert {
    /**
     * Very basic test. The inner parts are tested elsewhere.
     */
    @Test
    public void sanityCheck() {
        AutomaticSimpleMatrixConvert alg = new AutomaticSimpleMatrixConvert();

        SimpleMatrix a = SimpleMatrix.wrap(new DMatrixRMaj(1,1));
        SimpleMatrix b = SimpleMatrix.wrap(new FMatrixRMaj(1,1));

        alg.specify(a,b);

        assertTrue(a==alg.convert(a));
        assertFalse(b==alg.convert(b));
        assertTrue(alg.convert(b).getType()== MatrixType.DDRM);
    }
}