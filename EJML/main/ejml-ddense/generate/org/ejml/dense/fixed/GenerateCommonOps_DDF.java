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

package org.ejml.dense.fixed;

import org.ejml.UtilEjml;
import org.ejml.dense.row.misc.GenerateDeterminantFromMinor;
import org.ejml.dense.row.misc.GenerateInverseFromMinor;

import java.io.FileNotFoundException;

/**
 * Automatic code generator for FixedOps
 *
 * @author Peter Abeles
 */
public class GenerateCommonOps_DDF extends GenerateFixed {

    public GenerateCommonOps_DDF() {
        super("CommonOps_DDF");
    }

    @Override
    public void generate() throws FileNotFoundException {
        for( int dimension = 2; dimension <= 6; dimension++ ){
            printPreable(dimension);

            add(dimension);
            vector_add(dimension);
            addEquals(dimension);
            vector_addEquals(dimension);
            subtract(dimension);
            vector_subtract(dimension);
            subtractEquals(dimension);
            vector_subtractEquals(dimension);
            transpose_one(dimension);
            transpose_two(dimension);
            for( int i = 0; i < 2; i ++ ) {
                boolean add = i == 1;
                mult(dimension,add);
                multScale(dimension,add);
                multTransA(dimension,add);
                multTransAScale(dimension,add);
                multTransAB(dimension,add);
                multTransABScale(dimension,add);
                multTransB(dimension,add);
                multTransBScale(dimension,add);
            }
            multAddOuter(dimension);
            mult_m_v_v(dimension);
            mult_v_m_v(dimension);
            dot(dimension);
            setIdentity(dimension);
            if( dimension <= UtilEjml.maxInverseSize ) {
                invert(dimension);
                det(dimension);
            }
            cholL(dimension);
            cholU(dimension);
            trace(dimension);
            diag(dimension);
            elementMax(dimension);
            elementMax_vector(dimension);
            elementMaxAbs(dimension);
            elementMaxAbs_vector(dimension);
            elementMin(dimension);
            elementMin_vector(dimension);
            elementMinAbs(dimension);
            elementMinAbs_vector(dimension);
            elementMult_two(dimension);
            elementMult_vector_two(dimension);
            elementMult_three(dimension);
            elementMult_vector_three(dimension);
            elementDiv_two(dimension);
            elementDiv_vector_two(dimension);
            elementDiv_three(dimension);
            elementDiv_vector_three(dimension);
            scale_two(dimension);
            scale_vector_two(dimension);
            scale_three(dimension);
            scale_vector_three(dimension);
            divide_two(dimension);
            divide_vector_two(dimension);
            divide_three(dimension);
            divide_vector_three(dimension);
            changeSign(dimension);
            changeSign_vector(dimension);
            fill(dimension);
            fill_vector(dimension);
            extract(dimension);

            out.println("}\n");
        }
    }

    public void printPreable( int dimen ) throws FileNotFoundException {

        setClassNames(dimen);

        out.print(
                "import org.ejml.UtilEjml;\n" +
                "import org.ejml.data."+nameVector+";\n" +
                "import org.ejml.data."+nameMatrix+";\n" +
                "\n" +
                "/**\n" +
                " * <p>Common matrix operations for fixed sized matrices which are "+dimen+" x "+dimen+" or "+dimen+" element vectors.</p>\n" +
                " * <p>DO NOT MODIFY.  Automatically generated code created by "+getClass().getSimpleName()+"</p>\n" +
                " *\n" +
                " * @author Peter Abeles\n" +
                " */\n" +
                "public class "+className+" {\n");
    }

    private void add(int dimen){
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c = a + b <br>\n" +
                "     * c<sub>ij</sub> = a<sub>ij</sub> + b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * <p>\n" +
                "     * Matrix C can be the same instance as Matrix A and/or B.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Matrix. Not modified.\n" +
                "     * @param b A Matrix. Not modified.\n" +
                "     * @param c A Matrix where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void add( " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                String n = y+""+x;
                out.print("        c.a"+n+" = a.a"+n+" + b.a"+n+";\n");
            }
        }
        out.print("    }\n\n");
    }

    private void vector_add( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c = a + b <br>\n" +
                "     * c<sub>i</sub> = a<sub>i</sub> + b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * <p>\n" +
                "     * Vector C can be the same instance as Vector A and/or B.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Vector. Not modified.\n" +
                "     * @param b A Vector. Not modified.\n" +
                "     * @param c A Vector where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void add( " + nameVector + " a , " + nameVector + " b , " + nameVector + " c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        c.a"+y+" = a.a"+y+" + b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void addEquals( int dimen ){
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * a = a + b <br>\n" +
                "     * a<sub>ij</sub> = a<sub>ij</sub> + b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Matrix. Modified.\n" +
                "     * @param b A Matrix. Not modified.\n" +
                "     */\n" +
                "    public static void addEquals( " + nameMatrix + " a , " + nameMatrix + " b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                String n = y+""+x;
                out.print("        a.a"+n+" += b.a"+n+";\n");
            }
        }
        out.print("    }\n\n");
    }

    private void vector_addEquals( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * a = a + b <br>\n" +
                "     * a<sub>i</sub> = a<sub>i</sub> + b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Vector. Modified.\n" +
                "     * @param b A Vector. Not modified.\n" +
                "     */\n" +
                "    public static void addEquals( " + nameVector + " a , " + nameVector + " b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" += b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void subtract(int dimen){
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c = a - b <br>\n" +
                "     * c<sub>ij</sub> = a<sub>ij</sub> - b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * <p>\n" +
                "     * Matrix C can be the same instance as Matrix A and/or B.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Matrix. Not modified.\n" +
                "     * @param b A Matrix. Not modified.\n" +
                "     * @param c A Matrix where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void subtract( " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                String n = y+""+x;
                out.print("        c.a"+n+" = a.a"+n+" - b.a"+n+";\n");
            }
        }
        out.print("    }\n\n");
    }

    private void vector_subtract( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c = a - b <br>\n" +
                "     * c<sub>i</sub> = a<sub>i</sub> - b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * <p>\n" +
                "     * Vector C can be the same instance as Vector A and/or B.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Vector. Not modified.\n" +
                "     * @param b A Vector. Not modified.\n" +
                "     * @param c A Vector where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void subtract( "+nameVector+" a , "+nameVector+" b , "+nameVector+" c ) {\n");
        for( int y = 1; y <= dimen; y++) {
            out.print("        c.a" + y + " = a.a" + y + " - b.a" + y + ";\n");
        }
        out.print("    }\n\n");
    }

    private void subtractEquals(int dimen) {
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * a = a - b <br>\n" +
                "     * a<sub>ij</sub> = a<sub>ij</sub> - b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Matrix. Modified.\n" +
                "     * @param b A Matrix. Not modified.\n" +
                "     */\n" +
                "    public static void subtractEquals( "+nameMatrix+" a , "+nameMatrix+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            for (int x = 1; x <= dimen; x++) {
                String n = y + "" + x;
                out.print("        a.a" + n + " -= b.a" + n + ";\n");
            }
        }
        out.print("    }\n\n");
    }

    private void vector_subtractEquals( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * a = a - b <br>\n" +
                "     * a<sub>i</sub> = a<sub>i</sub> - b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A Vector. Modified.\n" +
                "     * @param b A Vector. Not modified.\n" +
                "     */\n" +
                "    public static void subtractEquals( "+nameVector+" a , "+nameVector+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            String n = ""+y;
            out.print("        a.a"+n+" -= b.a"+n+";\n");
        }
        out.print("    }\n\n");
    }

    private void transpose_one( int dimen ){
        out.print("    /**\n" +
                "     * Performs an in-place transpose.  This algorithm is only efficient for square\n" +
                "     * matrices.\n" +
                "     *\n" +
                "     * @param m The matrix that is to be transposed. Modified.\n" +
                "     */\n" +
                "    public static void transpose( " + nameMatrix + " m ) {\n" +
                "        double tmp;\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = y + 1; x <= dimen; x++) {
                String f = +y + "" + x;
                String t = +x + "" + y;

                out.print("        tmp = m.a"+f+"; m.a"+f+" = m.a"+t+"; m.a"+t+" = tmp;\n");
            }
        }
        out.print("    }\n\n");
    }

    private void transpose_two( int dimen ){
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Transposes matrix 'a' and stores the results in 'b':<br>\n" +
                "     * <br>\n" +
                "     * b<sub>ij</sub> = a<sub>ji</sub><br>\n" +
                "     * where 'b' is the transpose of 'a'.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param input The original matrix.  Not modified.\n" +
                "     * @param output Where the transpose is stored. If null a new matrix is created. Modified.\n" +
                "     * @return The transposed matrix.\n" +
                "     */\n" +
                "    public static " + nameMatrix + " transpose( " + nameMatrix + " input , " + nameMatrix + " output ) {\n" +
                "        if( input == null )\n" +
                "            input = new " + nameMatrix + "();\n\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                String f = +y + "" + x;
                String t = +x+""+y;

                out.print("        output.a"+f+" = input.a"+t+";\n");
            }
        }

        out.print("\n        return output;\n" +
                "    }\n\n");
    }

    private void mult( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAdd" : "mult";

        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c "+plus+"= a * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> "+plus+"= &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void "+name+"( "+nameMatrix+" a , "+nameMatrix+" b , "+nameMatrix+" c) {\n");

        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                out.print("        c.a"+y+""+x+" "+plus+"= ");
                for( int k = 1; k <= dimen; k++ ) {
                    out.print("a.a"+y+""+k+"*b.a"+k+""+x);
                    if( k < dimen )
                        out.print(" + ");
                    else
                        out.print(";\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void multScale( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAdd" : "mult";

        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c "+plus+"= &alpha; * a * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> "+plus+"= &alpha; &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>kj</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha Scaling factor.\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void "+name+"( double alpha , "+nameMatrix+" a , "+nameMatrix+" b , "+nameMatrix+" c) {\n");

        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                out.print("        c.a"+y+""+x+" "+plus+"= alpha*(");
                for( int k = 1; k <= dimen; k++ ) {
                    out.print("a.a"+y+""+k+"*b.a"+k+""+x);
                    if( k < dimen )
                        out.print(" + ");
                    else
                        out.print(");\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void multTransA( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransA" : "multTransA";

        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c " + plus + "= a<sup>T</sup> * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> " + plus + "= &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * b<sub>kj</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= ");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + k + "" + y + "*b.a" + k + "" + x);
                    if (k < dimen)
                        out.print(" + ");
                    else
                        out.print(";\n");
                }
            }
        }
        out.printf("    }\n\n");
    }

    private void multTransAScale( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransA" : "multTransA";

        out.print("    /**\n" +
                "     * <p>Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c " + plus + "= &alpha; * a<sup>T</sup> * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> " + plus + "= &alpha; * &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * b<sub>kj</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha Scaling factor.\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( double alpha , " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= alpha*(");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + k + "" + y + "*b.a" + k + "" + x);
                    if (k < dimen)
                        out.print(" + ");
                    else
                        out.print(");\n");
                }
            }
        }
        out.printf("    }\n\n");
    }

    private void multTransAB( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransAB" : "multTransAB";

        out.printf("    /**\n" +
                "     * <p>\n" +
                "     * Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c " + plus + "= a<sup>T</sup> * b<sup>T</sup><br>\n" +
                "     * c<sub>ij</sub> " + plus + "= &sum;<sub>k=1:n</sub> { a<sub>ki</sub> * b<sub>jk</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= ");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + k + "" + y + "*b.a" + x + "" + k);
                    if (k < dimen)
                        out.print(" + ");
                    else
                        out.print(";\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void multTransABScale( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransAB" : "multTransAB";

        out.printf("    /**\n" +
                "     * <p>\n" +
                "     * Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c " + plus + "= &alpha;*a<sup>T</sup> * b<sup>T</sup><br>\n" +
                "     * c<sub>ij</sub> " + plus + "= &alpha;*&sum;<sub>k=1:n</sub> { a<sub>ki</sub> * b<sub>jk</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha Scaling factor.\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( double alpha , " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= alpha*(");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + k + "" + y + "*b.a" + x + "" + k);
                    if (k < dimen)
                        out.print(" + ");
                    else
                        out.print(");\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void multTransB( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransB" : "multTransB";

        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c "+plus+"= a * b<sup>T</sup> <br>\n" +
                "     * c<sub>ij</sub> "+plus+"= &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>jk</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= ");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + y + "" + k + "*b.a" + x + "" + k);
                    if (k < dimen )
                        out.print(" + ");
                    else
                        out.print(";\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void multTransBScale( int dimen , boolean add ){
        String plus = add ? "+" : "";
        String name = add ? "multAddTransB" : "multTransB";

        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs the following operation:<br>\n" +
                "     * <br>\n" +
                "     * c "+plus+"= &alpha; * a * b<sup>T</sup> <br>\n" +
                "     * c<sub>ij</sub> "+plus+"= &alpha;*&sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>jk</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha Scaling factor.\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void " + name + "( double alpha , " + nameMatrix + " a , " + nameMatrix + " b , " + nameMatrix + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++) {
                out.print("        c.a" + y + "" + x + " " + plus + "= alpha*(");
                for (int k = 1; k <= dimen; k++) {
                    out.print("a.a" + y + "" + k + "*b.a" + x + "" + k);
                    if (k < dimen )
                        out.print(" + ");
                    else
                        out.print(");\n");
                }
            }
        }
        out.print("    }\n\n");
    }

    private void mult_m_v_v( int dimen ){
        out.print("    /**\n" +
                "     * <p>Performs matrix to vector multiplication:<br>\n" +
                "     * <br>\n" +
                "     * c = a * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>i</sub> = &sum;<sub>k=1:n</sub> { a<sub>ik</sub> * b<sub>k</sub>}\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right vector in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void mult( " + nameMatrix + " a , " + nameVector + " b , " + nameVector + " c) {\n");
        for (int y = 1; y <= dimen; y++) {
            out.print("        c.a" + y + " = ");
            for (int x = 1; x <= dimen; x++) {
                out.print("a.a" + y + "" + x + "*b.a"+x);
                if( x < dimen )
                    out.print(" + ");
                else
                    out.print(";\n");
            }
        }
        out.printf("    }\n\n");

    }

    private void multAddOuter( int dimen ){
        out.print("    /**\n" +
                "     * C = &alpha;A + &beta;u*v<sup>T</sup>\n" +
                "     * \n" +
                "     * @param alpha scale factor applied to A\n" +
                "     * @param A matrix\n" +
                "     * @param beta scale factor applies to outer product\n" +
                "     * @param u vector\n" +
                "     * @param v vector\n" +
                "     * @param C Storage for solution. Can be same instance as A.\n" +
                "     */\n" +
                "    public static void multAddOuter( double alpha , " + nameMatrix + " A , double beta , " + nameVector + " u , " + nameVector + " v , "+nameMatrix+" C ) {\n");
        for (int i = 1; i <= dimen; i++) {
            for (int j = 1; j <= dimen; j++) {
                String m = i + "" + j;
                out.println("        C.a"+m+" = alpha*A.a"+m+" + beta*u.a"+i+"*v.a"+j+";");
            }
        }
        out.printf("    }\n\n");
    }


    private void mult_v_m_v( int dimen ){
        out.print("    /**\n" +
                "     * <p>Performs vector to matrix multiplication:<br>\n" +
                "     * <br>\n" +
                "     * c = a * b <br>\n" +
                "     * <br>\n" +
                "     * c<sub>j</sub> = &sum;<sub>k=1:n</sub> { b<sub>k</sub> * a<sub>kj</sub> }\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left vector in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void mult( "+nameVector+" a , "+nameMatrix+" b , "+nameVector+" c) {\n");

        for( int y = 1; y <= dimen; y++ ) {
            out.print("        c.a"+y+" = ");
            for( int x = 1; x <= dimen; x++ ) {
                out.print("a.a"+x+"*b.a"+x+""+y);
                if( x < dimen )
                    out.print(" + ");
                else
                    out.print(";\n");
            }
        }
        out.print("    }\n\n");

    }

    private void dot( int dimen ){
        out.print("    /**\n" +
                "     * <p>Performs the vector dot product:<br>\n" +
                "     * <br>\n" +
                "     * c = a * b <br>\n" +
                "     * <br>\n" +
                "     * c &ge; &sum;<sub>k=1:n</sub> { b<sub>k</sub> * a<sub>k</sub> }\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The left vector in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @return The dot product\n" +
                "     */\n" +
                "    public static double dot( "+nameVector+" a , "+nameVector+" b ) {\n");
        out.print("        return ");
        for( int i = 1; i <= dimen; i++) {
            out.print("a.a"+i+"*b.a"+i);
            if( i < dimen )
                out.print(" + ");
            else
                out.print(";\n");
        }
        out.print("    }\n\n");
    }

    private void setIdentity( int dimen ){
        out.print("    /**\n" +
                "     * Sets all the diagonal elements equal to one and everything else equal to zero.\n" +
                "     * If this is a square matrix then it will be an identity matrix.\n" +
                "     *\n" +
                "     * @param a A matrix.\n" +
                "     */\n" +
                "    public static void setIdentity( "+nameMatrix+" a ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                int val = x==y?1:0;
                out.print("a.a"+x+""+y+" = "+val+";");
                if( x < dimen )
                    out.print(" ");
                else
                    out.print("\n");
            }
        }
        out.print("    }\n\n");
    }

    private void invert( int dimen ){
        out.print("    /**\n" +
                "     * Inverts matrix 'a' using minor matrices and stores the results in 'inv'.  Scaling is applied to improve\n" +
                "     * stability against overflow and underflow.\n" +
                "     *\n" +
                "     * WARNING: Potentially less stable than using LU decomposition.\n" +
                "     *\n" +
                "     * @param a Input matrix. Not modified.\n" +
                "     * @param inv Inverted output matrix.  Modified.\n" +
                "     * @return true if it was successful or false if it failed.  Not always reliable.\n" +
                "     */\n" +
                "    public static boolean invert( "+nameMatrix+" a , "+nameMatrix+" inv ) {\n" +
                "\n" +
                "        double scale = 1.0/elementMaxAbs(a);\n" +
                "\n");

        int matrix[] = new int[dimen*dimen];
        int index = 0;
        for (int y = 1; y <= dimen; y++) {
            for (int x = 1; x <= dimen; x++, index++) {
                matrix[index] = index;
                String coor = y + "" + x;
                out.print("        double a" + coor + " = a.a" + coor + "*scale;\n");
            }
        }
        out.println();

        try {
            GenerateInverseFromMinor gen = new GenerateInverseFromMinor(false);
            gen.printMinors(matrix, dimen, out);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        out.println();

        for (int y = 1; y <= dimen; y++) {
            for( int x = 1; x <= dimen; x++ ) {
                String coor0 = y+""+x;
                String coor1 = x+""+y;
                out.print("        inv.a"+coor0+" = m"+coor1+"/det;\n");
            }
        }
        out.println();
        out.print("        return !Double.isNaN(det) && !Double.isInfinite(det);\n");
        out.print("    }\n\n");
    }

    private void cholL(int N ){
        out.print("    /**\n" +
                "     * Performs a lower Cholesky decomposition of matrix 'A' and stores result in A.\n" +
                "     *\n" +
                "     * @param A (Input) SPD Matrix. (Output) lower cholesky.\n"+
                "     * @return true if it was successful or false if it failed.  Not always reliable.\n" +
                "     */\n" +
                "    public static boolean cholL( "+nameMatrix+" A ) {\n" +
                "\n");

        for( int i = 1; i <= N; i++ ) {
            for( int j = 1; j <= N; j++ ) {
                if( j > i ) {
                    out.print("        A."+el(i,j)+" = 0;\n");
                } else if( i == j ) {
                    out.print("        A."+el(i,i)+" = Math.sqrt(A."+el(i,i));
                    for (int k = 1; k < j; k++) {
                        out.print("-A."+el(i,k)+"*A."+el(i,k));
                    }
                    out.println(");");
                } else {
                    out.print("        A." + el(i,j) + " = (A."+el(i,j));
                    for (int k = 1; k < j; k++) {
                        out.print("-A."+el(i,k)+"*A."+el(j,k));
                    }
                    out.println(")/A."+el(j,j)+";");
                }
            }
        }
        out.println("        return !UtilEjml.isUncountable(A."+el(N,N)+");");
        out.print("    }\n\n");
    }

    private void cholU(int N ){
        out.print("    /**\n" +
                "     * Performs an upper Cholesky decomposition of matrix 'A' and stores result in A.\n" +
                "     *\n" +
                "     * @param A (Input) SPD Matrix. (Output) upper cholesky.\n"+
                "     * @return true if it was successful or false if it failed.  Not always reliable.\n" +
                "     */\n" +
                "    public static boolean cholU( "+nameMatrix+" A ) {\n" +
                "\n");

        for( int j = 1; j <= N; j++ ) {
            for( int i = 1; i <= N; i++ ) {
                if( j < i ) {
                    out.println("        A." + el(i,j) + " = 0;");
                } else if( i == j ) {
                    out.print("        A." + el(i,i) + " = Math.sqrt(A."+el(i,i));
                    for (int k = 1; k < i; k++) {
                        out.print("-A."+el(k,i)+"*A."+el(k,i));
                    }
                    out.println(");");
                } else {
                    out.print("        A." + el(i,j)+ " = (A."+el(i,j));
                    for (int k = 1; k < i; k++) {
                        out.print("-A."+el(k,i)+"*A."+el(k,j));
                    }
                    out.println(")/A."+el(i,i)+";");
                }
            }
        }
        out.println("        return !UtilEjml.isUncountable(A."+el(N,N)+");");
        out.print("    }\n\n");
    }

    private static String el( int row , int col ) {
        return "a"+row+""+col;
    }

    private void trace(int dimen) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * This computes the trace of the matrix:<br>\n" +
                "     * <br>\n" +
                "     * trace = &sum;<sub>i=1:n</sub> { a<sub>ii</sub> }\n" +
                "     * </p>\n" +
                "     * <p>\n" +
                "     * The trace is only defined for square matrices.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A square matrix.  Not modified.\n" +
                "     */\n" +
                "    public static double trace( "+nameMatrix+" a ) {\n");
        out.print("        return ");
        for( int i = 1; i <= dimen; i++ ) {
            out.print("a.a"+i+""+i);
            if( i < dimen )
                out.print(" + ");
            else
                out.println(";");
        }
        out.print("    }\n\n");
    }

    private void det( int dimen ){
        out.print("    /**\n" +
                "     * Computes the determinant using minor matrices.<br>\n" +
                "     * WARNING: Potentially less stable than using LU decomposition.\n" +
                "     *\n" +
                "     * @param mat Input matrix.  Not modified.\n" +
                "     * @return The determinant.\n" +
                "     */\n" +
                "    public static double det( "+nameMatrix+" mat ) {\n" +
                "\n");
        if( dimen == 2 ) {
            out.print("        return mat.a11*mat.a22 - mat.a12*mat.a21;\n");
        } else if( dimen == 3 ) {
            out.print( "        double a = mat.a11*(mat.a22*mat.a33 - mat.a23*mat.a32);\n" +
                    "        double b = mat.a12*(mat.a21*mat.a33 - mat.a23*mat.a31);\n" +
                    "        double c = mat.a13*(mat.a21*mat.a32 - mat.a31*mat.a22);\n" +
                    "\n" +
                    "        return a-b+c;\n");
        } else {
            GenerateDeterminantFromMinor helper = new GenerateDeterminantFromMinor(out) {
                @Override
                protected String getInputValue(int element) {
                    int row = element/(N+1) + 1;
                    int col = element%(N+1) + 1;
                    return "mat.a"+row+""+col;
                }
            };
            helper.printFunctionInner(dimen);
            out.print("\n        return ret;\n");
        }

        out.print("    }\n\n");
    }

    private void diag( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Extracts all diagonal elements from 'input' and places them inside the 'out' vector. Elements\n" +
                "     * are in sequential order.\n" +
                "     * </p>\n" +
                "     *\n" +
                "     *\n" +
                "     * @param input Matrix.  Not modified.\n" +
                "     * @param out Vector containing diagonal elements.  Modified.\n" +
                "     */\n" +
                "    public static void diag( "+nameMatrix+" input , "+nameVector+" out ) {\n");
        for( int i = 1; i <= dimen; i++ ) {
            out.print("        out.a" + i + " = input.a" + i + "" + i + ";\n");
        }
        out.print("    }\n\n");
    }

    private void elementMax( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the value of the element in the matrix that has the largest value.<br>\n" +
                "     * <br>\n" +
                "     * Max{ a<sub>ij</sub> } for all i and j<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The max element value of the matrix.\n" +
                "     */\n" +
                "    public static double elementMax( "+nameMatrix+" a ) {\n");

        out.print("        double max = a.a11;\n");
        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                if( y == 1 && x == 1 )
                    continue;
                String e = "a.a"+y+""+x;
                out.print("        if( "+e+" > max ) max = "+e+";\n");
            }
        }
        out.print("\n" +
                "        return max;\n" +
                "    }\n\n");
    }

    private void elementMax_vector( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the value of the element in the vector that has the largest value.<br>\n" +
                "     * <br>\n" +
                "     * Max{ a<sub>i</sub> } for all i<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A vector. Not modified.\n" +
                "     * @return The max element value of the matrix.\n" +
                "     */\n" +
                "    public static double elementMax( "+nameVector+" a ) {\n");

        out.print("        double max = a.a1;\n");
        for( int y = 2; y <= dimen; y++ ) {
            String e = "a.a"+y;
            out.print("        if( "+e+" > max ) max = "+e+";\n");
        }
        out.print("\n" +
                "        return max;\n" +
                "    }\n\n");
    }

    private void elementMaxAbs( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the absolute value of the element in the matrix that has the largest absolute value.<br>\n" +
                "     * <br>\n" +
                "     * Max{ |a<sub>ij</sub>| } for all i and j<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The max abs element value of the matrix.\n" +
                "     */\n" +
                "    public static double elementMaxAbs( "+nameMatrix+" a ) {\n");

        out.print("        double max = Math.abs(a.a11);\n");
        out.print("        double tmp = Math.abs(a.a12); if( tmp > max ) max = tmp;\n");
        for (int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                if( y == 1 && x <= 2)
                    continue;
                out.print("        tmp = Math.abs(a.a"+y+""+x+"); if( tmp > max ) max = tmp;\n");
            }
        }
        out.print("\n" +
                "        return max;\n" +
                "    }\n\n");
    }

    private void elementMaxAbs_vector( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the absolute value of the element in the vector that has the largest absolute value.<br>\n" +
                "     * <br>\n" +
                "     * Max{ |a<sub>i</sub>| } for all i<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The max abs element value of the vector.\n" +
                "     */\n" +
                "    public static double elementMaxAbs( "+nameVector+" a ) {\n");

        out.print("        double max = Math.abs(a.a1);\n");
        out.print("        double tmp = Math.abs(a.a2); if( tmp > max ) max = tmp;\n");
        for( int y = 2; y <= dimen; y++ ) {
            out.print("        tmp = Math.abs(a.a"+y+"); if( tmp > max ) max = tmp;\n");
        }
        out.print("\n" +
                "        return max;\n" +
                "    }\n\n");
    }

    private void elementMin( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the value of the element in the matrix that has the minimum value.<br>\n" +
                "     * <br>\n" +
                "     * Min{ a<sub>ij</sub> } for all i and j<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The value of element in the matrix with the minimum value.\n" +
                "     */\n" +
                "    public static double elementMin( "+nameMatrix+" a ) {\n");

        out.print("        double min = a.a11;\n");
        for( int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                if( y == 1 && x == 1 )
                    continue;
                String e = "a.a"+y+""+x;
                out.print("        if( "+e+" < min ) min = "+e+";\n");
            }
        }
        out.print("\n" +
                "        return min;\n" +
                "    }\n\n");
    }

    private void elementMin_vector( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the value of the element in the vector that has the minimum value.<br>\n" +
                "     * <br>\n" +
                "     * Min{ a<sub>i</sub> } for all<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The value of element in the vector with the minimum value.\n" +
                "     */\n" +
                "    public static double elementMin( "+nameVector+" a ) {\n");

        out.print("        double min = a.a1;\n");
        for( int y = 2; y <= dimen; y++ ) {
            String e = "a.a"+y;
            out.print("        if( "+e+" < min ) min = "+e+";\n");
        }
        out.print("\n" +
                "        return min;\n" +
                "    }\n\n");
    }

    private void elementMinAbs( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the absolute value of the element in the matrix that has the smallest absolute value.<br>\n" +
                "     * <br>\n" +
                "     * Min{ |a<sub>ij</sub>| } for all i and j<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The max element value of the matrix.\n" +
                "     */\n" +
                "    public static double elementMinAbs( "+nameMatrix+" a ) {\n");

        out.print("        double min = Math.abs(a.a11);\n");
        out.print("        double tmp = Math.abs(a.a12); if( tmp < min ) min = tmp;\n");
        for (int y = 1; y <= dimen; y++ ) {
            for( int x = 1; x <= dimen; x++ ) {
                if( y == 1 && x <= 2)
                    continue;
                out.print("        tmp = Math.abs(a.a"+y+""+x+"); if( tmp < min ) min = tmp;\n");
            }
        }
        out.print("\n" +
                "        return min;\n" +
                "    }\n\n");
    }


    private void elementMinAbs_vector(int dimen) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Returns the absolute value of the element in the vector that has the smallest absolute value.<br>\n" +
                "     * <br>\n" +
                "     * Min{ |a<sub>i</sub>| } for all i<br>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Not modified.\n" +
                "     * @return The max element value of the vector.\n" +
                "     */\n" +
                "    public static double elementMinAbs( "+nameVector+" a ) {\n");

        out.print("        double min = Math.abs(a.a1);\n");
        out.print("        double tmp = Math.abs(a.a1); if( tmp < min ) min = tmp;\n");
        for( int y = 2; y <= dimen; y++ ) {
            out.print("        tmp = Math.abs(a.a"+y+"); if( tmp < min ) min = tmp;\n");
        }
        out.print("\n" +
                "        return min;\n" +
                "    }\n\n");
    }

    private void elementMult_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element multiplication operation:<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = a<sub>ij</sub> * b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left matrix in the multiplication operation. Modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     */\n" +
                "    public static void elementMult( " + nameMatrix + " a , " + nameMatrix + " b) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" *= b."+w+ ";");
                if (x < dimen)
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void elementMult_vector_two(int dimen) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element multiplication operation:<br>\n" +
                "     * <br>\n" +
                "     * a<sub>i</sub> = a<sub>i</sub> * b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left vector in the multiplication operation. Modified.\n" +
                "     * @param b The right vector in the multiplication operation. Not modified.\n" +
                "     */\n" +
                "    public static void elementMult( "+nameVector+" a , "+nameVector+" b) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" *= b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void elementMult_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element multiplication operation:<br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> = a<sub>ij</sub> * b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left matrix in the multiplication operation. Not modified.\n" +
                "     * @param b The right matrix in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void elementMult( "+nameMatrix+" a , "+nameMatrix+" b , "+nameMatrix+" c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("c."+w+" = a."+w+"*b."+w+";");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void elementMult_vector_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element multiplication operation:<br>\n" +
                "     * <br>\n" +
                "     * c<sub>i</sub> = a<sub>i</sub> * b<sub>j</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left vector in the multiplication operation. Not modified.\n" +
                "     * @param b The right vector in the multiplication operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void elementMult( "+nameVector+" a , "+nameVector+" b , "+nameVector+" c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        c.a"+y+" = a.a"+y+"*b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void elementDiv_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element division operation:<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = a<sub>ij</sub> / b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left matrix in the division operation. Modified.\n" +
                "     * @param b The right matrix in the division operation. Not modified.\n" +
                "     */\n" +
                "    public static void elementDiv( "+nameMatrix+" a , "+nameMatrix+" b) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" /= b."+w+";");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void elementDiv_vector_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element division operation:<br>\n" +
                "     * <br>\n" +
                "     * a<sub>i</sub> = a<sub>i</sub> / b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left vector in the division operation. Modified.\n" +
                "     * @param b The right vector in the division operation. Not modified.\n" +
                "     */\n" +
                "    public static void elementDiv( "+nameVector+" a , "+nameVector+" b) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" /= b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void elementDiv_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element division operation:<br>\n" +
                "     * <br>\n" +
                "     * c<sub>ij</sub> = a<sub>ij</sub> / b<sub>ij</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left matrix in the division operation. Not modified.\n" +
                "     * @param b The right matrix in the division operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void elementDiv( "+nameMatrix+" a , "+nameMatrix+" b , "+nameMatrix+" c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("c."+w+" = a."+w+"/b."+w+";");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void elementDiv_vector_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>Performs an element by element division operation:<br>\n" +
                "     * <br>\n" +
                "     * c<sub>i</sub> = a<sub>i</sub> / b<sub>i</sub> <br>\n" +
                "     * </p>\n" +
                "     * @param a The left vector in the division operation. Not modified.\n" +
                "     * @param b The right vector in the division operation. Not modified.\n" +
                "     * @param c Where the results of the operation are stored. Modified.\n" +
                "     */\n" +
                "    public static void elementDiv( "+nameVector+" a , "+nameVector+" b , "+nameVector+" c ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        c.a"+y+" = a.a"+y+"/b.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void scale_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an in-place element by element scalar multiplication.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = &alpha;*a<sub>ij</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The matrix that is to be scaled.  Modified.\n" +
                "     * @param alpha the amount each element is multiplied by.\n" +
                "     */\n" +
                "    public static void scale( double alpha , "+nameMatrix+" a ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" *= alpha;");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void scale_vector_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an in-place element by element scalar multiplication.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = &alpha;*a<sub>ij</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The vector that is to be scaled.  Modified.\n" +
                "     * @param alpha the amount each element is multiplied by.\n" +
                "     */\n" +
                "    public static void scale( double alpha , "+nameVector+" a ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" *= alpha;\n");
        }
        out.print("    }\n\n");
    }

    private void scale_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an element by element scalar multiplication.<br>\n" +
                "     * <br>\n" +
                "     * b<sub>ij</sub> = &alpha;*a<sub>ij</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha the amount each element is multiplied by.\n" +
                "     * @param a The matrix that is to be scaled.  Not modified.\n" +
                "     * @param b Where the scaled matrix is stored. Modified.\n" +
                "     */\n" +
                "    public static void scale( double alpha , "+nameMatrix+" a , "+nameMatrix+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("b."+w+" = a."+w+"*alpha;");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void scale_vector_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an element by element scalar multiplication.<br>\n" +
                "     * <br>\n" +
                "     * b<sub>i</sub> = &alpha;*a<sub>i</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha the amount each element is multiplied by.\n" +
                "     * @param a The vector that is to be scaled.  Not modified.\n" +
                "     * @param b Where the scaled matrix is stored. Modified.\n" +
                "     */\n" +
                "    public static void scale( double alpha , "+nameVector+" a , "+nameVector+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        b.a"+y+" = a.a"+y+"*alpha;\n");
        }
        out.print("    }\n\n");
    }

    private void divide_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an in-place element by element scalar division. Scalar denominator.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = a<sub>ij</sub>/&alpha;\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The matrix whose elements are to be divided.  Modified.\n" +
                "     * @param alpha the amount each element is divided by.\n" +
                "     */\n" +
                "    public static void divide( "+nameMatrix+" a , double alpha ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" /= alpha;");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void divide_vector_two( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an in-place element by element scalar division. Scalar denominator.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>i</sub> = a<sub>i</sub>/&alpha;\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a The vector whose elements are to be divided.  Modified.\n" +
                "     * @param alpha the amount each element is divided by.\n" +
                "     */\n" +
                "    public static void divide( "+nameVector+" a , double alpha ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" /= alpha;\n");
        }
        out.print("    }\n\n");
    }

    private void divide_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an element by element scalar division.  Scalar denominator.<br>\n" +
                "     * <br>\n" +
                "     * b<sub>ij</sub> = a<sub>ij</sub> /&alpha;\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha the amount each element is divided by.\n" +
                "     * @param a The matrix whose elements are to be divided.  Not modified.\n" +
                "     * @param b Where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void divide( "+nameMatrix+" a , double alpha , "+nameMatrix+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("b."+w+" = a."+w+"/alpha;");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void divide_vector_three( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Performs an element by element scalar division.  Scalar denominator.<br>\n" +
                "     * <br>\n" +
                "     * b<sub>i</sub> = a<sub>i</sub> /&alpha;\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param alpha the amount each element is divided by.\n" +
                "     * @param a The vector whose elements are to be divided.  Not modified.\n" +
                "     * @param b Where the results are stored. Modified.\n" +
                "     */\n" +
                "    public static void divide( "+nameVector+" a , double alpha , "+nameVector+" b ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
                out.print("        b.a"+y+" = a.a"+y+"/alpha;\n");
        }
        out.print("    }\n\n");
    }

    private void changeSign( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Changes the sign of every element in the matrix.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = -a<sub>ij</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A matrix. Modified.\n" +
                "     */\n" +
                "    public static void changeSign( "+nameMatrix+" a )\n" +
                "    {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" = -a."+w+";");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
        out.print("    }\n\n");
    }

    private void changeSign_vector( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Changes the sign of every element in the vector.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>i</sub> = -a<sub>i</sub>\n" +
                "     * </p>\n" +
                "     *\n" +
                "     * @param a A vector. Modified.\n" +
                "     */\n" +
                "    public static void changeSign( "+nameVector+" a )\n" +
                "    {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" = -a.a"+y+";\n");
        }
        out.print("    }\n\n");
    }

    private void fill( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Sets every element in the matrix to the specified value.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>ij</sub> = value\n" +
                "     * <p>\n" +
                "     *\n" +
                "     * @param a A matrix whose elements are about to be set. Modified.\n" +
                "     * @param v The value each element will have.\n" +
                "     */\n" +
                "    public static void fill( "+nameMatrix+" a , double v  ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        ");
            for( int x = 1; x <= dimen; x++ ) {
                String w = "a"+y+""+x;
                out.print("a."+w+" = v;");
                if( x < dimen )
                    out.print(" ");
                else
                    out.println();
            }
        }
                out.print("    }\n\n");
    }

    private void fill_vector( int dimen ) {
        out.print("    /**\n" +
                "     * <p>\n" +
                "     * Sets every element in the vector to the specified value.<br>\n" +
                "     * <br>\n" +
                "     * a<sub>i</sub> = value\n" +
                "     * <p>\n" +
                "     *\n" +
                "     * @param a A vector whose elements are about to be set. Modified.\n" +
                "     * @param v The value each element will have.\n" +
                "     */\n" +
                "    public static void fill( "+nameVector+" a , double v  ) {\n");
        for( int y = 1; y <= dimen; y++ ) {
            out.print("        a.a"+y+" = v;\n");
        }
        out.print("    }\n\n");
    }

    private void extract( int dimen ) {
        out.print("    /**\n" +
                "     * Extracts the row from the matrix a.\n" +
                "     * @param a Input matrix\n" +
                "     * @param row Which row is to be extracted\n" +
                "     * @param out output. Storage for the extracted row. If null then a new vector will be returned.\n" +
                "     * @return The extracted row.\n" +
                "     */\n" +
                "    public static "+nameVector+" extractRow( "+nameMatrix+" a , int row , "+nameVector+" out ) {\n" +
                "        if( out == null) out = new "+nameVector+"();\n" +
                "        switch( row ) {\n");
        for (int i = 0; i < dimen; i++) {
            out.print("            case "+i+":\n");
            for (int j = 0; j < dimen; j++) {
                int n = j+1;
                out.print("                out.a"+n+" = a.a"+(i+1)+""+n+";\n");
            }
            out.print("            break;\n");
        }
        out.print("            default:\n" +
                "                throw new IllegalArgumentException(\"Out of bounds row.  row = \"+row);\n" +
                "        }\n" +
                "        return out;\n" +
                "    }\n" +
                "\n" +
                "    /**\n" +
                "     * Extracts the column from the matrix a.\n" +
                "     * @param a Input matrix\n" +
                "     * @param column Which column is to be extracted\n" +
                "     * @param out output. Storage for the extracted column. If null then a new vector will be returned.\n" +
                "     * @return The extracted column.\n" +
                "     */\n" +
                "    public static "+nameVector+" extractColumn( "+nameMatrix+" a , int column , "+nameVector+" out ) {\n" +
                "        if( out == null) out = new "+nameVector+"();\n" +
                "        switch( column ) {\n");
        for (int i = 0; i < dimen; i++) {
            out.print("            case "+i+":\n");
            for (int j = 0; j < dimen; j++) {
                int n = j+1;
                out.print("                out.a"+n+" = a.a"+n+""+(i+1)+";\n");
            }
            out.print("            break;\n");
        }
        out.print("            default:\n" +
                "                throw new IllegalArgumentException(\"Out of bounds column.  column = \"+column);\n" +
                "        }\n" +
                "        return out;\n" +
                "    }\n\n");
    }

    public static void main( String args[] ) throws FileNotFoundException {
        GenerateCommonOps_DDF app = new GenerateCommonOps_DDF();

        app.generate();
    }

}