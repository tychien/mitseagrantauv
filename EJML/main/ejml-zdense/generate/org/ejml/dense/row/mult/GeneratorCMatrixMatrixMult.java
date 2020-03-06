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

import org.ejml.CodeGeneratorMisc;

import java.io.FileNotFoundException;
import java.io.PrintStream;

/**
 * @author Peter Abeles
 */
public class GeneratorCMatrixMatrixMult {
    PrintStream stream;

    public GeneratorCMatrixMatrixMult(String fileName) throws FileNotFoundException {
        stream = new PrintStream(fileName);
    }

    public void createClass() {
        String preamble = CodeGeneratorMisc.COPYRIGHT +
                "package org.ejml.dense.row.mult;\n" +
                "\n" +
                "import org.ejml.data.ZMatrixRMaj;\n" +
                "import CommonOps_ZDRM;\n" +
                "import MatrixDimensionException;\n" +
                "\n" +
                "/**\n" +
                " * <p>Matrix multiplication routines for complex row matrices in a row-major format.</p>\n" +
                " *\n" +
                " * <p>\n" +
                " * DO NOT MODIFY! Auto generated by "+getClass().getCanonicalName()+".\n" +
                " * </p>\n" +
                " *\n" +
                " * @author Peter Abeles\n" +
                " */\n" +
                "@SuppressWarnings(\"Duplicates\")\n" +
                "public class MatrixMatrixMult_ZDRM {\n";

        stream.print(preamble);

        for( int i = 0; i < 2; i++ ) {
            boolean alpha = i == 1;
            for( int j = 0; j < 2; j++ ) {
                boolean add = j == 1;
                printMult_reroder(alpha,add);
                stream.print("\n");
                printMult_small(alpha,add);
                stream.print("\n");
                printMultTransA_reorder(alpha,add);
                stream.print("\n");
                printMultTransA_small(alpha,add);
                stream.print("\n");
                printMultTransB(alpha,add);
                stream.print("\n");
                printMultTransAB(alpha,add);
                stream.print("\n");
                printMultTransAB_aux(alpha,add);
                stream.print("\n");
            }
        }
        stream.print("}\n");
    }

    public void printMult_reroder( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult","reorder",add,alpha, false, false,false);

        String tempVars = "";

        if( alpha ) {
            tempVars = "        double realTmp,imagTmp;";
            valLine = "            realTmp = a.data[indexA++];\n" +
                      "            imagTmp = a.data[indexA++];\n" +
                      "            realA = realAlpha*realTmp - imagAlpha*imagTmp;\n" +
                      "            imagA = realAlpha*imagTmp + imagAlpha*realTmp;\n";
        } else {
            valLine = "                realA = a.data[indexA++];\n" +
                      "                imagA = a.data[indexA++];\n";
        }

        String assignment = add ? "+=" : "=";

        String foo = header + makeBoundsCheck(false,false, null)+handleZeros(add) +
                "        double realA,imagA;\n" +
                tempVars +
                "\n" +
                "        int indexCbase= 0;\n" +
                "        int strideA = a.getRowStride();\n" +
                "        int strideB = b.getRowStride();\n" +
                "        int strideC = c.getRowStride();\n" +
                "        int endOfKLoop = b.numRows*strideB;\n" +
                "\n" +
                "        for( int i = 0; i < a.numRows; i++ ) {\n" +
                "            int indexA = i*strideA;\n" +
                "\n" +
                "            // need to assign c.data to a value initially\n" +
                "            int indexB = 0;\n" +
                "            int indexC = indexCbase;\n" +
                "            int end = indexB + strideB;\n" +
                "\n" +
                valLine +
                "\n" +
                "            while( indexB < end ) {\n" +
                "                double realB = b.data[indexB++];\n" +
                "                double imgB = b.data[indexB++];\n" +
                "\n" +
                "                c.data[indexC++] "+assignment+" realA*realB - imagA*imgB;\n" +
                "                c.data[indexC++] "+assignment+" realA*imgB + imagA*realB;\n" +
                "            }\n" +
                "\n" +
                "            // now add to it\n" +
                "            while( indexB != endOfKLoop ) { // k loop\n" +
                "                indexC = indexCbase;\n" +
                "                end = indexB + strideB;\n" +
                "\n" +
                valLine +
                "\n" +
                "                while( indexB < end ) { // j loop\n" +
                "                    double realB = b.data[indexB++];\n" +
                "                    double imgB = b.data[indexB++];\n" +
                "\n" +
                "                    c.data[indexC++] += realA*realB - imagA*imgB;\n" +
                "                    c.data[indexC++] += realA*imgB + imagA*realB;\n" +
                "                }\n" +
                "            }\n" +
                "            indexCbase += strideC;\n" +
                "        }\n" +
                "    }\n\n";

        stream.print(foo);
    }

    public void printMult_small( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult","small",add,alpha, false, false,false);

        String assignment = add ? "+=" : "=";

        if( alpha ) {
            valLine = "                c.data[indexC++] "+assignment+" realAlpha*realTotal - imagAlpha*imgTotal;\n" +
                      "                c.data[indexC++] "+assignment+" realAlpha*imgTotal + imagAlpha*realTotal;\n";
        } else {
            valLine = "                c.data[indexC++] "+assignment+" realTotal;\n" +
                      "                c.data[indexC++] "+assignment+" imgTotal;\n";
        }

        String foo =
                header + makeBoundsCheck(false,false, null)+
                        "        int aIndexStart = 0;\n" +
                        "        int indexC = 0;\n" +
                        "\n" +
                        "        int strideA = a.getRowStride();\n" +
                        "        int strideB = b.getRowStride();\n" +
                        "\n" +
                        "        for( int i = 0; i < a.numRows; i++ ) {\n" +
                        "            for( int j = 0; j < b.numCols; j++ ) {\n" +
                        "                double realTotal = 0;\n" +
                        "                double imgTotal = 0;\n" +
                        "\n" +
                        "                int indexA = aIndexStart;\n" +
                        "                int indexB = j*2;\n" +
                        "                int end = indexA + strideA;\n" +
                        "                while( indexA < end ) {\n" +
                        "                    double realA = a.data[indexA++];\n" +
                        "                    double imagA = a.data[indexA++];\n" +
                        "\n" +
                        "                    double realB = b.data[indexB];\n" +
                        "                    double imgB = b.data[indexB+1];\n" +
                        "\n" +
                        "                    realTotal += realA*realB - imagA*imgB;\n" +
                        "                    imgTotal += realA*imgB + imagA*realB;\n" +
                        "\n" +
                        "                    indexB += strideB;\n" +
                        "                }\n" +
                        "\n" +
                        valLine +
                        "            }\n" +
                        "            aIndexStart += strideA;\n" +
                        "        }\n" +
                        "    }\n\n";

        stream.print(foo);
    }

    public void printMultTransA_reorder( boolean alpha , boolean add ) {
        String header,valLine1,valLine2;

        header = makeHeader("mult","reorder",add,alpha, false, true,false);

        String assignment = add ? "+=" : "=";

        String tempVars = "";

        if( alpha ) {
            tempVars = "        double realTmp,imagTmp;\n";
            valLine1 = "            realTmp = a.data[i*2];\n" +
                       "            imagTmp = a.data[i*2+1];\n" +
                       "            realA = realAlpha*realTmp + imagAlpha*imagTmp;\n" +
                       "            imagA = realAlpha*imagTmp - imagAlpha*realTmp;\n";

            valLine2 = "            realTmp = a.getReal(k,i);\n" +
                       "            imagTmp = a.getImaginary(k,i);\n" +
                       "            realA = realAlpha*realTmp + imagAlpha*imagTmp;\n" +
                       "            imagA = realAlpha*imagTmp - imagAlpha*realTmp;\n";
        } else {
            valLine1 = "            realA = a.data[i*2];\n" +
                       "            imagA = a.data[i*2+1];\n";
            valLine2 = "            realA = a.getReal(k,i);\n" +
                       "            imagA = a.getImaginary(k,i);\n";
        }

        String foo =
                header + makeBoundsCheck(true,false, null)+handleZeros(add)+
                        "        double realA,imagA;\n" +
                        tempVars +
                        "\n" +
                        "        for( int i = 0; i < a.numCols; i++ ) {\n" +
                        "            int indexC_start = i*c.numCols*2;\n" +
                        "\n" +
                        "            // first assign R\n" +
                        valLine1 +
                        "            int indexB = 0;\n" +
                        "            int end = indexB+b.numCols*2;\n" +
                        "            int indexC = indexC_start;\n" +
                        "            while( indexB<end ) {\n" +
                        "                double realB = b.data[indexB++];\n" +
                        "                double imagB = b.data[indexB++];\n" +
                        "                c.data[indexC++] "+assignment+" realA*realB + imagA*imagB;\n" +
                        "                c.data[indexC++] "+assignment+" realA*imagB - imagA*realB;\n" +
                        "            }\n" +
                        "            // now increment it\n" +
                        "            for( int k = 1; k < a.numRows; k++ ) {\n" +
                        valLine2+
                        "                end = indexB+b.numCols*2;\n" +
                        "                indexC = indexC_start;\n" +
                        "                // this is the loop for j\n" +
                        "                while( indexB<end ) {\n" +
                        "                    double realB = b.data[indexB++];\n" +
                        "                    double imagB = b.data[indexB++];\n" +
                        "                    c.data[indexC++] += realA*realB + imagA*imagB;\n" +
                        "                    c.data[indexC++] += realA*imagB - imagA*realB;\n" +
                        "                }\n" +
                        "            }\n" +
                        "        }\n" +
                        "    }\n";
        stream.print(foo);
    }

    public void printMultTransA_small( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult","small",add,alpha, false, true,false);

        String assignment = add ? "+=" : "=";

        if( alpha ) {
            valLine = "                c.data[indexC++] "+assignment+" realAlpha*realTotal - imagAlpha*imagTotal;\n" +
                      "                c.data[indexC++] "+assignment+" realAlpha*imagTotal + imagAlpha*realTotal;\n";
        } else {
            valLine = "                c.data[indexC++] "+assignment+" realTotal;\n" +
                      "                c.data[indexC++] "+assignment+" imagTotal;\n";
        }

        String foo =
                header + makeBoundsCheck(true,false, null)+
                        "        int indexC = 0;\n" +
                        "\n" +
                        "        for( int i = 0; i < a.numCols; i++ ) {\n" +
                        "            for( int j = 0; j < b.numCols; j++ ) {\n" +
                        "                int indexA = i*2;\n" +
                        "                int indexB = j*2;\n" +
                        "                int end = indexB + b.numRows*b.numCols*2;\n" +
                        "\n" +
                        "                double realTotal = 0;\n" +
                        "                double imagTotal = 0;\n" +
                        "\n" +
                        "                // loop for k\n" +
                        "                for(; indexB < end; indexB += b.numCols*2 ) {\n" +
                        "                    double realA = a.data[indexA];\n" +
                        "                    double imagA = a.data[indexA+1];\n" +
                        "                    double realB = b.data[indexB];\n" +
                        "                    double imagB = b.data[indexB+1];\n" +
                        "                    realTotal += realA*realB + imagA*imagB;\n" +
                        "                    imagTotal += realA*imagB - imagA*realB;\n" +
                        "                    indexA += a.numCols*2;\n" +
                        "                }\n" +
                        "\n" +
                        valLine +
                        "            }\n" +
                        "        }\n" +
                        "    }\n";

         stream.print(foo);
    }

    public void printMultTransB( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult",null,add,alpha, false, false,true);

        String assignment = add ? "+=" : "=";

        if( alpha ) {
            valLine = "                c.data[indexC++] "+assignment+" realAlpha*realTotal - imagAlpha*imagTotal;\n" +
                      "                c.data[indexC++] "+assignment+" realAlpha*imagTotal + imagAlpha*realTotal;\n";
        } else {
            valLine = "                c.data[indexC++] "+assignment+" realTotal;\n" +
                      "                c.data[indexC++] "+assignment+" imagTotal;\n";
        }

        String foo =
                header + makeBoundsCheck(false,true, null)+
                        "        int indexC = 0;\n" +
                        "        int aIndexStart = 0;\n" +
                        "\n" +
                        "        for( int xA = 0; xA < a.numRows; xA++ ) {\n" +
                        "            int end = aIndexStart + b.numCols*2;\n" +
                        "            int indexB = 0;\n"+
                        "            for( int xB = 0; xB < b.numRows; xB++ ) {\n" +
                        "                int indexA = aIndexStart;\n" +
                        "\n" +
                        "                double realTotal = 0;\n" +
                        "                double imagTotal = 0;\n" +
                        "\n" +
                        "                while( indexA<end ) {\n" +
                        "                    double realA = a.data[indexA++];\n" +
                        "                    double imagA = a.data[indexA++];\n" +
                        "                    double realB = b.data[indexB++];\n" +
                        "                    double imagB = b.data[indexB++];\n" +
                        "                    realTotal += realA*realB + imagA*imagB;\n" +
                        "                    imagTotal += imagA*realB - realA*imagB;\n" +
                        "                }\n" +
                        "\n" +
                        valLine +
                        "            }\n" +
                        "            aIndexStart += a.numCols*2;\n" +
                        "        }\n" +
                        "    }\n";
        stream.print(foo);
    }

    public void printMultTransAB( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult",null,add,alpha, false, true,true);

        String assignment = add ? "+=" : "=";

        if( alpha ) {
            valLine = "                c.data[indexC++] "+assignment+" realAlpha*realTotal - imagAlpha*imagTotal;\n" +
                      "                c.data[indexC++] "+assignment+" realAlpha*imagTotal + imagAlpha*realTotal;\n";
        } else {
            valLine = "                c.data[indexC++] "+assignment+" realTotal;\n" +
                      "                c.data[indexC++] "+assignment+" imagTotal;\n";
        }

        String foo =
                header + makeBoundsCheck(true,true, null)+
                        "        int indexC = 0;\n" +
                        "\n" +
                        "        for( int i = 0; i < a.numCols; i++ ) {\n" +
                        "            int indexB = 0;\n"+
                        "            for( int j = 0; j < b.numRows; j++ ) {\n" +
                        "                int indexA = i*2;\n" +
                        "                int end = indexB + b.numCols*2;\n" +
                        "\n" +
                        "                double realTotal = 0;\n" +
                        "                double imagTotal = 0;\n" +
                        "\n" +
                        "                for( ;indexB<end; ) {\n" +
                        "                    double realA = a.data[indexA];\n" +
                        "                    double imagA = -a.data[indexA+1];\n" +
                        "                    double realB = b.data[indexB++];\n" +
                        "                    double imagB = -b.data[indexB++];\n" +
                        "                    realTotal += realA*realB - imagA*imagB;\n" +
                        "                    imagTotal += realA*imagB + imagA*realB;\n" +
                        "                    indexA += a.numCols*2;\n" +
                        "                }\n" +
                        "\n" +
                        valLine+
                        "            }\n" +
                        "        }\n"+
                        "    }\n";
        stream.print(foo);
    }

    public void printMultTransAB_aux( boolean alpha , boolean add ) {
        String header,valLine;

        header = makeHeader("mult","aux",add,alpha, true, true,true);

        String assignment = add ? "+=" : "=";

        if( alpha ) {
            valLine = "                c.data[indexC++] "+assignment+" realAlpha*realTotal - imagAlpha*imagTotal;\n" +
                      "                c.data[indexC++] "+assignment+" realAlpha*imagTotal + imagAlpha*realTotal;\n";
        } else {
            valLine = "                c.data[indexC++] "+assignment+" realTotal;\n" +
                      "                c.data[indexC++] "+assignment+" imagTotal;\n";
        }

        String foo =
                header + makeBoundsCheck(true,true, "a.numRows")+handleZeros(add)+
                        "        int indexC = 0;\n" +
                        "        for( int i = 0; i < a.numCols; i++ ) {\n" +
                        "            int indexA = i*2;\n" +
                        "            for( int k = 0; k < b.numCols; k++ ) {\n" +
                        "                aux[k*2]   = a.data[indexA];\n" +
                        "                aux[k*2+1] = a.data[indexA+1];\n" +
                        "                indexA += a.numCols*2;\n" +
                        "            }\n" +
                        "\n" +
                        "            for( int j = 0; j < b.numRows; j++ ) {\n" +
                        "                int indexAux = 0;\n" +
                        "                int indexB = j*b.numCols*2;\n" +
                        "                double realTotal = 0;\n" +
                        "                double imagTotal = 0;\n" +
                        "\n" +
                        "                for( int k = 0; k < b.numCols; k++ ) {\n" +
                        "                    double realA = aux[indexAux++];\n" +
                        "                    double imagA = -aux[indexAux++];\n" +
                        "                    double realB = b.data[indexB++];\n" +
                        "                    double imagB = -b.data[indexB++];\n" +
                        "                    realTotal += realA*realB - imagA*imagB;\n" +
                        "                    imagTotal += realA*imagB + imagA*realB;\n" +
                        "                }\n" +
                        valLine +
                        "            }\n" +
                        "        }\n"+
                        "    }\n";
        stream.print(foo);
    }

    private String makeBoundsCheck(boolean tranA, boolean tranB, String auxLength)
    {
        String a_numCols = tranA ? "a.numRows" : "a.numCols";
        String a_numRows = tranA ? "a.numCols" : "a.numRows";
        String b_numCols = tranB ? "b.numRows" : "b.numCols";
        String b_numRows = tranB ? "b.numCols" : "b.numRows";

        String ret =
                "        if( a == c || b == c )\n" +
                        "            throw new IllegalArgumentException(\"Neither 'a' or 'b' can be the same matrix as 'c'\");\n"+
                        "        else if( "+a_numCols+" != "+b_numRows+" ) {\n" +
                        "            throw new MatrixDimensionException(\"The 'a' and 'b' matrices do not have compatible dimensions\");\n" +
                        "        } else if( "+a_numRows+" != c.numRows || "+b_numCols+" != c.numCols ) {\n" +
                        "            throw new MatrixDimensionException(\"The results matrix does not have the desired dimensions\");\n" +
                        "        }\n" +
                        "\n";

        if( auxLength != null ) {
            ret += "        if( aux == null ) aux = new double[ "+auxLength+"*2 ];\n\n";
        }

        return ret;
    }

    private String handleZeros( boolean add ) {

        String fill = add ? "" : "            CommonOps_ZDRM.fill(c,0,0);\n";

        String ret =
                "        if( a.numCols == 0 || a.numRows == 0 ) {\n" +
                        fill +
                        "            return;\n" +
                        "        }\n";
        return ret;
    }

    private String makeHeader(String nameOp, String variant,
                              boolean add, boolean hasAlpha, boolean hasAux,
                              boolean tranA, boolean tranB)
    {
        if( add ) nameOp += "Add";

        // make the op name
        if( tranA && tranB ) {
            nameOp += "TransAB";
        } else if( tranA ) {
            nameOp += "TransA";
        } else if( tranB ) {
            nameOp += "TransB";
        }

        String ret = "    public static void "+nameOp;

        if( variant != null ) ret += "_"+variant+"( ";
        else ret += "( ";

        if( hasAlpha ) ret += "double realAlpha , double imagAlpha , ";

        if( hasAux ) {
            ret += "ZMatrixRMaj a , ZMatrixRMaj b , ZMatrixRMaj c , double []aux )\n";
        } else {
            ret += "ZMatrixRMaj a , ZMatrixRMaj b , ZMatrixRMaj c )\n";
        }

        ret += "    {\n";

        return ret;
    }

    public static void main(String[] args) throws FileNotFoundException {
        GeneratorCMatrixMatrixMult gen = new GeneratorCMatrixMatrixMult("MatrixMatrixMult_ZDRM.java");

        gen.createClass();
    }
}
