/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Matrix3d.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(Matrix3dTest, Matrix3d)
{
  {
    math::Matrix3d matrix;
    EXPECT_TRUE(matrix == math::Matrix3d(0, 0, 0, 0, 0, 0, 0, 0, 0));
  }

  {
    math::Matrix3d matrix(1, 2, 3, 4, 5, 6, 7, 8, 9);
    EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));

    math::Matrix3d matrix1(matrix);
    EXPECT_TRUE(matrix1 == math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));
  }

  math::Matrix3d matrix;
  matrix.SetFromAxes(math::Vector3d(1, 1, 1), math::Vector3d(2, 2, 2),
                     math::Vector3d(3, 3, 3));
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 3, 1, 2, 3, 1, 2, 3));

  matrix.SetFromAxis(math::Vector3d(1, 1, 1), M_PI);
  EXPECT_TRUE(matrix == math::Matrix3d(1, 2, 2, 2, 1, 2, 2, 2, 1));

  matrix.SetCol(0, math::Vector3d(3, 4, 5));
  EXPECT_TRUE(matrix == math::Matrix3d(3, 2, 2, 4, 1, 2, 5, 2, 1));

  EXPECT_THROW(matrix.SetCol(3, math::Vector3d(1, 1, 1)),
      ignition::math::IndexException);
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, IndexException)
{
  math::Matrix3d mat = math::Matrix3d::Zero;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      EXPECT_NO_THROW(mat(i, j));

  EXPECT_THROW(math::equal(mat(3, 0), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(mat(0, 3), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(mat(3, 3), 0.0), math::IndexException);

  EXPECT_THROW(mat(3, 0) = 0, math::IndexException);
  EXPECT_THROW(mat(0, 3) = 0, math::IndexException);
  EXPECT_THROW(mat(3, 3) = 0, math::IndexException);

  const math::Matrix3d constMat(math::Matrix3d::Zero);

  EXPECT_THROW(math::equal(constMat(3, 0), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(constMat(0, 3), 0.0), math::IndexException);
  EXPECT_THROW(math::equal(constMat(3, 3), 0.0), math::IndexException);
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorSubtract)
{
  math::Matrix3d matZero = math::Matrix3d::Zero;
  math::Matrix3d matIdent = math::Matrix3d::Identity;

  math::Matrix3d mat = matIdent - matZero;
  EXPECT_EQ(mat, matIdent);

  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  math::Matrix3d matB(10, 20, 30,
                      40, 50, 60,
                      70, 80, 90);

  mat = matB - matA;
  EXPECT_EQ(mat, math::Matrix3d(9, 18, 27, 36, 45, 54, 63, 72, 81));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorAdd)
{
  math::Matrix3d matZero = math::Matrix3d::Zero;
  math::Matrix3d matIdent = math::Matrix3d::Identity;

  math::Matrix3d mat = matIdent + matZero;
  EXPECT_EQ(mat, matIdent);

  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  math::Matrix3d matB(10, 20, 30,
                      40, 50, 60,
                      70, 80, 90);

  mat = matB + matA;
  EXPECT_EQ(mat, math::Matrix3d(11, 22, 33, 44, 55, 66, 77, 88, 99));
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorMul)
{
  {
    // Multiply arbitrary matrix by zeros of different sizes
    math::Matrix3d matrix(0.1, 0.2, 0.3,
                          0.4, 0.5, 0.6,
                          0.7, 0.8, 0.9);

    // Scalar 0
    EXPECT_EQ(math::Matrix3d::Zero, matrix * 0.0);
    EXPECT_EQ(math::Matrix3d::Zero, 0.0 * matrix);

    // Vector3d::Zero
    EXPECT_EQ(math::Vector3d::Zero, matrix * math::Vector3d::Zero);
    // left multiply with Vector3 not implemented

    // Matrix3d::Zero
    EXPECT_EQ(math::Matrix3d::Zero, matrix * math::Matrix3d::Zero);
    EXPECT_EQ(math::Matrix3d::Zero, math::Matrix3d::Zero * matrix);
  }

  {
    // Multiply arbitrary matrix by identity values
    math::Matrix3d matrix(0.1, 0.2, 0.3,
                          0.4, 0.5, 0.6,
                          0.7, 0.8, 0.9);

    // scalar 1.0
    EXPECT_EQ(matrix, matrix * 1.0);
    EXPECT_EQ(matrix, 1.0 * matrix);

    // Vector3d::Unit[X|Y|Z]
    EXPECT_EQ(math::Vector3d(matrix(0, 0), matrix(1, 0), matrix(2, 0)),
              matrix * math::Vector3d::UnitX);
    EXPECT_EQ(math::Vector3d(matrix(0, 1), matrix(1, 1), matrix(2, 1)),
              matrix * math::Vector3d::UnitY);
    EXPECT_EQ(math::Vector3d(matrix(0, 2), matrix(1, 2), matrix(2, 2)),
              matrix * math::Vector3d::UnitZ);

    // Matrix3d::Identity
    EXPECT_EQ(matrix, matrix * math::Matrix3d::Identity);
    EXPECT_EQ(matrix, math::Matrix3d::Identity * matrix);
  }

  {
    // Multiply with non-zero and non-identity values
    math::Matrix3d matrix(0.1, 0.2, 0.3,
                          0.4, 0.5, 0.6,
                          0.7, 0.8, 0.9);

    // Scalar 2.5
    EXPECT_EQ(matrix * 2.5, math::Matrix3d(0.25, 0.50, 0.75,
                                           1.00, 1.25, 1.50,
                                           1.75, 2.00, 2.25));
    EXPECT_EQ(2.5 * matrix, math::Matrix3d(0.25, 0.50, 0.75,
                                           1.00, 1.25, 1.50,
                                           1.75, 2.00, 2.25));

    // Vector {9.4, -3, 0.3}
    EXPECT_EQ(math::Vector3d(0.43, 2.44, 4.45),
              matrix * math::Vector3d(9.4, -3.0, 0.3));

    // Multiply with itself
    EXPECT_EQ(matrix*matrix, math::Matrix3d(0.30, 0.36, 0.42,
                                            0.66, 0.81, 0.96,
                                            1.02, 1.26, 1.50));

    // Multiply with other matrix
    math::Matrix3d matrix2(9, -6, 3,
                           8, -5, 2,
                           7, -4, 1);
    EXPECT_EQ(matrix*matrix2, math::Matrix3d(4.6,  -2.8, 1.0,
                                            11.8,  -7.3, 2.8,
                                            19.0, -11.8, 4.6));
    EXPECT_EQ(matrix2*matrix, math::Matrix3d(0.6, 1.2, 1.8,
                                             0.2, 0.7, 1.2,
                                            -0.2, 0.2, 0.6));
  }
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorStreamOut)
{
  math::Matrix3d matA(1, 2, 3,
                      4, 5, 6,
                      7, 8, 9);

  std::ostringstream stream;
  stream << matA;
  EXPECT_EQ(stream.str(), "1 2 3 4 5 6 7 8 9");
}

/////////////////////////////////////////////////
TEST(Matrix3dTest, OperatorStreamIn)
{
  math::Matrix3d mat;
  EXPECT_EQ(mat, math::Matrix3d::Zero);

  std::istringstream stream("1 2 3 4 5 6 7 8 9");
  stream >> mat;
  EXPECT_EQ(mat, math::Matrix3d(1, 2, 3, 4, 5, 6, 7, 8, 9));
}
