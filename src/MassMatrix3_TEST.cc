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

#define _USE_MATH_DEFINES
#include <gtest/gtest.h>

#include "ignition/math/Helpers.hh"
#include "ignition/math/MassMatrix3.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, Constructors)
{
  // Simple constructor, test default values
  {
    math::MassMatrix3d m;
    EXPECT_DOUBLE_EQ(m.Mass(), 1.0);
    EXPECT_DOUBLE_EQ(m.IXX(), 1.0);
    EXPECT_DOUBLE_EQ(m.IYY(), 1.0);
    EXPECT_DOUBLE_EQ(m.IZZ(), 1.0);
    EXPECT_DOUBLE_EQ(m.IXY(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXZ(), 0.0);
    EXPECT_DOUBLE_EQ(m.IYZ(), 0.0);
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
    EXPECT_EQ(m.ProductsofInertia(), math::Vector3d::Zero);
    EXPECT_EQ(m.MOI(), math::Matrix3d::Identity);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Constructor with default arguments
  // Should match simple constructor and with copy constructor
  {
    math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
    EXPECT_EQ(m, math::MassMatrix3d());
    EXPECT_EQ(m, math::MassMatrix3d(m));
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Constructor with non-default arguments
  {
    const double mass = 5.0;
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
    const math::Matrix3d MOI(2.0, 0.2, 0.3,
                             0.2, 3.0, 0.4,
                             0.3, 0.4, 4.0);
    math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);

    // Should not match simple constructor
    EXPECT_NE(m, math::MassMatrix3d());

    // Should match with copy constructor
    EXPECT_EQ(m, math::MassMatrix3d(m));

    // Test accessors
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);
    EXPECT_EQ(m.ProductsofInertia(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Test assignment operator
    math::MassMatrix3d m2;
    EXPECT_NE(m, m2);
    m2 = m;
    EXPECT_EQ(m, m2);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, Setters)
{
  // Simple constructor, test default values
  math::MassMatrix3d m;
  EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
  EXPECT_EQ(m.ProductsofInertia(), math::Vector3d::Zero);
  EXPECT_EQ(m.MOI(), math::Matrix3d::Identity);

  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  const math::Matrix3d MOI(2.0, 0.2, 0.3,
                           0.2, 3.0, 0.4,
                           0.3, 0.4, 4.0);

  // Test scalar setters
  EXPECT_TRUE(m.Mass(mass));
  EXPECT_TRUE(m.IXX(Ixxyyzz[0]));
  EXPECT_TRUE(m.IYY(Ixxyyzz[1]));
  EXPECT_TRUE(m.IZZ(Ixxyyzz[2]));
  EXPECT_TRUE(m.IXY(Ixyxzyz[0]));
  EXPECT_TRUE(m.IXZ(Ixyxzyz[1]));
  EXPECT_TRUE(m.IYZ(Ixyxzyz[2]));
  EXPECT_DOUBLE_EQ(m.Mass(), mass);
  EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
  EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
  EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
  EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
  EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
  EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
  EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);
  EXPECT_EQ(m.ProductsofInertia(), Ixyxzyz);
  EXPECT_EQ(m.MOI(), MOI);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  // Test vector setters for moment of inertia
  // reset to default values
  EXPECT_TRUE(m.PrincipalMoments(math::Vector3d::One));
  EXPECT_TRUE(m.ProductsofInertia(math::Vector3d::Zero));
  EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
  EXPECT_EQ(m.ProductsofInertia(), math::Vector3d::Zero);
  EXPECT_EQ(m.MOI(), math::Matrix3d::Identity);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  // Test Matrix3 setter for moment of inertia
  // set to specified values
  EXPECT_TRUE(m.MOI(MOI));
  EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);
  EXPECT_EQ(m.ProductsofInertia(), Ixyxzyz);
  EXPECT_EQ(m.MOI(), MOI);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());

  // Test atomic InertiaMatrix setter
  // reset to default values
  EXPECT_TRUE(m.InertiaMatrix(1, 1, 1, 0, 0, 0));
  EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
  EXPECT_EQ(m.ProductsofInertia(), math::Vector3d::Zero);
  EXPECT_EQ(m.MOI(), math::Matrix3d::Identity);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, EigenMoments)
{
  // Expect default inertia moments (1, 1, 1)
  {
    math::MassMatrix3d m;
    EXPECT_EQ(m.EigenMoments(), math::Vector3d::One);

    // Minor perturbations of product moments
    // shouldn't affect EigenMoments, given the tolerance
    // of the Vector3 equality operator
    EXPECT_TRUE(m.IXY(1e-10));
    EXPECT_TRUE(m.IXZ(2e-10));
    EXPECT_TRUE(m.IYZ(3e-10));
    EXPECT_EQ(m.EigenMoments(), math::Vector3d::One);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Non-equal eigen-moments
  {
    math::MassMatrix3d m;
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    EXPECT_TRUE(m.PrincipalMoments(Ixxyyzz));
    EXPECT_TRUE(m.ProductsofInertia(math::Vector3d::Zero));
    EXPECT_EQ(m.EigenMoments(), Ixxyyzz);

    // Minor perturbation of product moments
    EXPECT_TRUE(m.IXY(1e-10));
    EXPECT_TRUE(m.IXZ(2e-10));
    EXPECT_TRUE(m.IYZ(3e-10));
    EXPECT_EQ(m.EigenMoments(), Ixxyyzz);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Non-trivial off-diagonal product moments
  // Symmetric positive definite matrix from
  // Strang's Intro to Linear Algebra textbook
  // This isn't actually a valid inertia matrix though,
  // since it doesn't satisfy the triangle inequality
  // 2-sqrt(2) + 2 ~= 2.59
  // 2+sqrt(2) ~= 3.41
  {
    math::MassMatrix3d m;
    const math::Vector3d Ixxyyzz(2.0, 2.0, 2.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    const math::Vector3d Ieigen(2-M_SQRT2, 2, 2+M_SQRT2);
    EXPECT_TRUE(m.PrincipalMoments(Ixxyyzz));
    EXPECT_TRUE(m.ProductsofInertia(Ixyxzyz));
    EXPECT_EQ(m.EigenMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Non-trivial off-diagonal product moments
  // variant of previous example that is valid inertia matrix
  {
    math::MassMatrix3d m;
    const math::Vector3d Ixxyyzz(4.0, 4.0, 4.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    const math::Vector3d Ieigen(4-M_SQRT2, 4, 4+M_SQRT2);
    EXPECT_TRUE(m.PrincipalMoments(Ixxyyzz));
    EXPECT_TRUE(m.ProductsofInertia(Ixyxzyz));
    EXPECT_EQ(m.EigenMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }
}
