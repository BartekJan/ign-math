/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
    EXPECT_DOUBLE_EQ(m.Mass(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXX(), 0.0);
    EXPECT_DOUBLE_EQ(m.IYY(), 0.0);
    EXPECT_DOUBLE_EQ(m.IZZ(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXY(), 0.0);
    EXPECT_DOUBLE_EQ(m.IXZ(), 0.0);
    EXPECT_DOUBLE_EQ(m.IYZ(), 0.0);
    EXPECT_EQ(m.DiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.OffDiagonalMoments(), math::Vector3d::Zero);
    EXPECT_EQ(m.MOI(), math::Matrix3d::Zero);
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Constructor with default arguments
  // Should match simple constructor and with copy constructor
  {
    math::MassMatrix3d m(0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_EQ(m, math::MassMatrix3d());
    EXPECT_EQ(m, math::MassMatrix3d(m));
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
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
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
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
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  const math::Matrix3d MOI(2.0, 0.2, 0.3,
                           0.2, 3.0, 0.4,
                           0.3, 0.4, 4.0);

  // Scalar setters with simple constructor
  // MassMatrix3 won't be valid until enough properties are set
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));
    EXPECT_FALSE(m.IXX(Ixxyyzz[0]));
    EXPECT_FALSE(m.IYY(Ixxyyzz[1]));

    // Valid once enough properties are set
    EXPECT_TRUE(m.IZZ(Ixxyyzz[2]));
    EXPECT_TRUE(m.IXY(Ixyxzyz[0]));
    EXPECT_TRUE(m.IXZ(Ixyxzyz[1]));
    EXPECT_TRUE(m.IYZ(Ixyxzyz[2]));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid again if an invalid inertia is set
    EXPECT_FALSE(m.Mass(-1));
  }

  // Test vector setters for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.DiagonalMoments(Ixxyyzz));
    EXPECT_TRUE(m.OffDiagonalMoments(Ixyxzyz));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an invalid inertia is set
    EXPECT_FALSE(m.IXX(-1));
  }

  // Test Matrix3 setter for moment of inertia
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.MOI(MOI));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());

    // Invalid if an excessive off-diagonal inertia is set
    EXPECT_FALSE(m.IXY(1e3));
  }

  // // Test atomic InertiaMatrix setter
  {
    math::MassMatrix3d m;
    EXPECT_FALSE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());

    // Initially invalid
    EXPECT_FALSE(m.Mass(mass));

    // Valid once enough properties are set
    EXPECT_TRUE(m.InertiaMatrix(2, 3, 4, 0.2, 0.3, 0.4));

    // Verify values
    EXPECT_DOUBLE_EQ(m.Mass(), mass);
    EXPECT_DOUBLE_EQ(m.IXX(), Ixxyyzz[0]);
    EXPECT_DOUBLE_EQ(m.IYY(), Ixxyyzz[1]);
    EXPECT_DOUBLE_EQ(m.IZZ(), Ixxyyzz[2]);
    EXPECT_DOUBLE_EQ(m.IXY(), Ixyxzyz[0]);
    EXPECT_DOUBLE_EQ(m.IXZ(), Ixyxzyz[1]);
    EXPECT_DOUBLE_EQ(m.IYZ(), Ixyxzyz[2]);
    EXPECT_EQ(m.DiagonalMoments(), Ixxyyzz);
    EXPECT_EQ(m.OffDiagonalMoments(), Ixyxzyz);
    EXPECT_EQ(m.MOI(), MOI);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, CoverageExtra)
{
  // getting full destructor coverage
  math::MassMatrix3d *p = new math::MassMatrix3d;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalMoments)
{
  // Diagonal inertia moments (1, 1, 1)
  {
    math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);

    // Minor perturbations of product moments
    // shouldn't affect PrincipalMoments, given the tolerance
    // of the Vector3 equality operator
    EXPECT_TRUE(m.IXY(1e-10));
    EXPECT_TRUE(m.IXZ(2e-10));
    EXPECT_TRUE(m.IYZ(3e-10));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d::One);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }

  // Non-equal eigen-moments
  {
    const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(Ixxyyzz));
    EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);

    // Minor perturbation of product moments
    EXPECT_TRUE(m.IXY(1e-10));
    EXPECT_TRUE(m.IXZ(2e-10));
    EXPECT_TRUE(m.IYZ(3e-10));
    EXPECT_EQ(m.PrincipalMoments(), Ixxyyzz);
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
    const math::Vector3d Ixxyyzz(2.0, 2.0, 2.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(2-M_SQRT2, 2, 2+M_SQRT2);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_FALSE(m.IsValid());
  }

  // Non-trivial off-diagonal product moments
  // variant of previous example that is valid inertia matrix
  {
    const math::Vector3d Ixxyyzz(4.0, 4.0, 4.0);
    const math::Vector3d Ixyxzyz(-1.0, 0, -1.0);
    math::MassMatrix3d m(1.0, Ixxyyzz, Ixyxzyz);
    const math::Vector3d Ieigen(4-M_SQRT2, 4, 4+M_SQRT2);
    EXPECT_EQ(m.PrincipalMoments(), Ieigen);
    EXPECT_TRUE(m.IsPositive());
    EXPECT_TRUE(m.IsValid());
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetIdentity)
{
  // Identity inertia matrix, expect unit quaternion
  math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond());

  // Scale the diagonal terms
  EXPECT_TRUE(m.DiagonalMoments(3.5 * math::Vector3d::One));
  EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d::Zero));
  EXPECT_TRUE(m.IsValid());
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond());
}

/////////////////////////////////////////////////
/// \brief Helper function for verifying principal moments
/// and axes offset by reconstructing the moment of inertia matrix
/// from the eigenvectors and diagonalized matrix.
/// \param[in] _m mass matrix to verify
void VerifyPrincipalMomentsAndAxes(const math::MassMatrix3d &_m)
{
  auto q = _m.PrincipalAxesOffset();
  auto R = math::Matrix3d(q);
  auto moments = _m.PrincipalMoments();
  math::Matrix3d L(moments[0], 0, 0,
                   0, moments[1], 0,
                   0, 0, moments[2]);
  EXPECT_EQ(_m.MOI(), R * L * R.Transposed());
}

/////////////////////////////////////////////////
/// \brief Helper function for testing diagonal inertia matrices.
/// Expect the following:
/// * that principal moments match the diagonal values,
/// * that mass matrix is valid,
/// * that principal axes have no offset (identity quaternion)
/// * that reconstructed moment of inertia matrix matches the original
/// \param[in] _moments Diagonal/principal moments of inertia.
void VerifyDiagonalMomentsAndAxes(const math::Vector3d &_moments)
{
  math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
  EXPECT_TRUE(m.DiagonalMoments(_moments));
  EXPECT_EQ(m.PrincipalMoments(), m.DiagonalMoments());
  EXPECT_TRUE(m.IsValid());
  // Expect unit quaternion
  EXPECT_EQ(m.PrincipalAxesOffset(), math::Quaterniond());
  VerifyPrincipalMomentsAndAxes(m);
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetDiagonal)
{
  // repeated moments [2, 3, 3]
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 3.0, 2.0));
  // repeated moments [2, 2, 3]
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 2.0, 3.0));
  // non-repeated moments
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 3.0, 4.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(4.0, 2.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 4.0, 2.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(2.0, 4.0, 3.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(3.0, 2.0, 4.0));
  VerifyDiagonalMomentsAndAxes(math::Vector3d(4.0, 3.0, 2.0));
}

/////////////////////////////////////////////////
/// \brief Helper function for testing non-diagonal inertia matrices.
/// Expect the following:
/// * that principal moments match the supplied values,
/// * that mass matrix is valid,
/// * that principal axes have an offset (non-identity quaternion)
/// * that reconstructed moment of inertia matrix matches the original
/// \param[in] _principalMoments Expected principal moments of inertia
/// \param[in] _Ixxyyzz Diagonal moments of inertia.
/// \param[in] _Ixyxzyz Off-diagonal moments of inertia.
void VerifyNondiagonalMomentsAndAxes(const math::Vector3d &_principalMoments,
                                     const math::Vector3d &_Ixxyyzz,
                                     const math::Vector3d &_Ixyxzyz,
                                     const double _tolerance=1e-6)
{
  math::MassMatrix3d m(1.0, _Ixxyyzz, _Ixyxzyz);
  // EXPECT_EQ with default tolerance of 1e-6
  // this outputs more useful error messages
  EXPECT_EQ(m.PrincipalMoments(), _principalMoments);
  // also check equality with custom tolerance for small moments
  EXPECT_TRUE(m.PrincipalMoments().Equal(_principalMoments, _tolerance));
  EXPECT_TRUE(m.IsValid());
  // Expect non-unit quaternion
  EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
  VerifyPrincipalMomentsAndAxes(m);
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetRepeat)
{
  // Non-zero Ixy
  VerifyNondiagonalMomentsAndAxes(math::Vector3d(3, 3, 5),
    math::Vector3d(4, 4, 3), math::Vector3d(-1, 0, 0));
    
  // Rotated by [45, 0, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4, 4, 3)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1, 0, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 3, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero Ixz
  // Principal moments [3, 3, 5]
  // Rotated by [45, 0, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4, 3, 4)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, -1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 3, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero Iyz
  // Principal moments [3, 3, 5]
  // Rotated by [45, 0, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(3, 4, 4)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, 0, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 3, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero Ixy
  // Principal moments [3, 5, 5]
  // Rotated by [45, 0, 0] degrees
  // 3456
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4, 4, 5)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1, 0, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero Ixz
  // Principal moments [3, 5, 5]
  // Rotated by [45, 0, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4, 5, 4)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, -1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero Iyz
  // Principal moments [3, 5, 5]
  // Rotated by [45, 0, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5, 4, 4)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, 0, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 5, 5]
  // Rotated by [45, 45, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.75, 4.75)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-M_SQRT2, M_SQRT2, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 5, 5]
  // Rotated by [-45, 45, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.75, 4.75)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(M_SQRT2, M_SQRT2, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 5, 5]
  // Rotated by [45, -45, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.75, 4.75)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(M_SQRT2, -M_SQRT2, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 5, 5]
  // Rotated by [-45, -45, 0] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.75, 4.75)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-M_SQRT2, -M_SQRT2, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 5, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 5]
  // Rotated by [45, 45, 45] degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.25, 4.25)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-M_SQRT2, M_SQRT2, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 5]
  // different rotation
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.25, 4.25)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(M_SQRT2, M_SQRT2, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 5]
  // different rotation
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.25, 4.25)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-M_SQRT2, -M_SQRT2, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 5]
  // different rotation
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.5, 4.25, 4.25)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(M_SQRT2, -M_SQRT2, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 5));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms, small magnitude
  // Principal moments [4e-9, 4e-9, 5e-9]
  // different rotation
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(1e-9*math::Vector3d(4.5, 4.25, 4.25)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25e-9*math::Vector3d(M_SQRT2, -M_SQRT2, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4e-9, 4e-9, 5e-9));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 6]
  // rotate by 30, 60, 0 degrees
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5.5, 4.125, 4.375)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-sqrt(3), 3.0, -sqrt(3)/2)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-zero off-diagonal terms
  // Principal moments [4, 4, 6]
  // different rotation
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.125, 5.5, 4.375)));
    EXPECT_TRUE(m.OffDiagonalMoments(
      0.25*math::Vector3d(-sqrt(3), -sqrt(3)/2, 3.0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, PrincipalAxesOffsetNoRepeat)
{
  // These inertia matrices do not have repeated values

  // Non-diagonal inertia matrix with f1 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(3.0, 5.0, 5.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, 0, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-diagonal inertia matrix with f1 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(3.0, 5.0, 5.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, 0, -1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-diagonal inertia matrix with f2 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5.0, 4.0, 4.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1, 1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-diagonal inertia matrix with f2 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5.0, 4.0, 4.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(1, -1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-diagonal inertia matrix with f2 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5.0, 4.0, 4.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1, -1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Non-diagonal inertia matrix with f2 = 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(5.0, 4.0, 4.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(1, 1, 0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Similar non-diagonal inertia matrix with f2 != 0
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.0, 4.0, 5.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(0, 1, 1)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(3, 4, 6));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Nontrivial inertia matrix, expect non-unit quaternion
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.0, 4.0, 4.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1.0, 0, -1.0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(4-M_SQRT2, 4, 4+M_SQRT2));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Nontrivial inertia matrix, expect non-unit quaternion
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.0, 5.0, 6.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1.0, 0, -1.0)));
    EXPECT_EQ(m.PrincipalMoments(), math::Vector3d(5-sqrt(3), 5, 5+sqrt(3)));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Nontrivial inertia matrix, expect non-unit quaternion
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(math::Vector3d(4.0, 5.0, 6.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(math::Vector3d(-1.0, 0.5, -1.0)));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }

  // Nontrivial inertia matrix, expect non-unit quaternion, small magnitude
  {
    math::MassMatrix3d m(1.0, math::Vector3d::Zero, math::Vector3d::Zero);
    EXPECT_TRUE(m.DiagonalMoments(1e-9*math::Vector3d(4.0, 5.0, 6.0)));
    EXPECT_TRUE(m.OffDiagonalMoments(1e-9*math::Vector3d(-1.0, 0.5, -1.0)));
    EXPECT_TRUE(m.IsValid());
    EXPECT_NE(m.PrincipalAxesOffset(), math::Quaterniond());
    VerifyPrincipalMomentsAndAxes(m);
  }
}

/////////////////////////////////////////////////
TEST(MassMatrix3dTest, EquivalentBox)
{
  // Identity inertia matrix
  {
    math::MassMatrix3d m(1.0, math::Vector3d::One, math::Vector3d::Zero);
    math::Vector3d size;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size, rot));
    EXPECT_EQ(size, sqrt(6) * math::Vector3d::One);
    EXPECT_EQ(rot, math::Quaterniond::Identity);
  }

  // unit box with mass 1.0
  {
    double mass = 1.0;
    const math::Vector3d size(1, 1, 1);
    double Ixx = mass/12 * (std::pow(size.Y(), 2) + std::pow(size.Z(), 2));
    double Iyy = mass/12 * (std::pow(size.Z(), 2) + std::pow(size.X(), 2));
    double Izz = mass/12 * (std::pow(size.X(), 2) + std::pow(size.Y(), 2));
    math::Vector3d Ixxyyzz(Ixx, Iyy, Izz);
    math::MassMatrix3d m(mass, Ixxyyzz, math::Vector3d::Zero);
    math::Vector3d size2;
    math::Quaterniond rot;
    EXPECT_TRUE(m.EquivalentBox(size2, rot));
    EXPECT_EQ(size, size2);
    EXPECT_EQ(rot, math::Quaterniond::Identity);
  }
}
