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

#include <gtest/gtest.h>
#include <cmath>

#include "ignition/math/Inertial.hh"

using namespace ignition;

/////////////////////////////////////////////////
// Simple constructor, test default values
TEST(Inertial_d_Test, Constructor)
{
  math::Inertial_d inertial;
  EXPECT_EQ(inertial.Pose(), math::Pose3d::Zero);
  EXPECT_EQ(inertial.MassMatrix(), math::MassMatrix3d());
}

/////////////////////////////////////////////////
// Constructor with default arguments
// Should match simple constructor and with copy constructor
TEST(Inertial_d_Test, ConstructorDefaultValues)
{
  math::Inertial_d inertial(math::MassMatrix3d(), math::Pose3d::Zero);
  EXPECT_EQ(inertial, math::Inertial_d());
  EXPECT_EQ(inertial, math::Inertial_d(inertial));
}

/////////////////////////////////////////////////
// Constructor with non-default arguments
TEST(Inertial_d_Test, ConstructorNonDefaultValues)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, IGN_PI/6, 0, 0);
  math::Inertial_d inertial(m, pose);

  // Should not match simple constructor
  EXPECT_NE(inertial, math::Inertial_d());

  // Should match with copy constructor
  EXPECT_EQ(inertial, math::Inertial_d(inertial));

  // Test accessors
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);
  EXPECT_TRUE(inertial.MassMatrix().IsPositive());
  EXPECT_TRUE(inertial.MassMatrix().IsValid());

  // Test assignment operator
  math::Inertial_d inertial2;
  EXPECT_NE(inertial, inertial2);
  inertial2 = inertial;
  EXPECT_EQ(inertial, inertial2);
}

/////////////////////////////////////////////////
TEST(Inertial_d_Test, CoverageExtra)
{
  // getting full destructor coverage
  math::Inertial_d *p = new math::Inertial_d;
  EXPECT_TRUE(p != NULL);
  delete p;
}

/////////////////////////////////////////////////
TEST(Inertial_d_Test, Setters)
{
  const double mass = 5.0;
  const math::Vector3d Ixxyyzz(2.0, 3.0, 4.0);
  const math::Vector3d Ixyxzyz(0.2, 0.3, 0.4);
  math::MassMatrix3d m(mass, Ixxyyzz, Ixyxzyz);
  EXPECT_TRUE(m.IsPositive());
  EXPECT_TRUE(m.IsValid());
  const math::Pose3d pose(1, 2, 3, IGN_PI/6, 0, 0);
  math::Inertial_d inertial;

  // Initially invalid
  EXPECT_FALSE(inertial.Pose(pose));

  // Valid once valid mass matrix is set
  EXPECT_TRUE(inertial.MassMatrix(m));

  // Verify values
  EXPECT_EQ(inertial.MassMatrix(), m);
  EXPECT_EQ(inertial.Pose(), pose);

  // Invalid again if an invalid inertia is set
  math::MassMatrix3d mInvalid(-1, Ixxyyzz, Ixyxzyz);
  EXPECT_FALSE(inertial.MassMatrix(mInvalid));
}
