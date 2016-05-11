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

#include <cmath>

#include "ignition/math/Helpers.hh"
#include "ignition/math/Angle.hh"

using namespace ignition;

/////////////////////////////////////////////////
TEST(AngleTest, Angle)
{
  math::Angle angle1;
  EXPECT_TRUE(math::equal(0.0, angle1.Radian()));

  angle1.Degree(180.0);
  EXPECT_TRUE(angle1 == IGN_PI);

  EXPECT_FALSE(angle1 == IGN_PI + 0.1);
  EXPECT_TRUE(angle1 == IGN_PI + 0.0001);
  EXPECT_TRUE(angle1 == IGN_PI - 0.0001);
  EXPECT_TRUE(math::Angle(0) == math::Angle(0));
  EXPECT_TRUE(math::Angle(0) == math::Angle(0.001));

  angle1 = math::Angle(0.1) - math::Angle(0.3);
  EXPECT_TRUE(angle1 == -0.2);

  math::Angle angle(0.5);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle.Radian(IGN_PI);
  EXPECT_TRUE(math::equal(IGN_RTOD(IGN_PI), angle.Degree()));

  angle.Normalize();
  EXPECT_TRUE(math::equal(IGN_RTOD(IGN_PI), angle.Degree()));

  angle = math::Angle(0.1) + math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle = math::Angle(0.1) * math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.02, angle.Radian()));

  angle = math::Angle(0.1) / math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.5, angle.Radian()));

  angle -= math::Angle(0.1);
  EXPECT_TRUE(math::equal(0.4, angle.Radian()));

  angle += math::Angle(0.2);
  EXPECT_TRUE(math::equal(0.6, angle.Radian()));

  angle *= math::Angle(0.5);
  EXPECT_TRUE(math::equal(0.3, angle.Radian()));

  angle /= math::Angle(0.1);
  EXPECT_TRUE(math::equal(3.0, angle.Radian()));
  EXPECT_TRUE(angle == math::Angle(3));
  EXPECT_TRUE(angle != math::Angle(2));
  EXPECT_TRUE(angle < math::Angle(4));
  EXPECT_TRUE(angle > math::Angle(2));
  EXPECT_TRUE(angle >= math::Angle(3));
  EXPECT_TRUE(angle <= math::Angle(3));

  angle = 1.2;
  EXPECT_TRUE(angle <= 1.21);
  EXPECT_TRUE(angle <= 1.21_ign_rad);
  EXPECT_FALSE(angle <= 1.19);
  EXPECT_FALSE(angle <= 1.19_ign_rad);
  EXPECT_TRUE(angle <= 1.2);
  EXPECT_TRUE(angle <= 1.2_ign_rad);
  EXPECT_FALSE(angle <= -1.19);
  EXPECT_FALSE(angle <= -1.19_ign_rad);

  EXPECT_TRUE(1.2_ign_rad <= 1.2000000001_ign_rad);
  EXPECT_TRUE(1.2000000001_ign_rad <= 1.2_ign_rad);

  angle = 1.2;
  EXPECT_FALSE(angle >= 1.21);
  EXPECT_FALSE(angle >= 1.21_ign_rad);
  EXPECT_TRUE(angle >= 1.19);
  EXPECT_TRUE(angle >= 1.19_ign_rad);
  EXPECT_TRUE(angle >= 1.2);
  EXPECT_TRUE(angle >= 1.2_ign_rad);
  EXPECT_TRUE(angle >= -1.19);
  EXPECT_TRUE(angle >= -1.19_ign_rad);

  EXPECT_TRUE(math::Angle(1.2) >= math::Angle(1.2000000001));
  EXPECT_TRUE(1.2_ign_rad >= 1.2000000001_ign_rad);
  EXPECT_TRUE(math::Angle(1.2000000001) >= math::Angle(1.2));
  EXPECT_TRUE(1.2000000001_ign_rad >= 1.2_ign_rad);
}

/////////////////////////////////////////////////
TEST(AngleTest, StreamExtraction)
{
  math::Angle angle;
  std::istringstream stream("1.25");

  EXPECT_NEAR(*angle, 0, 1e-2);

  stream >> angle;
  EXPECT_NEAR(*angle, 1.25, 1e-2);
}

/////////////////////////////////////////////////
TEST(AngleTest, OperatorStreamOut)
{
  math::Angle a(0.1);
  std::ostringstream stream;
  stream << a;
  EXPECT_EQ(stream.str(), "0.1");
}

/////////////////////////////////////////////////
TEST(AngleTest, MoreOperators)
{
  math::Angle angle(1.23);
  math::Angle angle2(angle);
  EXPECT_EQ(+angle, angle2);
  EXPECT_NE(-angle, angle2);
  EXPECT_EQ(-angle, -angle2);
  EXPECT_NE(-angle, +angle2);

  EXPECT_EQ(angle - 1.0, 1.23_ign_rad - 1.0_ign_rad);
  EXPECT_EQ(angle + 1.0, 2.23_ign_rad);
  EXPECT_EQ(angle * 2.0, 1.23_ign_rad * 2.0_ign_rad);
  EXPECT_EQ(angle / 2.0, 1.23_ign_rad / 2.0_ign_rad);

  angle2 = angle;
  angle2 -= 2.3;
  EXPECT_EQ(angle - 2.3, angle2);

  angle2 = angle;
  angle2 += 1.2;
  EXPECT_EQ(angle + 1.2, angle2);

  angle2 = angle;
  angle2 *= 5.1;
  EXPECT_EQ(angle * 5.1, angle2);

  angle2 = angle;
  angle2 /= 2.2;
  EXPECT_EQ(angle / 2.2, angle2);

  EXPECT_TRUE(angle != 0.0);
  EXPECT_TRUE(angle != 1.0);
  EXPECT_TRUE(angle != angle2);

  angle2 = 1.23;
  EXPECT_TRUE(angle == 1.23);
  EXPECT_TRUE(angle <= 1.23);
  EXPECT_TRUE(angle < 1.24);
  EXPECT_TRUE(angle >= 1.23);
  EXPECT_TRUE(angle > 1.22);

  EXPECT_TRUE(angle() == 1.23);
  EXPECT_TRUE(angle() <= 1.23);
  EXPECT_TRUE(angle() < 1.24);
  EXPECT_TRUE(angle() >= 1.23);
  EXPECT_TRUE(angle() >= 1.22);

  EXPECT_TRUE(45.1_ign_deg == IGN_DTOR(45.1));
}
