// Copyright 2026 OpenAI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <string>

#include <gtest/gtest.h>
#include <mujoco/mujoco.h>

#include "mujoco_ros2_control_plugins/obstacle_management.hpp"

namespace mujoco_ros2_control_plugins
{
namespace
{

class ObstacleManagementTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    constexpr char xml[] = R"(<mujoco>
      <worldbody>
        <geom name="floor" type="plane" size="10 10 0.1"/>
        <geom name="obstacle" type="box" size="0.4 0.5 0.6" pos="1 2 0.6"/>
      </worldbody>
    </mujoco>)";
    char error[1024] = {};
    spec = mj_parseXMLString(xml, nullptr, error, sizeof(error));
    ASSERT_NE(spec, nullptr) << error;
    model = mj_compile(spec, nullptr);
    ASSERT_NE(model, nullptr) << mjs_getError(spec);
    data = mj_makeData(model);
    ASSERT_NE(data, nullptr);
  }

  void TearDown() override
  {
    if (data)
    {
      mj_deleteData(data);
    }
    if (model)
    {
      mj_deleteModel(model);
    }
    if (spec)
    {
      mj_deleteSpec(spec);
    }
  }

  mjSpec* spec{ nullptr };
  mjModel* model{ nullptr };
  mjData* data{ nullptr };
};

TEST_F(ObstacleManagementTest, RecompilesFullDimensionsAndKeepsBoxOnFloor)
{
  ObstacleManagement management(spec, model, data);
  std::string error;

  ASSERT_TRUE(management.set_obstacle(-1.5, 3.0, 2.0, 4.0, 1.2, error)) << error;

  const ObstacleState state = management.state();
  EXPECT_DOUBLE_EQ(state.x, -1.5);
  EXPECT_DOUBLE_EQ(state.y, 3.0);
  EXPECT_DOUBLE_EQ(state.width, 2.0);
  EXPECT_DOUBLE_EQ(state.length, 4.0);
  EXPECT_DOUBLE_EQ(state.height, 1.2);
  EXPECT_DOUBLE_EQ(state.z, state.height / 2.0);
}

TEST_F(ObstacleManagementTest, RejectsNonPositiveDimensionsWithoutChangingModel)
{
  ObstacleManagement management(spec, model, data);
  const ObstacleState before = management.state();
  std::string error;

  EXPECT_FALSE(management.set_obstacle(5.0, 6.0, 0.0, 1.0, 1.0, error));

  const ObstacleState after = management.state();
  EXPECT_DOUBLE_EQ(after.x, before.x);
  EXPECT_DOUBLE_EQ(after.width, before.width);
  EXPECT_FALSE(error.empty());
}

}  // namespace
}  // namespace mujoco_ros2_control_plugins
