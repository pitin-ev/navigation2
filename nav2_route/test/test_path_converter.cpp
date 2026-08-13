// Copyright (c) 2025, Open Navigation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_route/path_converter.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

using namespace nav2_route;  // NOLINT

TEST(PathConverterTest, test_path_converter_api)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  auto node_thread = std::make_unique<nav2_util::NodeThread>(node);

  nav_msgs::msg::Path path_msg;
  auto sub = node->create_subscription<nav_msgs::msg::Path>(
    "plan", rclcpp::QoS(10), [&, this](nav_msgs::msg::Path msg) {path_msg = msg;});

  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);
  Route route;
  Node test_node1, test_node2, test_node3;
  test_node1.nodeid = 10;
  test_node1.coords.x = 0.0;
  test_node1.coords.y = 0.0;
  test_node2.nodeid = 11;
  test_node2.coords.x = 10.0;
  test_node2.coords.y = 10.0;
  test_node3.nodeid = 12;
  test_node3.coords.x = 20.0;
  test_node3.coords.y = 20.0;

  DirectionalEdge test_edge1, test_edge2;
  test_edge1.edgeid = 13;
  test_edge1.start = &test_node1;
  test_edge1.end = &test_node2;
  test_edge2.edgeid = 14;
  test_edge2.start = &test_node2;
  test_edge2.end = &test_node3;

  route.start_node = &test_node1;
  route.route_cost = 50.0;
  route.edges.push_back(&test_edge1);
  route.edges.push_back(&test_edge2);
  ReroutingState info;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.header.frame_id, frame);
  EXPECT_EQ(path.header.stamp.nanosec, time.nanoseconds());

  // 2 * sqrt(200) * 20 (0.05 density/m) + 1 (for starting node)
  EXPECT_EQ(path.poses.size(), 567u);
  EXPECT_NEAR(path.poses[0].pose.position.x, 0.0, 0.01);
  EXPECT_NEAR(path.poses[0].pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(path.poses.back().pose.position.x, 20.0, 0.01);
  EXPECT_NEAR(path.poses.back().pose.position.y, 20.0, 0.01);

  rclcpp::Rate r(10);
  r.sleep();

  // Checks the same as returned and actually was published
  EXPECT_EQ(path_msg.poses.size(), path.poses.size());
  node_thread.reset();
}

TEST(PathConverterTest, test_path_single_pt_path)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 10.0;
  test_node.coords.y = 40.0;

  Route route;
  route.start_node = &test_node;
  ReroutingState info;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.poses.size(), 1u);
  EXPECT_NEAR(path.poses[0].pose.position.x, 10.0, 0.01);
  EXPECT_NEAR(path.poses[0].pose.position.y, 40.0, 0.01);
}

TEST(PathConverterTest, test_prev_info_path)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  std::string frame = "fake_frame";
  rclcpp::Time time(1000);

  Node test_node;
  test_node.nodeid = 17;
  test_node.coords.x = 1.0;
  test_node.coords.y = 0.0;

  Route route;
  route.start_node = &test_node;

  DirectionalEdge edge;
  edge.end = &test_node;

  ReroutingState info;
  info.closest_pt_on_edge.x = 0.0;
  info.closest_pt_on_edge.y = 0.0;
  info.curr_edge = &edge;

  auto path = converter.densify(route, info, frame, time);
  EXPECT_EQ(path.poses.size(), 21u);  // 20 for density + 1 for single node point
}

TEST(PathConverterTest, test_path_converter_interpolation)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  float x0 = 10.0, y0 = 10.0, x1 = 20.0, y1 = 20.0;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  converter.interpolateEdge(x0, y0, x1, y1, poses);

  EXPECT_EQ(poses.size(), 283u);  // regular density + edges
  for (unsigned int i = 0; i != poses.size() - 1; i++) {
    // Check its always closer than the requested density
    EXPECT_LT(
      hypotf(
        poses[i].pose.position.x - poses[i + 1].pose.position.x,
        poses[i].pose.position.y - poses[i + 1].pose.position.y), 0.05);
  }
}

TEST(PathConverterTest, test_path_converter_zero_length_edge)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("edge_scorer_test");
  PathConverter converter;
  converter.configure(node);

  float x0 = 10.0, y0 = 10.0, x1 = 10.0, y1 = 10.0;
  std::vector<geometry_msgs::msg::PoseStamped> poses;
  converter.interpolateEdge(x0, y0, x1, y1, poses);
  ASSERT_TRUE(poses.empty());
}

// ── Per-edge orientation from graph metadata ────────────────────────────────
//
// Stock nav2_route makes every path pose face the path tangent. For an
// omni-directional robot that throws away a degree of freedom it actually has:
// it can hold any heading while translating (crab / strafe).
//
// VDA5050 v3.0.0 (§Edge, p.73) already standardises this, so the edge metadata
// is read using the spec's own names:
//
//   orientation      [rad]  heading to hold on the edge
//   orientationType  GLOBAL (map-absolute, "only valid for omnidirectional
//                    robots") | TANGENTIAL (0 = forwards, PI = backwards).
//                    Default TANGENTIAL.
//
// "If no orientation is defined, the mobile robot may assume any orientation on
// the edge" — so an edge without metadata keeps the stock tangent and existing
// graphs are unaffected.

namespace
{

/// yaw of a pure-Z quaternion.
double yawOf(const geometry_msgs::msg::PoseStamped & p)
{
  return 2.0 * atan2(p.pose.orientation.z, p.pose.orientation.w);
}

/// Single edge from (0,0) to (10,0): tangent is 0 rad (+x).
struct StraightRoute
{
  Node n0, n1;
  DirectionalEdge edge;
  Route route;

  StraightRoute()
  {
    n0.nodeid = 1; n0.coords.x = 0.0; n0.coords.y = 0.0;
    n1.nodeid = 2; n1.coords.x = 10.0; n1.coords.y = 0.0;
    edge.edgeid = 100;
    edge.start = &n0;
    edge.end = &n1;
    route.start_node = &n0;
    route.edges.push_back(&edge);
  }
};

}  // namespace

TEST(PathConverterTest, test_no_metadata_keeps_tangent)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_none");
  PathConverter converter;
  converter.configure(node);

  StraightRoute r;
  ReroutingState info;
  auto path = converter.densify(r.route, info, "map", rclcpp::Time(0));

  ASSERT_GT(path.poses.size(), 2u);
  for (const auto & p : path.poses) {
    EXPECT_NEAR(yawOf(p), 0.0, 1e-6) << "an edge without metadata must keep the tangent";
  }
}

TEST(PathConverterTest, test_global_orientation_overrides_tangent)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_global");
  PathConverter converter;
  converter.configure(node);

  StraightRoute r;
  double orientation = M_PI / 2.0;          // face +y while travelling +x
  std::string type = "GLOBAL";
  r.edge.metadata.setValue("orientation", orientation);
  r.edge.metadata.setValue("orientationType", type);

  ReroutingState info;
  auto path = converter.densify(r.route, info, "map", rclcpp::Time(0));

  ASSERT_GT(path.poses.size(), 2u);
  for (const auto & p : path.poses) {
    EXPECT_NEAR(yawOf(p), M_PI / 2.0, 1e-6)
      << "GLOBAL orientation must be applied verbatim — this is what lets an "
         "omni robot strafe with a fixed heading";
  }
}

TEST(PathConverterTest, test_tangential_orientation_is_relative)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_tangential");
  PathConverter converter;
  converter.configure(node);

  StraightRoute r;
  double orientation = M_PI;                // spec: PI = drive backwards
  std::string type = "TANGENTIAL";
  r.edge.metadata.setValue("orientation", orientation);
  r.edge.metadata.setValue("orientationType", type);

  ReroutingState info;
  auto path = converter.densify(r.route, info, "map", rclcpp::Time(0));

  ASSERT_GT(path.poses.size(), 2u);
  // tangent 0 + PI, normalised to (-PI, PI].
  EXPECT_NEAR(fabs(yawOf(path.poses.front())), M_PI, 1e-6);
}

TEST(PathConverterTest, test_default_orientation_type_is_tangential)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_default_type");
  PathConverter converter;
  converter.configure(node);

  StraightRoute r;
  double orientation = 0.0;                 // orientationType deliberately absent
  r.edge.metadata.setValue("orientation", orientation);

  ReroutingState info;
  auto path = converter.densify(r.route, info, "map", rclcpp::Time(0));

  ASSERT_GT(path.poses.size(), 2u);
  EXPECT_NEAR(yawOf(path.poses.front()), 0.0, 1e-6)
    << "the VDA5050 default is TANGENTIAL, so 0 rad means 'along the edge'";
}

TEST(PathConverterTest, test_wrong_metadata_type_does_not_throw)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_badtype");
  PathConverter converter;
  converter.configure(node);

  StraightRoute r;
  // A graph author writing 0 instead of 0.0 stores an int. Metadata::getValue
  // is std::any_cast underneath and would throw — mid-plan, inside the route
  // server. A typo in a graph file must not be able to stop a 2 t robot.
  int wrong = 1;
  r.edge.metadata.setValue("orientation", wrong);

  ReroutingState info;
  nav_msgs::msg::Path path;
  EXPECT_NO_THROW(path = converter.densify(r.route, info, "map", rclcpp::Time(0)));

  ASSERT_GT(path.poses.size(), 2u);
  EXPECT_NEAR(yawOf(path.poses.front()), 0.0, 1e-6) << "must fall back to the tangent";
}

TEST(PathConverterTest, test_orientation_is_resolved_per_edge)
{
  auto node = std::make_shared<nav2_util::LifecycleNode>("po_per_edge");
  PathConverter converter;
  converter.configure(node);

  // (0,0) → (10,0) → (10,10): tangents 0 and PI/2.
  Node n0, n1, n2;
  n0.nodeid = 1; n0.coords.x = 0.0; n0.coords.y = 0.0;
  n1.nodeid = 2; n1.coords.x = 10.0; n1.coords.y = 0.0;
  n2.nodeid = 3; n2.coords.x = 10.0; n2.coords.y = 10.0;

  DirectionalEdge e0, e1;
  e0.edgeid = 10; e0.start = &n0; e0.end = &n1;
  e1.edgeid = 11; e1.start = &n1; e1.end = &n2;

  // First edge holds a fixed heading; second edge says nothing → tangent.
  double orientation = M_PI / 2.0;
  std::string type = "GLOBAL";
  e0.metadata.setValue("orientation", orientation);
  e0.metadata.setValue("orientationType", type);

  Route route;
  route.start_node = &n0;
  route.edges.push_back(&e0);
  route.edges.push_back(&e1);

  ReroutingState info;
  auto path = converter.densify(route, info, "map", rclcpp::Time(0));

  ASSERT_GT(path.poses.size(), 10u);
  // Early poses belong to e0 → the GLOBAL heading.
  EXPECT_NEAR(yawOf(path.poses[2]), M_PI / 2.0, 1e-6);
  // The final pose belongs to e1, which carries no metadata → its tangent,
  // which here happens to also be PI/2. Assert the *last* edge resolves on its
  // own rather than inheriting e0's entry.
  EXPECT_NEAR(yawOf(path.poses.back()), M_PI / 2.0, 1e-6);
}
