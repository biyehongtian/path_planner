#ifndef ALGORITHM_H
#define ALGORITHM_H

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>

typedef ompl::base::SE2StateSpace::StateType State;

#include "node3d.h"
#include "node2d.h"
#include "visualize.h"
#include "collisiondetection.h"

namespace HybridAStar {
class Node3D;
class Node2D;
class Visualize;

/*!
 * \brief A class that encompasses the functions central to the search.
 包含搜索中心函数的类。 
 */
class Algorithm {
 public:
  /// The deault constructor
  Algorithm() {}

  // HYBRID A* ALGORITHM
  /*!
     \brief The heart of the planner, the main algorithm starting the search for a collision free and drivable path.
     \param start the start pose
     \param goal the goal pose
     \param nodes3D the array of 3D nodes representing the configuration space C in R^3
     \param nodes2D the array of 2D nodes representing the configuration space C in R^2
     \param width the width of the grid in number of cells
     \param height the height of the grid in number of cells
     \param configurationSpace the lookup of configurations and their spatial occupancy enumeration
     \param dubinsLookup the lookup of analytical solutions (Dubin's paths)
     \param visualization the visualization object publishing the search to RViz
     \return the pointer to the node satisfying the goal condition
        \规划者的心脏，主要的算法开始搜索无碰撞和可驾驶的路径。
        \开始做起始姿势
        \目标-目标姿势
        \nodes3d表示R^3中配置空间c的3d节点数组
        \nodes2d表示r^2中配置空间c的2d节点数组
        \以单元格数表示的网格宽度
        \网格的高度（以单元格数表    示）
        \配置空间配置的查找及其空间占用率枚举
        \Dubins查找分析解决方案的查找（Dubin的路径）
        \可视化将搜索发布到rviz的可视化对象
        \返回指向满足目标条件的节点的指针
  */
  static Node3D* hybridAStar(Node3D& start,
                             const Node3D& goal,
                             Node3D* nodes3D,
                             Node2D* nodes2D,
                             int width,
                             int height,
                             CollisionDetection& configurationSpace,
                             float* dubinsLookup,
                             Visualize& visualization);

  static Node2D* aStarDirect(const Node3D& start1,
            Node3D& goal1,
            Node2D* nodes2D,
            int width,
            int height,
            CollisionDetection& configurationSpace,
            Visualize& visualization);

};
}
#endif // ALGORITHM_H
