#ifndef COSTMAP_2D_TESTING_HELPER_H
#define COSTMAP_2D_TESTING_HELPER_H

#include<layered_costmap_2d/cost_values.h>
#include<layered_costmap_2d/costmap_2d.h>
#include <layered_costmap_2d/static_layer.h>
#include <costmap_2d/obstacle_layer.h>
#include <layered_costmap_2d/inflation_layer.h>

const double MAX_Z(1.0);

void setValues(layered_costmap_2d::Costmap2D& costmap, const unsigned char* map)
{
  int index = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      costmap.setCost(j, i, map[index]);
    }
  }
}

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case layered_costmap_2d::NO_INFORMATION: return '?';
  case layered_costmap_2d::LETHAL_OBSTACLE: return 'L';
  case layered_costmap_2d::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case layered_costmap_2d::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(layered_costmap_2d::Costmap2D& costmap)
{
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

unsigned int countValues(layered_costmap_2d::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(layered_costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  layered_costmap_2d::StaticLayer* slayer = new layered_costmap_2d::StaticLayer();
  layers.addPlugin(boost::shared_ptr<layered_costmap_2d::Layer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

layered_costmap_2d::ObstacleLayer* addObstacleLayer(layered_costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  layered_costmap_2d::ObstacleLayer* olayer = new layered_costmap_2d::ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
  layers.addPlugin(boost::shared_ptr<layered_costmap_2d::Layer>(olayer));
  return olayer;
}

void addObservation(layered_costmap_2d::ObstacleLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(1);
  cloud.points[0].x = x;
  cloud.points[0].y = y;
  cloud.points[0].z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  layered_costmap_2d::Observation obs(p, cloud, 100.0, 100.0);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

layered_costmap_2d::InflationLayer* addInflationLayer(layered_costmap_2d::LayeredCostmap& layers, tf::TransformListener& tf)
{
  layered_costmap_2d::InflationLayer* ilayer = new layered_costmap_2d::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<layered_costmap_2d::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
}


#endif  // COSTMAP_2D_TESTING_HELPER_H
