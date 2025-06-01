/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *scipa optimase
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/
#include <tuple>
#include <cmath>
#include <string>
#include <memory>
#include <random>
#include <iostream>
#include "nav2_util/node_utils.hpp"
#include "nav2_straightline_planner/straight_line_planner.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}


double distance(double x1, double y1, double x2, double y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

double heuristics(double x1, double y1,double goal_x, double goal_y){

   return abs(x1 - goal_x) + abs(x1 - goal_y);
}



nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;



  // std::cout <<"costmap_========================================================" << std::endl;
  unsigned int cost_r = costmap_->getCost(4, 5);
  double resolution = costmap_->getResolution();// wartość po jakiej poruszamy się po mapie 
  unsigned int size_x = costmap_->getSizeInCellsX();//rozmiar mapy x
  unsigned int size_y = costmap_->getSizeInCellsY();//rozmiar mapy y
  std::cout <<"costmap_ ::::" << cost_r <<" end_costmap" <<size_y <<"resolution"<< resolution<< std::endl;


  //do losowania punktów
  std::random_device rd; // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_x(0, size_x/resolution); // define the range
  std::uniform_int_distribution<> distr_y(0, size_y/resolution); // define the range



  // start.pose

  int amount_of_random_points = 9000;
  int ocupate = 200;
  double random_poins_tab[amount_of_random_points+2][2];





  int size_of_graph=0;
  



  random_poins_tab[0][0]= static_cast<int>(std::round(( start.pose.position.x - costmap_->getOriginX()) / costmap_->getResolution()));
  random_poins_tab[0][1] = static_cast<int>(std::round((start.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution()));




  for(int n=0; n <amount_of_random_points; ++n){//losowanie punktów
    double rand_x = distr_x(gen)*resolution;
    double rand_y = distr_y(gen)*resolution;

    unsigned int cost = costmap_->getCost(rand_x,rand_y);//wartość mapy w wylosowanym punkcie 

    double target[] = {rand_x, rand_y};
    int size_n = sizeof(random_poins_tab ) / sizeof(*random_poins_tab );
    bool exists = std::find(random_poins_tab, random_poins_tab + size_n, target) != random_poins_tab + size_n;//do sprawdzenia czy punkt się nie powtarza w tablicy wylosowanych punktów

    if( cost<ocupate && exists==false ) 
    {
      random_poins_tab[size_of_graph][0] = rand_x;
      random_poins_tab[size_of_graph][1] = rand_y;
      size_of_graph=size_of_graph+1;
    }
  }



  random_poins_tab[size_of_graph][0] = static_cast<int>(std::round((goal.pose.position.x- costmap_->getOriginX()) / costmap_->getResolution()));
  random_poins_tab[size_of_graph][1] = static_cast<int>(std::round((goal.pose.position.y - costmap_->getOriginY()) / costmap_->getResolution()));
  size_of_graph=size_of_graph+1;
// //tworzenie grafu 
// // jescze nie gotowe !!!!!!!!!!!!!

//   //GRAPH==================================================================================================================================
  std::cout <<"\n"<< size_of_graph<< "  "<<"Graph size================";
  double radious = 30;// obszar w którym punkty zostaną połączone 
  int graph[size_of_graph][size_of_graph][4];//tablica przechowująca graf   size_of_graph*size_of_graph*4
  
  int index_graph_1=0;
  // std::cout<<"size)graph "<<size_of_graph<<"gah";
  
  for(int i=0; i <size_of_graph; ++i){
    int index_graph_2=0;
    for(int j=0; j <size_of_graph; ++j){
      double distans=distance(random_poins_tab[i][0], random_poins_tab[i][1], random_poins_tab[j][0], random_poins_tab[j][1]);

      
    
      if(distans< radious){

        std::cout<<"rad :"<<radious<<"\n";

        if (i!=j){
    
                  // równanie lini
        //sprawdzenie czy między dwoma punktami nie ma przeszkody
          double a=((random_poins_tab[i][1]/resolution-random_poins_tab[j][1]/resolution)/(random_poins_tab[i][0]/resolution- random_poins_tab[j][0]/resolution));// wartośc do równania prostej między dowma 
          double b=(-a)*random_poins_tab[i][0]/resolution+random_poins_tab[i][1]/resolution;// wartośc do równania prostej między dwoma punktami
          double start = 0;
          double stop = 0;

          if(random_poins_tab[i][0]>random_poins_tab[j][0]){
            
            start = random_poins_tab[j][0] /resolution;
            stop = random_poins_tab[i][0]/resolution;
          }
          else{
            
            start = random_poins_tab[i][0] /resolution;
            stop = random_poins_tab[j][0]/resolution;
          }
          
          bool clear= false; 
          for(double index_x=start; index_x <stop ; ++index_x){// indeksowanie po prostej do prawdzenia czy nie przecina zajętego punktu
            double  index_y =  round(a * index_x + b);      
            unsigned int cost_between= costmap_->getCost(index_x*resolution,index_y*resolution);
            
            if (cost_between<ocupate){
              clear=true;
             
            }else{
              clear=false;

              // std::cout <<"\nfalse";
              break;
              
            }
       

          }
          // std::cout <<"graph"<< clear<<"next=========================";
          if (clear==true){
            double distance_heuristics=heuristics(random_poins_tab[j][0], random_poins_tab[j][1],goal.pose.position.x,goal.pose.position.y);
            graph[index_graph_1][index_graph_2][0]=i;
            graph[index_graph_1][index_graph_2][1]=j;
            graph[index_graph_1][index_graph_2][2]=distans;
            graph[index_graph_1][index_graph_2][3]=distance_heuristics;

            if(radious>5){
              radious=radious-0.1;
            }

            index_graph_2=index_graph_2+1;

          }

        }
      }

    }

    if (index_graph_2>0){
      index_graph_1=index_graph_1+1;
    }
    
  }
   std::cout <<"\ngraph_size"<<index_graph_1<<" X "<<"next=================\n========";




  //BFS#####################################################################################################################
  if (graph[index_graph_1-1][0][0]==size_of_graph-1){
    std::cout <<"____________________conection with goal point "<<size_of_graph-1 <<"=="<<graph[index_graph_1-1][0][0]<<"____________________";
  }
  std::cout <<" \npoint "<<size_of_graph <<"=="<<graph[index_graph_1-1][0][0]<<"____________________";




  bool end=false;
  double point_to_visit_value[index_graph_1][5];//tablica zawierająca indexy odwiedzonych punktów, odległość od rodzica, heurylystyke, index roadzica 
  int visited_point_list[index_graph_1];//tablica zawierająca indexy odwiedzonych punktów
  int point_to_visit_list[index_graph_1];//tablica zawierająca indexy punktów do odwiedzenia 
  int graph_path[index_graph_1][2];
  int index_while_to_visit=0;
  int size_visit_value=0;

  if(graph[0][0][0]==0){
    point_to_visit_list[0]=0;
    visited_point_list[0]=0;
    

    graph_path[0][0]=0;
    graph_path[0][1]=0;
    size_visit_value=size_visit_value+1;
    std::cout <<"______________conceted with start point"<< "__________________";

  }

  
 

  int index1_A = 1;
  int index_to_visit=1;
  int index_to_visitED=1;
  if (graph[index_graph_1-1][0][0]==size_of_graph-1){
    std::cout<<"conection_______";

    while( end == false){
      int point_to_visit=point_to_visit_list[0];
      visited_point_list[index1_A]=point_to_visit_list[0];

      // std::cout<<"..............end "<<point_to_visit<<" , "<<point_to_visit_list[0]<<" visit "<<index_graph_1-1<<">>>>>end_______";
      if (point_to_visit==(index_graph_1-1)){
        end = true;
        std::cout<<point_to_visit<<".."<<(index_graph_1-2)<<"===END,DDDDDDDDDDDDDDDDDDD===";
      }

          

      index1_A=index1_A+1;
      bool end_while=false;
      int index_while=0;
      
      while(end_while == false){
        if(graph[point_to_visit][index_while][3] == 0 && graph[point_to_visit][index_while][2] == 0){
          end_while=true;
          
        }
        // std::cout<<"==="<<graph[point_to_visit][index_while][1]<<"===";
        
        int target=0;
      
        for(int ix=0; ix <(index_graph_1 ); ++ix){
          // target=0;
          // std::cout<<"=="<<graph[indx][0][0]<<"_____"<< graph[point_to_visit][index_while][1]<<">>>>>";
          if(graph[ix][0][0]==graph[point_to_visit][index_while][1]){

            target=ix;
            int n = sizeof(point_to_visit_list) / sizeof(*point_to_visit_list);
            bool exists = std::find(point_to_visit_list, point_to_visit_list + n, target) != point_to_visit_list + n;

            int n2 = sizeof(visited_point_list) / sizeof(*visited_point_list);
            bool exists2 = std::find(visited_point_list, visited_point_list + n2, target) != visited_point_list + n2;

            if (exists==false && exists2==false) {
              point_to_visit_list[index_to_visit]=ix;
              graph_path[index_to_visitED][0]=ix;
              graph_path[index_to_visitED][1]=point_to_visit;
              index_to_visit=index_to_visit+1;
              index_to_visitED=index_to_visitED+1;
              
            }
            
            break;
            }
        }
        
        index_while=index_while+1;
      }
      for (int idx = 0; idx <sizeof(point_to_visit_list)-1; ++idx){
        point_to_visit_list[idx] = point_to_visit_list[idx + 1]; 
      }
      // std::cout<<"=="<<point_to_visit_list[0]<<">>>>>";
      

      index_to_visit=index_to_visit-1;

    }
  

    // std::cout<<"\n=="<<"11path start"<<">>>>>";
    // for (size_t i = 0; i < index_to_visitED; i++) {
    //       std::cout <<"\n"<< graph_path[i][0]<<" -- "<<graph_path[i][1] << " .. ";
    // }
    // std::cout<<"\n=="<<"11path end"<<">>>>>";
    
    int path[index_graph_1];
    int p = index_graph_1-1;
    path[0]=p;
    std::cout<<"\n=="<<path[0]<<".."<<index_graph_1-1<<">>>>>";
    int path_while_index=1;

    while (p!=0){
      for (int i = 0; i <index_to_visitED; ++i){
          if (p==graph_path[i][0]){
                  // std::cout<<"=="<<point_to_visit_list[0]<<">>>>>";

              path[path_while_index]=graph_path[i][1];
              p=graph_path[i][1];
              path_while_index=path_while_index+1;
              break;
          }
      } 
    }

    std::cout<<"\n=="<<"path start"<<path_while_index<<">>>>>";
    for (size_t i = 0; i < path_while_index; i++) {
          std::cout <<"\n"<< path[i] << " .. ";
    }
    std::cout<<"\n=="<<"path end"<<graph[path[0]][0][0]<<">>>>>";
  
    int path_size =path_while_index-1;

    for (int i = path_size; i > 0; --i) {

      int random_point_index_start = graph[path[i]][0][0];
      int random_point_index_stop = graph[path[i-1]][0][0];
      double lx_start = random_poins_tab[random_point_index_start][0];
      double ly_start = random_poins_tab[random_point_index_start][1];


      double lx_stop = random_poins_tab[random_point_index_stop][0];
      double ly_stop = random_poins_tab[random_point_index_stop][1];

      double wx_start=(lx_start * costmap_->getResolution()) + costmap_->getOriginX();
      double wy_start=(ly_start * costmap_->getResolution()) + costmap_->getOriginX();

      double wx_stop=(lx_stop * costmap_->getResolution()) + costmap_->getOriginX();
      double wy_stop=(ly_stop * costmap_->getResolution()) + costmap_->getOriginX();

      int total_number_of_loop = std::hypot(
      wx_stop - wx_start,
      wy_stop - wy_start) /
      interpolation_resolution_;

      double x_increment = (wx_stop - wx_start) / total_number_of_loop;
      double y_increment = (wy_stop - wy_start) / total_number_of_loop;



      for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = wx_start + x_increment * i;
        pose.pose.position.y = wy_start + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
      }
    }
    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    global_path.poses.push_back(goal_pose);

    return global_path;
  }
}

}  // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
