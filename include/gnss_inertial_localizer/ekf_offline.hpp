#include "ros/ros.h"
#include "std_msgs/String.h"

#define _USE_MATH_DEFINES

#include "ekf.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sstream>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

class ekf_offline
{
    private:

    public:
      void initialize();
      void loop();
      bool getcsv(std::string path, std::vector<std::vector<std::string>>& values,const char delim);
      void write_data(std::vector<double> time_keeper, Eigen::MatrixXf);
      int get_index(std::vector<double> time_keeper);
};
