#include "ekf_offline.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "ekf_tester");
  ros::NodeHandle n;


  ekf ekf;
  ekf.initialize();

  float x;
  float y;
  std::vector<std::vector<std::string> > lgnss_data;

  int i_lgnss = 0;
  std::ifstream ifs("/home/mmasahiro/data/gnss_right.csv");

  std::string line;
  while (getline(ifs, line)) {
    std::istringstream stream(line);
    std::string field;
    std::vector<std::string> result;
    while (getline(stream, field, ',')) {
        result.push_back(field);
    }
    lgnss_data.push_back(result);
  }

  while(true){
    double longitude = std::stod(lgnss_data[i_lgnss][3]);
    double latitude = std::stod(lgnss_data[i_lgnss][2]);
    double altitude = std::stod(lgnss_data[i_lgnss][4]);

    Eigen::Vector3f xyz = ekf.transform_enu2xyz(longitude,latitude,altitude);
    x = xyz[0];
    y = xyz[1];
    std::fstream fs;

    fs.open("/home/mmasahiro/gnss_test.csv", std::ios::app);
    if(! fs.is_open()) {
    }
    fs << x << std::flush;
    fs << "," << y << std::endl;
    fs.close();

    i_lgnss++;
  }
  return 0;
}

bool getcsv(std::string path, std::vector<std::vector<std::string>>& values, const char delim){
    std::ifstream ifs(path);

    std::string line;
    while (getline(ifs, line)) {
      std::istringstream stream(line);
      std::string field;
      std::vector<std::string> result;
      while (getline(stream, field, delim)) {
          result.push_back(field);
      }
      values.push_back(result);
    }
    return true;
}

void write_data(float x, float y){
  std::fstream fs;

  fs.open("/home/mmasahiro/gnss_test.csv", std::ios::app);
  if(! fs.is_open()) {
      return;
  }
  fs << x << std::flush;
  fs << "," << y << std::endl;
  fs.close();
}
