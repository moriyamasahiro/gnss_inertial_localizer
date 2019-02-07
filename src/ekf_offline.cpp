#include "ekf_offline.hpp"


void ekf_offline::initialize(){
}

void ekf_offline::loop(){
  int count = 0;
  ekf ekf;
  ekf.initialize();
  std::vector<std::vector<std::string> > imu_data;
  std::vector<std::vector<std::string> > lgnss_data;
  std::vector<std::vector<std::string> > rgnss_data;
  std::vector<std::vector<std::string> > image_data;
  ekf_offline::getcsv("/home/mmasahiro/data/imu.csv", imu_data,',');
  ekf_offline::getcsv("/home/mmasahiro/data/gnss_left.csv", lgnss_data,',');
  ekf_offline::getcsv("/home/mmasahiro/data/gnss_right.csv", rgnss_data,',');
  ekf_offline::getcsv("/home/mmasahiro/data/image.csv", image_data,',');

  double imu_time = std::stod(imu_data[0][0]) + std::stod(imu_data[0][1]) * 1e-9;
  double lgnss_time = std::stod(lgnss_data[0][0]) + std::stod(lgnss_data[0][1]) * 1e-9;
  double rgnss_time = std::stod(rgnss_data[0][0]) + std::stod(rgnss_data[0][1]) * 1e-9;
  double image_time = std::stod(image_data[0][0]) + std::stod(image_data[0][1]) * 1e-9;
  int i_imu = 0;
  int i_lgnss = 0;
  int i_rgnss = 0;
  int i_image = 0;

  std::vector<double> time_keeper{imu_time, lgnss_time, rgnss_time, image_time};
  std::vector<double> nnn{1e20, 1e20, 1e20};

  bool is_start = false;

  ros::Time t;
  t.sec = std::stoi(imu_data[0][0]);
  t.nsec = std::stoi(imu_data[0][1]);

  ekf.set_time(t);
  ekf.set_time(t);
  while(time_keeper[0] != 1e20 && time_keeper[1] != 1e20 &&time_keeper[2] != 1e20){
    if(get_index(time_keeper) == 0){
      t.sec = std::stoi(imu_data[i_imu][0]);
      t.nsec = std::stoi(imu_data[i_imu][1]);

      float accel_x = std::stof(imu_data[i_imu][2]);
      float accel_y = std::stof(imu_data[i_imu][3]);
      float accel_z = std::stof(imu_data[i_imu][4]);
      float gyro_x = std::stof(imu_data[i_imu][5]);
      float gyro_y = std::stof(imu_data[i_imu][6]);
      float gyro_z = std::stof(imu_data[i_imu][7]);

      ekf.predict_state(accel_x, -accel_y, -accel_z, gyro_x, -gyro_y, -gyro_z, t);
      ekf_offline::write_data(time_keeper, ekf.get_state());

      i_imu +=1;
      //if(sizeof(imu_data) <= i_imu){
      //  time_keeper[0] = 1e20;
      //  is_start = false;
      //}
      //else{
        time_keeper[0] = std::stod(imu_data[i_imu][0]) + std::stod(imu_data[i_imu][1]) * 1e-9;
        is_start = true;
      //}
    }

    else if(get_index(time_keeper) == 1){
      if(!is_start){
        i_lgnss +=1;
        //if(sizeof(lgnss_data) <= i_lgnss){
        //  time_keeper[1] = 1e20;
        //}
        //else{
          time_keeper[1] = std::stod(lgnss_data[i_lgnss][0]) + std::stod(lgnss_data[i_lgnss][1]) * 1e-9;
        //}
      }
      else{
        t.sec = std::stoi(lgnss_data[i_lgnss][0]);
        t.nsec = std::stoi(lgnss_data[i_lgnss][1]);
        double longitude = std::stod(lgnss_data[i_lgnss][3]);
        double latitude = std::stod(lgnss_data[i_lgnss][2]);
        double altitude = std::stod(lgnss_data[i_lgnss][4]);
        double x_cov = std::stof(lgnss_data[i_lgnss][5]);
        double y_cov = std::stof(lgnss_data[i_lgnss][6]);
        double z_cov = std::stof(lgnss_data[i_lgnss][7]);
        ekf.predict_state(t);
        ekf.update_state_with_lgnss(longitude, latitude, altitude, x_cov, y_cov, z_cov);
        ekf_offline::write_data(time_keeper, ekf.get_state());

        i_lgnss +=1;
        //if(sizeof(lgnss_data) <= i_lgnss){
        //  time_keeper[1] = 1e20;
        //}
        //else{
          time_keeper[1] = std::stod(lgnss_data[i_lgnss][0]) + std::stod(lgnss_data[i_lgnss][1]) * 1e-9;
        //}
      }
    }

    else if(get_index(time_keeper) == 2){
      if(!is_start){
        i_lgnss +=1;
        //if(sizeof(lgnss_data) <= i_lgnss){
        //  time_keeper[1] = 1e20;
        //}
        //else{
          time_keeper[2] = std::stod(rgnss_data[i_rgnss][0]) + std::stod(rgnss_data[i_rgnss][1]) * 1e-9;
        //}
      }
      else{
        std::cout << "1 " << std::endl;
        t.sec = std::stoi(rgnss_data[i_rgnss][0]);
        t.nsec = std::stoi(rgnss_data[i_rgnss][1]);
        double longitude = std::stod(rgnss_data[i_rgnss][3]);
        double latitude = std::stod(rgnss_data[i_rgnss][2]);
        double altitude = std::stod(rgnss_data[i_rgnss][4]);
        std::cout << "b" << std::endl;
        double x_cov = std::stof(rgnss_data[i_rgnss][5]);
        double y_cov = std::stof(rgnss_data[i_rgnss][6]);
        double z_cov = std::stof(rgnss_data[i_rgnss][7]);
        std::cout << "c" << std::endl;
        ekf.predict_state(t);
        std::cout << "d" << std::endl;
        ekf.update_state_with_rgnss(longitude, latitude, altitude, x_cov, y_cov, z_cov);
        std::cout << "e"<< std::endl;
        ekf_offline::write_data(time_keeper, ekf.get_state());
        std::cout << "f" << std::endl;

        i_rgnss +=1;
        //if(sizeof(lgnss_data) <= i_lgnss){
        //  time_keeper[1] = 1e20;
        //}
        //else{
        time_keeper[2] = std::stod(rgnss_data[i_rgnss][0]) + std::stod(rgnss_data[i_rgnss][1]) * 1e-9;
        //}
      }
    }

    else if(get_index(time_keeper) == 3){
      if(!is_start){
        i_image +=1;
        //if(sizeof(image_data) <= i_image){
        //  time_keeper[2] = 1e20;
        //}
        //else{
          time_keeper[3] = std::stod(image_data[i_image][0]) + std::stod(image_data[i_image][1]) * 1e-9;
        //}
      }
      else{
        t.sec = std::stoi(image_data[i_image][0]);
        t.nsec = std::stoi(image_data[i_image][1]);


        ekf.predict_state(t);
        ekf_offline::write_data(time_keeper, ekf.get_state());
        i_image +=1;
        //if(sizeof(image_data) <= i_image){
        //  time_keeper[2] = 1e20;
        //}
        //else{
          time_keeper[3] = std::stod(image_data[i_image][0]) + std::stod(image_data[i_image][1]) * 1e-9;
        //}
      }
    }

    ekf.debug_msg();
  }
  std::cout << "finished" << std::endl;
  std::cout << "count is " << sizeof(imu_data) << std::endl;
  for(int j=0;j <sizeof(imu_data[0]);j++){
    std::cout << imu_data[0][j] << "," << std::flush;
  }
}

bool ekf_offline::getcsv(std::string path, std::vector<std::vector<std::string>>& values, const char delim){
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

void ekf_offline::write_data(std::vector<double> time_keeper, Eigen::MatrixXf state){
  std::fstream fs;

  fs.open("/home/mmasahiro/ekf.csv", std::ios::app);
  if(! fs.is_open()) {
      return;
  }
  fs << get_index(time_keeper) << std::flush;
  fs << "," << time_keeper[get_index(time_keeper)] << std::flush;
  for(int i=0;i<10;i++){
    fs << "," << state(i,0) << std::flush;
  }
  fs << "" << std::endl;

  fs.close();
}

int ekf_offline::get_index(std::vector<double> time_keeper){
  double timer = 1e15;
  int index = 0;
  for(int i=0;i<3;i++){
    if(time_keeper[i] < timer){
      timer = time_keeper[i];
      index = i;
    }
  }
  return index;
}
