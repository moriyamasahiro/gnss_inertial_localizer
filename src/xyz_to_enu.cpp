#include "ekf.hpp"

const double ASPECT_RATIO = 1.653;
const double daa = 6378137; //長半径
const double dF = 298.257222101d; //逆扁平率
const double dM0 = 0.9999; //平面直角座標系のY軸上における縮尺係数(UTM座標系の場合→0.9996)

Eigen::Vector3d ekf::transform_xyz2enu(float x, float y, float z){
            double dn = 1. / (2. * dF - 1.);

              //ラジアン単位に
              double Lon0 = Deg2Rad(ekf::longitude0);
              double Lat0 = Deg2Rad(ekf::latitude0);


              //Sφ0、A
              double dA[6];
              dA[0] = 1. + pow(dn, 2.) / 4. + pow(dn, 4.) / 64.;
              dA[1] = -3./ 2.* (dn - pow(dn, 3.) / 8. - pow(dn, 5.) / 64.);
              dA[2] = 15./ 16.* (pow(dn, 2.) - pow(dn, 4.) / 4.);
              dA[3] = -35./ 48.* (pow(dn, 3.) - 5./ 16.* pow(dn, 5.));
              dA[4] = 315./ 512.* pow(dn, 4.);
              dA[5] = -693./ 1280.* pow(dn, 5.);
              double dAb = dM0 * daa / (1. + dn) * dA[0];
              double dSb = 0.;
              for (int j = 1; j <= 5; j++)
              {
                  dSb = dSb + dA[j] * sin(2 * j * Lat0);
              }
              dSb = dM0 * daa / (1. + dn) * (dA[0] * Lat0 + dSb);

              //ξ・η
              double dXi = (y + dSb) / dAb;
              double dEt = x / dAb;

              //β
              double dBt[6];
              dBt[1] = 1./ 2.* dn - 2./ 3.* pow(dn, 2.) + 37./ 96.* pow(dn, 3.) - 1./ 360.* pow(dn, 4.) - 81./ 512.* pow(dn, 5.);
              dBt[2] = 1./ 48.* pow(dn, 2.) + 1./ 15.* pow(dn, 3.) - 437./ 1440.* pow(dn, 4.) + 46./ 105.* pow(dn, 5.);
              dBt[3] = 17./ 480.* pow(dn, 3) - 37./ 840.* pow(dn, 4.) - 209./ 4480.* pow(dn, 5.);
              dBt[4] = 4397./ 161280.* pow(dn, 4.) - 11./ 504.* pow(dn, 5.);
              dBt[5] = 4583./ 161280.* pow(dn, 5.);

              //ξ’・η'・σ'・τ'・χ
              double dXi2 = 0.;
              double dEt2 = 0.;
              double dSg2 = 0.;
              double dTu2 = 0.;
              for (int j = 1; j <= 5; j++)
              {
                  dXi2 = dXi2 + dBt[j] * sin(2. * j * dXi) * cosh(2. * j * dEt);
                  dEt2 = dEt2 + dBt[j] * cos(2. * j * dXi) * sinh(2. * j * dEt);
                  dSg2 = dSg2 + dBt[j] * cos(2. * j * dXi) * cosh(2. * j * dEt);
                  dTu2 = dTu2 + dBt[j] * sin(2. * j * dXi) * sinh(2. * j * dEt);
              }
              dXi2 = dXi - dXi2;
              dEt2 = dEt - dEt2;
              dSg2 = 1 - dSg2;
              double dCi = asin(sin(dXi2) / cosh(dEt2));

              //δ
              double dDt[7];
              dDt[1] = 2. * dn - 2./ 3.* pow(dn, 2.) - 2. * pow(dn, 3.) + 116./ 45.* pow(dn, 4.) + 26./ 45.* pow(dn, 5.) - 2854./ 675.* pow(dn, 6.);
              dDt[2] = 7./ 3.* pow(dn, 2.) - 8./ 5.* pow(dn, 3.) - 227./ 45.* pow(dn, 4.) + 2704./ 315.* pow(dn, 5.) + 2323./ 945.* pow(dn, 6.);
              dDt[3] = 56./ 15.* pow(dn, 3.) - 136./ 35.* pow(dn, 4.) - 1262./ 105.* pow(dn, 5.) + 73814./ 2835.* pow(dn, 6.);
              dDt[4] = 4279./ 630.* pow(dn, 4.) - 332./ 35.* pow(dn, 5.) - 399572./ 14175.* pow(dn, 6.);
              dDt[5] = 4174./ 315.* pow(dn, 5.) - 144838./ 6237.* pow(dn, 6.);
              dDt[6] = 601676./ 22275.* pow(dn, 6.);

              //ラジアン単位の緯度経度
              double Lon = Lon0 + atan(sinh(dEt2) / cos(dXi2));
              double Lat = dCi;
              for (int j = 1; j <= 6; j++)
              {
                  Lat = Lat + dDt[j] * sin(2 * j * dCi);
              }

              //度単位に
              Lon = ekf::Rad2Deg(Lon);
              Lat = ekf::Rad2Deg(Lat);

              Eigen::Vector3d enu;
              enu << Lon, Lat, double(z);

              return enu;
}

double ekf::Deg2Rad(double degrees){
  double radians = degrees * M_PI / 180.;

  return radians;
}

double ekf::Rad2Deg(double radians){
  double degrees = 180. * radians / M_PI;

  return degrees;
}
