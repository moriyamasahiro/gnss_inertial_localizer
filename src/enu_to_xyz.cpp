#include "ekf.hpp"

const double ASPECT_RATIO = 1.653;
const double daa = 6378137; //長半径
const double dF = 298.257222101d; //逆扁平率
const double dM0 = 0.9999; //平面直角座標系のY軸上における縮尺係数(UTM座標系の場合→0.9996)

Eigen::Vector3f ekf::transform_enu2xyz(double longitude, double latitude, double altitude){
  double dn = 1. / (2. * dF - 1.);
  double Lon = ekf::Deg2Rad(longitude);
  double Lat = ekf::Deg2Rad(latitude);
  double Lon0 = ekf::Deg2Rad(ekf::longitude0);
  double Lat0 = ekf::Deg2Rad(ekf::latitude0);

  double dt = sinh(atanh(sin(Lat)) - (2. * sqrt(dn)) / (1. + dn) * atanh(2. * sqrt(dn) / (1. + dn) * sin(Lat)));
  double dtb = sqrt(1. + pow(dt, 2.));
  double dLmc = cos(Lon - Lon0);
  double dLms = sin(Lon - Lon0);
  double dXi = atan(dt / dLmc);
  double dEt = atanh(dLms / dtb);

  //α1→0～α5→4
  double dal[6];
  dal[0] = 0.;
  dal[1] = 1./ 2.* dn - 2./ 3.* pow(dn, 2.) + 5./ 16.* pow(dn, 3.) + 41./ 180.* pow(dn, 4.) - 127./ 288.* pow(dn, 5.);
  dal[2] = 13./ 48.* pow(dn, 2.) - 3./ 5.* pow(dn, 3.) + 557./ 1440.* pow(dn, 4.) + 281./ 630.* pow(dn, 5.);
  dal[3] = 61./ 240.* pow(dn, 3.) - 103./ 140.* pow(dn, 4.) + 15061./ 26880.* pow(dn, 5.);
  dal[4] = 49561./ 161280.* pow(dn, 4.) - 179./ 168.* pow(dn, 5.);
  dal[5] = 34729./ 80640.* pow(dn, 5.);
  double dSg = 0.; double dTu = 0.;
  for (int j = 1; j <= 5; j++)
  {
      dSg = dSg + 2. * j * dal[j] * cos(2. * j * dXi) * cosh(2. * j * dEt);
      dTu = dTu + 2. * j * dal[j] * sin(2. * j * dXi) * sinh(2. * j * dEt);
  }
  dSg = 1 + dSg;

  //A0-A5
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
      dSb = dSb + dA[j] * sin(2. * j * Lat0);
  }
  dSb = dM0 * daa / (1. + dn) * (dA[0] * Lat0 + dSb);

  double Y = 0.; double X = 0.;
  for (int j = 1; j <= 5; j++)
  {
    Y = Y + dal[j] * sin(2. * j * dXi) * cosh(2. * j * dEt);
    X = X + dal[j] * cos(2. * j * dXi) * sinh(2. * j * dEt);
  }
  Y = dAb * (dXi + Y) - dSb;
  X = dAb * (dEt + X);

  Eigen::Vector3f xyz;
  xyz << float(X), float(Y), float(altitude);

  return xyz;
}
