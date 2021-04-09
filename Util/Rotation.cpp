#include "Rotation.h"

/*** 
  计算照片的平均经度和纬度
  把所有点转换成球面向量并求解向量和
  rotations: 外在旋转的顺序, 即切面角->纬度->经度
  ***/
void angleConvert(const vector<vector<double> > photo_rotation, vector<double> & new_rotations) {
  int photo_num = photo_rotation.size();

  double tmp_sphere[3];
  double tmp_coordinate[3];
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;

  // 计算向量和
  for (int i = 0; i < photo_num; i ++) {
    vector<double> tmp_rotation = photo_rotation[i];
    tmp_sphere[0] = tmp_rotation[2];// 经度
    tmp_sphere[1] = tmp_rotation[1];// 纬度
    sphere2Coordinate(tmp_sphere, tmp_coordinate);
    sumX += tmp_coordinate[0];
    sumX += tmp_coordinate[1];
    sumX += tmp_coordinate[2];
  }

  // 将向量和位置换算成单位向量, 并转换成球面坐标
  double rate = sqrt(1 / (tmp_coordinate[0]*tmp_coordinate[0] + tmp_coordinate[1]*tmp_coordinate[1] + tmp_coordinate[2]*tmp_coordinate[2]));
  tmp_coordinate[0] *= rate;
  tmp_coordinate[1] *= rate;
  tmp_coordinate[2] *= rate;
  coordinate2Sphere(tmp_coordinate, tmp_sphere);

  // 保存中心位置
  double photo_center[2];
  photo_center[0] = tmp_sphere[0];// 经度
  photo_center[1] = tmp_sphere[1];// 纬度

  // 计算新的北极点
  double sphereN_[2];
  sphereN_[1] = photo_center[1] + M_PI / 2;// 纬度
  if (sphereN_[1] > M_PI / 2) {
    // 超过了北纬90
    sphereN_[1] = M_PI - sphereN_[1];
    // 计算经度
    if (photo_center[0] > 0) {
      sphereN_[0] = photo_center[0] - M_PI;
    } else {
      sphereN_[0] = photo_center[0] + M_PI;
    }
  } else {
    sphereN_[0] = photo_center[0];// 经度
  }
  double pointN_[3];
  sphere2Coordinate(sphereN_, pointN_);// 换算成空间直角坐标

  // 重新计算所有点的切面旋转角度
  new_rotations.clear();
  for (int i = 0; i < photo_num; i ++) {
    vector<double> tmp_rotation = photo_rotation[i];
    tmp_sphere[0] = tmp_rotation[2];// 经度
    tmp_sphere[1] = tmp_rotation[1];// 纬度
    tmp_sphere[2] = tmp_rotation[0];// 屏幕角度
    new_rotations.emplace_back(sphereThetaConvert(pointN_, tmp_sphere));
    LOG("new rotation %lf", new_rotations[i]);
  }
}

/*** 球面坐标系转空间直角坐标系, (经度, 纬度)->(x, y, z) ***/
void sphere2Coordinate(const double sphere[], double coordinate[]) {
  coordinate[2] = sin(sphere[1]);// 纬度->z
  double xy = sqrt(1 - coordinate[2]*coordinate[2]);// sqrt(x^2 + y^2)
  coordinate[0] = xy * cos(sphere[0]);// x
  coordinate[1] = xy * sin(sphere[0]);// x
}

/*** 空间直角坐标系转球面坐标系, (x, y, z)->(经度, 纬度) ***/
void coordinate2Sphere(const double coordinate[], double sphere[]) {
  sphere[0] = atan(coordinate[1] / coordinate[0]);// tan = y / x
  sphere[1] = asin(coordinate[2]);
  // 角度调整
  if (coordinate[1] > 0 && sphere[0] < 0) {
    sphere[0] += M_PI;
  } else if (coordinate[1] < 0 && sphere[0] > 0) {
    sphere[0] -= M_PI;
  }
}

/*** 
  计算P点绕axis旋转theta之后的坐标P'
  右手大拇指与axis同向, 四指的方向为正方向
  ***/
void rotateByVector(const double pointP[], const double vectorAxis[], const double _theta, double pointP_[]) {
  // 待旋转的点坐标
  double xP = pointP[0];
  double yP = pointP[1];
  double zP = pointP[2];
  // 向量的坐标
  double xV = vectorAxis[0];
  double yV = vectorAxis[1];
  double zV = vectorAxis[2];
  // 旋转
  double c = cos(_theta);
  double s = sin(_theta);
  double xP_ = (xV*xV*(1 - c) + c)*xP + (xV*yV*(1 - c) - zV*s)*yP + (xV*zV*(1 - c) + yV*s)*zP;
  double yP_ = (yV*xV*(1 - c) + zV*s)*xP + (yV*yV*(1- c) + c)*yP + (yV*zV*(1 - c) - xV*s)*zP;
  double zP_ = (zV*xV*(1 - c) - yV*s)*xP + (zV*yV*(1 - c) + xV*s)*yP + (zV*zV*(1 - c) + c)*zP;
  pointP_[0] = xP_;
  pointP_[1] = yP_;
  pointP_[2] = zP_;
}

/*** 
  计算求坐标系变换后, 新的切面旋转角度
  输入为: 新北极点(原空间直角坐标系下的坐标), 待变换的点(必须是3维, 包含切面旋转角度)
  ***/
double sphereThetaConvert(const double pointM[], const double sphereP[]) {
  // 临时变量
  double a, b, product, c;
  double pointN[3] = {0, 0, 1};// 原北极点N

  // 计算在P处的正北方向PR
  double pointP[3];
  double vectorPR[3];
  sphere2Coordinate(sphereP, pointP);
  // 计算OP与ON的夹角
  c = (pointN[0]*pointP[0] + pointN[1]*pointP[1] + pointN[2]*pointP[2])/
      (sqrt(pointN[0]*pointN[0] + pointN[1]*pointN[1] + pointN[2]*pointN[2])
      *sqrt(pointP[0]*pointP[0] + pointP[1]*pointP[1] + pointP[2]*pointP[2]));
  if (c > 1) {// TODO 余弦值误差
    c = 1;
  } else if (c < -1) {
    c = -1;
  }
  double thetaNOP = acos(c);
  // 向量分量
  a = 1 / fabs(sin(thetaNOP));
  b = -a * cos(thetaNOP);
  vectorPR[0] = a * pointN[0] + b * pointP[0];
  vectorPR[1] = a * pointN[1] + b * pointP[1];
  vectorPR[2] = a * pointN[2] + b * pointP[2];
  if (vectorPR[2] < 0) {
      // 求得方向为正南
      vectorPR[0] = -vectorPR[0];
      vectorPR[1] = -vectorPR[1];
      vectorPR[2] = -vectorPR[2];
  }

  // 计算PQ(手机的朝向): 在坐标系中角度旋转的正方向与函数的相反
  double thetaRPQ = sphereP[2];
  double vectorPQ[3];
  rotateByVector(vectorPR, pointP, -thetaRPQ, vectorPQ);
  
  // 求新直角坐标系下的正北方向PS
  double vectorPS[3];
  // 计算OP与OM的夹角
  c = (pointM[0]*pointP[0] + pointM[1]*pointP[1] + pointM[2]*pointP[2])/
      (sqrt(pointM[0]*pointM[0] + pointM[1]*pointM[1] + pointM[2]*pointM[2])
      *sqrt(pointP[0]*pointP[0] + pointP[1]*pointP[1] + pointP[2]*pointP[2]));
  if (c > 1) {
    c = 1;
  } else if (c < -1) {
    c = -1;
  }
  double thetaMOP = acos(c);
  // 计算分量
  a = 1 / fabs(sin(thetaMOP));
  b = -a * cos(thetaMOP);
  vectorPS[0] = a * pointM[0] + b * pointP[0];
  vectorPS[1] = a * pointM[1] + b * pointP[1];
  vectorPS[2] = a * pointM[2] + b * pointP[2];
  product = vectorPS[0]*pointM[0] + vectorPS[1]*pointM[1] + vectorPS[2]*pointM[2];
  if (product < 0) {
    // 方向为正南
    vectorPS[0] = -vectorPS[0];
    vectorPS[1] = -vectorPS[1];
    vectorPS[2] = -vectorPS[2];
  }

  // 用OM与OP的叉乘(右手定则)计算P处的正东方向PE
  double vectorPE[3];
  vectorPE[0] = pointM[1]*pointP[2] - pointP[1]*pointM[2];
  vectorPE[1] = pointP[0]*pointM[2] - pointM[0]*pointP[2];
  vectorPE[2] = pointM[0]*pointP[1] - pointP[0]*pointM[1];

  // 计算PS与PQ的夹角(0 - PI)
  c = (vectorPS[0]*vectorPQ[0] + vectorPS[1]*vectorPQ[1] + vectorPS[2]*vectorPQ[2])/
      (sqrt(vectorPS[0]*vectorPS[0] + vectorPS[1]*vectorPS[1] + vectorPS[2]*vectorPS[2])
      *sqrt(vectorPQ[0]*vectorPQ[0] + vectorPQ[1]*vectorPQ[1] + vectorPQ[2]*vectorPQ[2]));
  if (c > 1) {
    c = 1;
  } else if (c < -1) {
    c = -1;
  }
  double thetaSPQ = acos(c);

  // 判断PQ是向东(PE)还是向西
  product = vectorPQ[0]*vectorPE[0] + vectorPQ[1]*vectorPE[1] + vectorPQ[2]*vectorPE[2];
  if (product < 0) {
    // PQ朝向西方
    thetaSPQ = -thetaSPQ;
  }
  return thetaSPQ;
}