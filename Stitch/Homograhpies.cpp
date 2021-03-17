#include "Homographies.h"

void Homographies::compute(
    const vector<Point2f> & _p_src,
    const vector<Point2f> & _p_dst,
    const vector<Point2f> & _src,
    vector<Point2f>       & _dst,
    vector<Mat>           & _homographies) {
  LOG("APAP %ld %ld", _p_src.size(), _src.size());

  vector<Point2f> nf1, nf2, cf1, cf2;
  Mat N1, N2, C1, C2;
  // 归一化
  N1 = getNormalize2DPts(_p_src, nf1);
  N2 = getNormalize2DPts(_p_dst, nf2);
  // 均值为0,标准偏差为sqrt(2)
  C1 = getConditionerFromPts(nf1);
  C2 = getConditionerFromPts(nf2);
  cf1.reserve(nf1.size());
  cf2.reserve(nf2.size());

  for (int i = 0; i < nf1.size(); i ++) {
    cf1.emplace_back(nf1[i].x * C1.at<double>(0, 0) + C1.at<double>(0, 2),
        nf1[i].y * C1.at<double>(1, 1) + C1.at<double>(1, 2));

    cf2.emplace_back(nf2[i].x * C2.at<double>(0, 0) + C2.at<double>(0, 2),
        nf2[i].y * C2.at<double>(1, 1) + C2.at<double>(1, 2));
  }
  // 计算常量
  double sigma_inv_2 = 1. / (APAP_SIGMA * APAP_SIGMA);
  double gamma = APAP_GAMMA;

  MatrixXd A = MatrixXd::Zero(cf1.size() * DIMENSION_2D, HOMOGRAPHY_VARIABLES_COUNT);
  assert(_src.size() > 0);
  _dst.reserve(_src.size());
  _homographies.reserve(_src.size());

  for (int i = 0; i < _src.size(); i ++) {
    for (int j = 0; j < _p_src.size(); j ++) {
      Point2f d = _src[i] - _p_src[j];
      double tmp_w = exp(-sqrt(d.x * d.x + d.y * d.y) * sigma_inv_2);

      double www = max(gamma, tmp_w);

      A(2*j  , 0) = www * cf1[j].x;
      A(2*j  , 1) = www * cf1[j].y;
      A(2*j  , 2) = www * 1;
      A(2*j  , 6) = www * -cf2[j].x * cf1[j].x;
      A(2*j  , 7) = www * -cf2[j].x * cf1[j].y;
      A(2*j  , 8) = www * -cf2[j].x;

      A(2*j+1, 3) = www * cf1[j].x;
      A(2*j+1, 4) = www * cf1[j].y;
      A(2*j+1, 5) = www * 1;
      A(2*j+1, 6) = www * -cf2[j].y * cf1[j].x;
      A(2*j+1, 7) = www * -cf2[j].y * cf1[j].y;
      A(2*j+1, 8) = www * -cf2[j].y;
    }

    JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV);
    MatrixXd V = jacobi_svd.matrixV();
    Mat H(3, 3, CV_64FC1);
    // (row, col)
    for (int j = 0; j < V.rows(); j ++) {// 0 to 8
      // [0,0],[0,1],[0,2],[1,0],[1,1],[1,2],[2,0],[2,1],[2,2]
      H.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
    }

    // decondition
    H = C2.inv() * H * C1;
    // denormalise
    H = N2.inv() * H * N1;

    _dst.emplace_back(applyTransform3x3(_src[i].x, _src[i].y, H));
    _homographies.emplace_back(H);
  }
}