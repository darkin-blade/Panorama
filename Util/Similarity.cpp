#include "Similarity.h"

Scalar SSIM(
  const Mat & src_image,
  const Mat & dst_image,
  const Mat & mask)
{
  assert(src_image.size() == dst_image.size());
  assert(src_image.channels() == dst_image.channels());

  Mat tmp1, tmp2, i1, i2;
  src_image.copyTo(tmp1, mask);
  dst_image.copyTo(tmp2, mask);
  show_img("1", tmp1);
  show_img("2", tmp2);

  cvtColor(tmp1, tmp1, COLOR_RGBA2GRAY);
  cvtColor(tmp2, tmp2, COLOR_RGBA2GRAY);
  blur(tmp1, tmp1, Size(3, 3));
  blur(tmp2, tmp2, Size(3, 3));

  int edgeThresh = 100;
  Canny(tmp1, i1, edgeThresh, edgeThresh * 3, 3);
  Canny(tmp2, i2, edgeThresh, edgeThresh * 3, 3);

  // http://www.opencv.org.cn/opencvdoc/2.3.2/html/doc/tutorials/gpu/gpu-basics-similarity/gpu-basics-similarity.html
  const double C1 = 6.5025, C2 = 58.5225;

  Mat I1, I2;
  // cannot calculate on one byte large values
  i1.convertTo(I1, CV_32F);
  i2.convertTo(I2, CV_32F);

  Mat I1_2  = I1.mul(I1);// I_1^2
  Mat I2_2  = I2.mul(I2);// I_2^2
  Mat I1_I2 = I1.mul(I2);// I_1 * I_2

  Mat mu1, mu2;
  GaussianBlur(I1, mu1, Size(11, 11), 1.5);
  GaussianBlur(I2, mu2, Size(11, 11), 1.5);

  Mat mu1_2   = mu1.mul(mu1);
  Mat mu2_2   = mu2.mul(mu2);
  Mat mu1_mu2 = mu1.mul(mu2);

  Mat sigma1_2, sigma2_2, sigma12;
  
  GaussianBlur(I1_2, sigma1_2, Size(11, 11), 1.5);
  sigma1_2 -= mu1_2;
  
  GaussianBlur(I2_2, sigma2_2, Size(11, 11), 1.5);
  sigma2_2 -= mu2_2;
  
  GaussianBlur(I1_I2, sigma12, Size(11, 11), 1.5);
  sigma12 -= mu1_mu2;

  Mat t1, t2, t3;

  t1 = 2 * mu1_mu2 + C1;
  t2 = 2 * sigma12 + C2;
  t3 = t1.mul(t2);

  t1 = mu1_2 + mu2_2 + C1;
  t2 = sigma1_2 + sigma2_2 + C2;
  t1 = t1.mul(t2);

  Mat ssim_map;
  divide(t3, t1, ssim_map);

  Scalar mean_ssim, std_ssim;
  meanStdDev(ssim_map, mean_ssim, std_ssim, mask);
  LOG("score:");
  cout << mean_ssim << endl;
  cout << std_ssim << endl;
  return mean_ssim;
}