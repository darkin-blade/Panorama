#include "FeatureController.h"

double FeatureController::getDistance(
    const vector<Mat> & _descriptor1,
    const vector<Mat> & _descriptor2,
    const double _threshold) {
  const vector<Mat> & data1 = _descriptor1;
  const vector<Mat> & data2 = _descriptor2;
  double result = MAXFLOAT;
  for(int i = 0; i < data1.size(); ++i) {
    for(int j = 0; j < data2.size(); ++j) {
      double distance = 0;
      for(int k = 0; k < SIFT_DESCRIPTOR_DIM; ++k) {
        distance += ((data1[i].at<vl_sift_pix>(k) - data2[j].at<vl_sift_pix>(k)) *
            (data1[i].at<vl_sift_pix>(k) - data2[j].at<vl_sift_pix>(k)));

        /* at<vl_sift_pix>(k) == at<vl_sift_pix>(0, k) */

        if(distance >= _threshold) {
          break;
        }
      }
      result = min(result, distance);
    }
  }
  return result;
}

void FeatureController::detect(
    const Mat & _grey_img,// TODO
    vector<Point2f> & _feature_points,
    vector<vector<Mat> > & _feature_descriptors) {
  Mat grey_img_float = _grey_img.clone();
  grey_img_float.convertTo(grey_img_float, CV_32FC1);

  const int  width = _grey_img.cols;
  const int height = _grey_img.rows;

  VlSiftFilt * vlSift = vl_sift_new(width, height,
      log2(min(width, height)),  // noctaves
      SIFT_LEVEL_COUNT,          // nlevels
      SIFT_MINIMUM_OCTAVE_INDEX);// o_min

  LOG("sift args: [%lf %d %d]", log2(min(width, height)), SIFT_LEVEL_COUNT, SIFT_MINIMUM_OCTAVE_INDEX);

  vl_sift_set_peak_thresh(vlSift, SIFT_PEAK_THRESH);
  vl_sift_set_edge_thresh(vlSift, SIFT_EDGE_THRESH);

  if(vl_sift_process_first_octave(vlSift, (vl_sift_pix const *) grey_img_float.data) != VL_ERR_EOF) {
    do {
      vl_sift_detect(vlSift);// 特征点检测
      LOG("feature points: [%d]", vlSift->nkeys);
      for(int i = 0; i < vlSift->nkeys; i ++) {
        double angles[4];
        _feature_points.emplace_back(vlSift->keys[i].x, vlSift->keys[i].y);
        vector<Mat> descriptor;
        int angleCount = vl_sift_calc_keypoint_orientations(vlSift, angles, &vlSift->keys[i]);
        for(int j = 0; j < angleCount; j ++) {
          Mat descriptor_array(1, SIFT_DESCRIPTOR_DIM, CV_32FC1);
          vl_sift_calc_keypoint_descriptor(vlSift, (vl_sift_pix *) descriptor_array.data, &vlSift->keys[i], angles[j]);
          descriptor.emplace_back(descriptor_array);
        }
        _feature_descriptors.emplace_back(descriptor);
      }
    } while (vl_sift_process_next_octave(vlSift) != VL_ERR_EOF);
  }
  vl_sift_delete(vlSift);
}
