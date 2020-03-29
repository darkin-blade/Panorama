#include "MultiImages.h"

MultiImages::MultiImages() {
  img_num = 0;
}

void MultiImages::read_img(const char *img_path) {
  // 读取图片
  ImageData *imageData = new ImageData();
  imageData->data = imread(img_path);

  // 检查图片数量
  imgs.push_back(imageData);
  img_num ++;
  assert(img_num == imgs.size());
}

vector<pair<int, int> > MultiImages::getInitialFeaturePairs(const int m1, const int m2) {
  const int nearest_size = 2;
  const bool ratio_test = true, intersect = true;

  assert(nearest_size > 0);

  const int feature_size[2];
  feature_size[0] = imgs[m1]->feature_points.size();
  feature_size[1] = imgs[m2]->feature_points.size();
  const int pair_match[2] = { m1, m2 };

  vector<FeatureDistance> feature_pairs[2];

  for(int p = 0; p < 1; ++p) {
    const int another_feature_size = feature_size[1 - p];
    const int nearest_k = min(nearest_size, another_feature_size);
    const vector<FeatureDescriptor> & feature_descriptors_1 = images_data[pair_match[ p]].getFeatureDescriptors();
    const vector<FeatureDescriptor> & feature_descriptors_2 = images_data[pair_match[!p]].getFeatureDescriptors();
    for(int f1 = 0; f1 < feature_size[p]; ++f1) {
      set<FeatureDistance> feature_distance_set;
      feature_distance_set.insert(FeatureDistance(MAXFLOAT, p, -1, -1));
      for(int f2 = 0; f2 < feature_size[!p]; ++f2) {
        const double dist = FeatureDescriptor::getDistance(feature_descriptors_1[f1], feature_descriptors_2[f2], feature_distance_set.begin()->distance);
        if(dist < feature_distance_set.begin()->distance) {
          if(feature_distance_set.size() == nearest_k) {
            feature_distance_set.erase(feature_distance_set.begin());
          }
          feature_distance_set.insert(FeatureDistance(dist, p, f1, f2));
        }
      }
      set<FeatureDistance>::const_iterator it = feature_distance_set.begin();
      if(ratio_test) {
        const set<FeatureDistance>::const_iterator it2 = std::next(it, 1);
        if(nearest_k == nearest_size &&
            it2->distance * FEATURE_RATIO_TEST_THRESHOLD > it->distance) {
          continue;
        }
        it = it2;
      }
      feature_pairs[p].insert(feature_pairs[p].end(), it, feature_distance_set.end());
    }
  }
  vector<FeatureDistance> feature_pairs_result = std::move(feature_pairs[0]);// 变量转移

  vector<double> distances;
  distances.reserve(feature_pairs_result.size());
  for(int i = 0; i < feature_pairs_result.size(); ++i) {
    distances.emplace_back(feature_pairs_result[i].distance);
  }
  double mean, std;
  Statistics::getMeanAndSTD(distances, mean, std);

  const double OUTLIER_THRESHOLD = (INLIER_TOLERANT_STD_DISTANCE * std) + mean;
  vector<pair<int, int> > initial_indices;
  initial_indices.reserve(feature_pairs_result.size());
  for(int i = 0; i < feature_pairs_result.size(); ++i) {
    if(feature_pairs_result[i].distance < OUTLIER_THRESHOLD) {
      initial_indices.emplace_back(feature_pairs_result[i].feature_index[0],
          feature_pairs_result[i].feature_index[1]);
    }
  }
  return initial_indices;
}

bool compareFeaturePair(const FeatureDistance &fd_1, const FeatureDistance &fd_2) {
  return
    (fd_1.feature_index[0] == fd_2.feature_index[0]) ?
    (fd_1.feature_index[1]  < fd_2.feature_index[1]) :
    (fd_1.feature_index[0]  < fd_2.feature_index[0]) ;
}