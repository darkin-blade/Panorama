#include "OpenCV_Stitching.h"

// 参数详细说明看stitching_detailed.cpp

Mat OpenCV_Stitching::opencv_stitch(vector<Mat> _images) {

  // 跑得更快, 效果更差
  bool preview = false;
  bool try_cuda = false;
  double work_megapix = 0.6;
  double seam_megapix = 0.1;
  double compose_megapix = -1;
  // 判断两张图片在一个全景中的threshold
  float conf_thresh = 0.3f;
  string features_type = "surf";// orb, akaze, surf, sift
  string matcher_type = "homography";
  string estimator_type = "homography";
  string ba_cost_func = "ray";
  string ba_refine_mask = "xxxxx";
  bool do_wave_correct = true;
  WaveCorrectKind wave_correct = detail::WAVE_CORRECT_HORIZ;
  bool save_graph = false;
  string warp_type = "spherical";
  int expos_comp_type = ExposureCompensator::GAIN_BLOCKS;
  int expos_comp_nr_feeds = 1;
  int expos_comp_nr_filtering = 2;
  int expos_comp_block_size = 32;
  // 特征匹配时的置信度, 不同方法默认值不一样
  float match_conf = 0.35f;
  string seam_find_type = "gc_color";
  int blend_type = Blender::MULTI_BAND;
  float blend_strength = 5;
  int range_width = -1;

  Mat failed_result = Mat::zeros(1, 1, CV_8UC3);

  double work_scale = 1, seam_scale = 1, compose_scale = 1;
  bool is_work_scale_set = false, is_seam_scale_set = false, is_compose_scale_set = false;

  int num_images = static_cast<int>(_images.size());
  if (num_images < 2) {
    return failed_result;
  }

  // 寻找特征点,参考: https://blog.csdn.net/zhaocj/article/details/78798687
  Ptr<Feature2D> finder;
  if (features_type == "orb") {
    finder = ORB::create();
  } else if (features_type == "akaze") {
    finder = AKAZE::create();
  } else if (features_type == "surf") {
    finder = xfeatures2d::SURF::create();
  } else if (features_type == "sift") {
    finder = xfeatures2d::SIFT::create();
  } else {
    return failed_result;
  }

  Mat full_img, img;
  vector<ImageFeatures> features(num_images);
  vector<Mat> images(num_images);
  vector<Size> full_img_sizes(num_images);
  double seam_work_aspect = 1;

  for (int i = 0; i < num_images; ++i) {
    full_img = _images[i].clone();
    full_img_sizes[i] = full_img.size();

    if (full_img.empty()) {
      assert(0);
    } if (work_megapix < 0) {
      img = full_img;
      work_scale = 1;
      is_work_scale_set = true;
    } else {
      if (!is_work_scale_set) {
        work_scale = min(1.0, sqrt(work_megapix * 1e6 / full_img.size().area()));
        is_work_scale_set = true;
      }
      resize(full_img, img, Size(), work_scale, work_scale, INTER_LINEAR_EXACT);
    }
    if (!is_seam_scale_set) {
      seam_scale = min(1.0, sqrt(seam_megapix * 1e6 / full_img.size().area()));
      seam_work_aspect = seam_scale / work_scale;
      is_seam_scale_set = true;
    }

    computeImageFeatures(finder, img, features[i]);
    features[i].img_idx = i;

    resize(full_img, img, Size(), seam_scale, seam_scale, INTER_LINEAR_EXACT);
    images[i] = img.clone();
  }

  full_img.release();
  img.release();

  // 进行特征匹配,参考: https://blog.csdn.net/zhaocj/article/details/78799194
  vector<MatchesInfo> pairwise_matches;
  Ptr<FeaturesMatcher> matcher;
  if (matcher_type == "affine")
    matcher = makePtr<AffineBestOf2NearestMatcher>(false, try_cuda, match_conf);
  else if (range_width == -1)// 默认选项
    matcher = makePtr<BestOf2NearestMatcher>(try_cuda, match_conf);// try_cude 默认 false, match_conf 默认 0.3
  else
    matcher = makePtr<BestOf2NearestRangeMatcher>(range_width, try_cuda, match_conf);

  (*matcher)(features, pairwise_matches);
  matcher->collectGarbage();

  // Leave only images we are sure are from the same panorama
  // 保存那些认为属于同一张全景照的图片
  vector<int> indices = leaveBiggestComponent(features, pairwise_matches, conf_thresh);
  vector<Mat> img_subset;
  vector<Size> full_img_sizes_subset;
  for (size_t i = 0; i < indices.size(); ++i) {
    img_subset.push_back(images[indices[i]]);
    full_img_sizes_subset.push_back(full_img_sizes[indices[i]]);
  }

  images = img_subset;
  full_img_sizes = full_img_sizes_subset;

  // Check if we still have enough images
  num_images = static_cast<int>(img_subset.size());
  if (num_images < 2) {
    return failed_result;
  }

  // 相机参数评估,参考: https://blog.csdn.net/zhaocj/article/details/78809143
  Ptr<Estimator> estimator;
  if (estimator_type == "affine")
    estimator = makePtr<AffineBasedEstimator>();
  else
    estimator = makePtr<HomographyBasedEstimator>();

  vector<CameraParams> cameras;
  if (!(*estimator)(features, pairwise_matches, cameras)) {
    return failed_result;
  }

  for (size_t i = 0; i < cameras.size(); ++i) {
    Mat R;
    cameras[i].R.convertTo(R, CV_32F);
    cameras[i].R = R;
  }

  // 细化相机参数,光束平差法
  Ptr<detail::BundleAdjusterBase> adjuster;
  if (ba_cost_func == "reproj") adjuster = makePtr<detail::BundleAdjusterReproj>();
  else if (ba_cost_func == "ray") adjuster = makePtr<detail::BundleAdjusterRay>();
  else if (ba_cost_func == "affine") adjuster = makePtr<detail::BundleAdjusterAffinePartial>();
  else if (ba_cost_func == "no") adjuster = makePtr<NoBundleAdjuster>();
  else {
    return failed_result;
  }
  adjuster->setConfThresh(conf_thresh);
  Mat_<uchar> refine_mask = Mat::zeros(3, 3, CV_8U);
  if (ba_refine_mask[0] == 'x') refine_mask(0,0) = 1;
  if (ba_refine_mask[1] == 'x') refine_mask(0,1) = 1;
  if (ba_refine_mask[2] == 'x') refine_mask(0,2) = 1;
  if (ba_refine_mask[3] == 'x') refine_mask(1,1) = 1;
  if (ba_refine_mask[4] == 'x') refine_mask(1,2) = 1;
  adjuster->setRefinementMask(refine_mask);
  if (!(*adjuster)(features, pairwise_matches, cameras)) {
    return failed_result;
  }

  // Find median focal length

  vector<double> focals;
  for (size_t i = 0; i < cameras.size(); ++i) {
    focals.push_back(cameras[i].focal);
  }

  sort(focals.begin(), focals.end());
  float warped_image_scale;
  if (focals.size() % 2 == 1)
    warped_image_scale = static_cast<float>(focals[focals.size() / 2]);
  else
    warped_image_scale = static_cast<float>(focals[focals.size() / 2 - 1] + focals[focals.size() / 2]) * 0.5f;

  // TODO 波形矫正
  if (do_wave_correct) {
    vector<Mat> rmats;
    for (size_t i = 0; i < cameras.size(); ++i)
      rmats.push_back(cameras[i].R.clone());
    waveCorrect(rmats, wave_correct);
    for (size_t i = 0; i < cameras.size(); ++i)
      cameras[i].R = rmats[i];
  }

  vector<Point> corners(num_images);
  vector<UMat> masks_warped(num_images);
  vector<UMat> images_warped(num_images);
  vector<Size> sizes(num_images);
  vector<UMat> masks(num_images);

  // Preapre images masks
  for (int i = 0; i < num_images; ++i) {
    masks[i].create(images[i].size(), CV_8U);
    masks[i].setTo(Scalar::all(255));
  }

  // Warp images and their masks

  // 图像投影变换,参考: https://blog.csdn.net/zhaocj/article/details/78829736
  Ptr<WarperCreator> warper_creator;
#ifdef HAVE_OPENCV_CUDAWARPING
  if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
  {
    if (warp_type == "plane")
      warper_creator = makePtr<cv::PlaneWarperGpu>();
    else if (warp_type == "cylindrical")
      warper_creator = makePtr<cv::CylindricalWarperGpu>();
    else if (warp_type == "spherical")
      warper_creator = makePtr<cv::SphericalWarperGpu>();
  }
  else
#endif
  {
    if (warp_type == "plane")
      warper_creator = makePtr<cv::PlaneWarper>();
    else if (warp_type == "affine")
      warper_creator = makePtr<cv::AffineWarper>();
    else if (warp_type == "cylindrical")
      warper_creator = makePtr<cv::CylindricalWarper>();
    else if (warp_type == "spherical")
      warper_creator = makePtr<cv::SphericalWarper>();
    else if (warp_type == "fisheye")
      warper_creator = makePtr<cv::FisheyeWarper>();
    else if (warp_type == "stereographic")
      warper_creator = makePtr<cv::StereographicWarper>();
    else if (warp_type == "compressedPlaneA2B1")
      warper_creator = makePtr<cv::CompressedRectilinearWarper>(2.0f, 1.0f);
    else if (warp_type == "compressedPlaneA1.5B1")
      warper_creator = makePtr<cv::CompressedRectilinearWarper>(1.5f, 1.0f);
    else if (warp_type == "compressedPlanePortraitA2B1")
      warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(2.0f, 1.0f);
    else if (warp_type == "compressedPlanePortraitA1.5B1")
      warper_creator = makePtr<cv::CompressedRectilinearPortraitWarper>(1.5f, 1.0f);
    else if (warp_type == "paniniA2B1")
      warper_creator = makePtr<cv::PaniniWarper>(2.0f, 1.0f);
    else if (warp_type == "paniniA1.5B1")
      warper_creator = makePtr<cv::PaniniWarper>(1.5f, 1.0f);
    else if (warp_type == "paniniPortraitA2B1")
      warper_creator = makePtr<cv::PaniniPortraitWarper>(2.0f, 1.0f);
    else if (warp_type == "paniniPortraitA1.5B1")
      warper_creator = makePtr<cv::PaniniPortraitWarper>(1.5f, 1.0f);
    else if (warp_type == "mercator")
      warper_creator = makePtr<cv::MercatorWarper>();
    else if (warp_type == "transverseMercator")
      warper_creator = makePtr<cv::TransverseMercatorWarper>();
  }

  if (!warper_creator) {
    return failed_result;
  }

  Ptr<RotationWarper> warper = warper_creator->create(static_cast<float>(warped_image_scale * seam_work_aspect));

  for (int i = 0; i < num_images; ++i) {
    Mat_<float> K;
    cameras[i].K().convertTo(K, CV_32F);
    float swa = (float)seam_work_aspect;
    K(0,0) *= swa; K(0,2) *= swa;
    K(1,1) *= swa; K(1,2) *= swa;

    corners[i] = warper->warp(images[i], K, cameras[i].R, INTER_LINEAR, BORDER_REFLECT, images_warped[i]);
    sizes[i] = images_warped[i].size();

    warper->warp(masks[i], K, cameras[i].R, INTER_NEAREST, BORDER_CONSTANT, masks_warped[i]);
  }

  vector<UMat> images_warped_f(num_images);
  // 图像数据类型转换
  for (int i = 0; i < num_images; ++i)
    images_warped[i].convertTo(images_warped_f[i], CV_32F);

  // TODO 曝光补偿
  Ptr<ExposureCompensator> compensator = ExposureCompensator::createDefault(expos_comp_type);
  if (dynamic_cast<GainCompensator*>(compensator.get())) {
    GainCompensator* gcompensator = dynamic_cast<GainCompensator*>(compensator.get());
    gcompensator->setNrFeeds(expos_comp_nr_feeds);
  }

  if (dynamic_cast<ChannelsCompensator*>(compensator.get())) {
    ChannelsCompensator* ccompensator = dynamic_cast<ChannelsCompensator*>(compensator.get());
    ccompensator->setNrFeeds(expos_comp_nr_feeds);
  }

  if (dynamic_cast<BlocksCompensator*>(compensator.get())) {
    BlocksCompensator* bcompensator = dynamic_cast<BlocksCompensator*>(compensator.get());
    bcompensator->setNrFeeds(expos_comp_nr_feeds);
    bcompensator->setNrGainsFilteringIterations(expos_comp_nr_filtering);
    bcompensator->setBlockSize(expos_comp_block_size, expos_comp_block_size);
  }

  compensator->feed(corners, images_warped, masks_warped);

  // TODO 寻找接缝线,参考: https://blog.csdn.net/zhaocj/article/details/78944867
  Ptr<SeamFinder> seam_finder;
  if (seam_find_type == "no")
    seam_finder = makePtr<detail::NoSeamFinder>();
  else if (seam_find_type == "voronoi")
    seam_finder = makePtr<detail::VoronoiSeamFinder>();
  else if (seam_find_type == "gc_color")// 默认
  {
#ifdef HAVE_OPENCV_CUDALEGACY
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
      seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR);
    else
#endif
      seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR);
  }
  else if (seam_find_type == "gc_colorgrad")
  {
#ifdef HAVE_OPENCV_CUDALEGACY
    if (try_cuda && cuda::getCudaEnabledDeviceCount() > 0)
      seam_finder = makePtr<detail::GraphCutSeamFinderGpu>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
    else
#endif
      seam_finder = makePtr<detail::GraphCutSeamFinder>(GraphCutSeamFinderBase::COST_COLOR_GRAD);
  }
  else if (seam_find_type == "dp_color")
    seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR);
  else if (seam_find_type == "dp_colorgrad")
    seam_finder = makePtr<detail::DpSeamFinder>(DpSeamFinder::COLOR_GRAD);
  if (!seam_finder) {
    return failed_result;
  }

  seam_finder->find(images_warped_f, corners, masks_warped);

  // Release unused memory
  images.clear();
  images_warped.clear();
  images_warped_f.clear();
  masks.clear();

  // TODO 图像融合,参考: https://blog.csdn.net/zhaocj/article/details/78960325
  Mat img_warped, img_warped_s;
  Mat dilated_mask, seam_mask, mask, mask_warped;
  Ptr<Blender> blender;// 图像融合的基类
  //double compose_seam_aspect = 1;
  double compose_work_aspect = 1;

  for (int img_idx = 0; img_idx < num_images; ++img_idx) {
    // Read image and resize it if necessary
    full_img = _images[img_idx].clone();// TODO 读取图片
    if (!is_compose_scale_set) {
      if (compose_megapix > 0)
        compose_scale = min(1.0, sqrt(compose_megapix * 1e6 / full_img.size().area()));
      is_compose_scale_set = true;

      // Compute relative scales
      //compose_seam_aspect = compose_scale / seam_scale;
      compose_work_aspect = compose_scale / work_scale;

      // Update warped image scale
      warped_image_scale *= static_cast<float>(compose_work_aspect);
      warper = warper_creator->create(warped_image_scale);

      // Update corners and sizes
      for (int i = 0; i < num_images; ++i) {
        // Update intrinsics
        cameras[i].focal *= compose_work_aspect;
        cameras[i].ppx *= compose_work_aspect;
        cameras[i].ppy *= compose_work_aspect;

        // Update corner and size
        Size sz = full_img_sizes[i];
        if (std::abs(compose_scale - 1) > 1e-1) {
          sz.width = cvRound(full_img_sizes[i].width * compose_scale);
          sz.height = cvRound(full_img_sizes[i].height * compose_scale);
        }

        Mat K;
        cameras[i].K().convertTo(K, CV_32F);
        Rect roi = warper->warpRoi(sz, K, cameras[i].R);
        corners[i] = roi.tl();
        sizes[i] = roi.size();
      }
    }
    if (abs(compose_scale - 1) > 1e-1)
      resize(full_img, img, Size(), compose_scale, compose_scale, INTER_LINEAR_EXACT);
    else
      img = full_img;
    full_img.release();
    Size img_size = img.size();

    Mat K;
    cameras[img_idx].K().convertTo(K, CV_32F);

    // Warp the current image
    warper->warp(img, K, cameras[img_idx].R, INTER_LINEAR, BORDER_REFLECT, img_warped);

    // Warp the current image mask
    mask.create(img_size, CV_8U);
    mask.setTo(Scalar::all(255));
    warper->warp(mask, K, cameras[img_idx].R, INTER_NEAREST, BORDER_CONSTANT, mask_warped);

    // Compensate exposure
    compensator->apply(img_idx, corners[img_idx], img_warped, mask_warped);

    img_warped.convertTo(img_warped_s, CV_16S);
    img_warped.release();
    img.release();
    mask.release();

    dilate(masks_warped[img_idx], dilated_mask, Mat());
    resize(dilated_mask, seam_mask, mask_warped.size(), 0, 0, INTER_LINEAR_EXACT);
    mask_warped = seam_mask & mask_warped;

    if (!blender) {
      blender = Blender::createDefault(blend_type, try_cuda);
      Size dst_sz = resultRoi(corners, sizes).size();
      float blend_width = sqrt(static_cast<float>(dst_sz.area())) * blend_strength / 100.f;
      if (blend_width < 1.f)
        blender = Blender::createDefault(Blender::NO, try_cuda);
      else if (blend_type == Blender::MULTI_BAND) {
        MultiBandBlender* mb = dynamic_cast<MultiBandBlender*>(blender.get());
        mb->setNumBands(static_cast<int>(ceil(log(blend_width)/log(2.)) - 1.));
      } else if (blend_type == Blender::FEATHER) {
        FeatherBlender* fb = dynamic_cast<FeatherBlender*>(blender.get());
        fb->setSharpness(1.f/blend_width);
      }
      blender->prepare(corners, sizes);
    }

    blender->feed(img_warped_s, mask_warped, corners[img_idx]);
  }

  Mat result, result_mask, result_RGBA;
  blender->blend(result, result_mask);
  result.convertTo(result, CV_8UC3);
  cvtColor(result, result, COLOR_RGB2RGBA);
  result_RGBA = Mat::zeros(result.cols, result.rows, CV_8UC4);
  result.copyTo(result_RGBA, result_mask);

  return result_RGBA;
}