#if !defined(ImageData_H)
#define ImageData_H

#include "../common.h"

const int EDGE_VERTEX_SIZE = 2;

class Edge {
public:
  Edge(int _e1, int _e2) {
    indices[0] = _e1;
    indices[1] = _e2;
  }
  int indices[EDGE_VERTEX_SIZE];
};

class ImageData {
public:
  // Mesh2D
  int nw;// 横向mesh数目
  int nh;// 纵向mesh数目
  double lw;// mesh宽度
  double lh;// mesh高度
  
  vector<Point2f> mesh_points;// 网格点
  vector<vector<int> > polygons_indices;// TODO Indices
  vector<vector<int> > triangulation_indices;// TODO Indices
  vector<Edge> edges;
  vector<vector<int> > vertex_structures;// TODO Indices

  // ImageData
  Mat data;
  Mat grey_data;
  Mat rgba_data;
  Mat alpha_mask;// TODO
  char *name;

  vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
  vector<Point2f> feature_points;// 特征点(全部)
  vector<KeyPoint> key_points;// 关键点(全部)(包含特征点)

  // [i],以目前图片为目标,第i个图片为参照
  vector<vector<Point2f> > matching_points;// 此图片在第i个图片的匹配点
  vector<vector<Mat> > homographies;// 此图片的单应矩阵在第i张图片的单应矩阵

  void init_data(const char *img_path);
  void get_img(const char *img_path);
  /** Mesh2D **/
  void get_size();
  /** MeshGrid **/
  void getMesh2dPoints();// 所有mesh点
  void getPolygonsIndices();// 所有mesh点线性索引
  void getTriangulationIndices();// 将矩形区域划分为两个三角形
  void getEdges();// TODO
  void getVertexStructures();// TODO
  /** ImageData **/
};

#endif