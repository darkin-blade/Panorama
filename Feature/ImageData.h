#if !defined(ImageData_H)
#define ImageData_H

#include "../common.h"
#include "../Util/Statistics.h"

const int EDGE_VERTEX_SIZE = 2;

class Edge {
  public:
    Edge(int _e1, int _e2) {
      indices[0] = _e1;
      indices[1] = _e2;
    }
    int indices[EDGE_VERTEX_SIZE];
};

class InterpolateVertex {
  public:
    int polygon;
    vector<double> weights;
    InterpolateVertex() {
      polygon = -1;
    }
    InterpolateVertex(const InterpolateVertex & _iv) {
      polygon = _iv.polygon;
      weights = _iv.weights;
    }
    InterpolateVertex(const int _polygon,
        const double _w0, const double _w1, const double _w2) {
      polygon = _polygon;
      weights.emplace_back(_w0);
      weights.emplace_back(_w1);
      weights.emplace_back(_w2);
    }
    InterpolateVertex(const int _polygon,
        const vector<double> & _weights) {
      polygon = _polygon;
      weights = _weights;
    }
};

class ImageData {
  public:
    // Mesh2D
    int nw;// 横向mesh数目
    int nh;// 纵向mesh数目
    double lw;// mesh宽度
    double lh;// mesh高度

    vector<Point2f> mesh_points;// 网格点
    vector<int> boundary_edge_indices;// 边界顶点索引
    vector<vector<int> > polygons_indices;// TODO Indices
    vector<vector<int> > triangulation_indices;// TODO Indices
    vector<Edge> edges;
    vector<vector<int> > vertex_structures;// TODO Indices
    vector<vector<int> > polygons_neighbors;// TODO Indices
    vector<Point2f> polygons_center;// TODO
    vector<vector<int> > edge_structures;// TODO Indices

    // ImageData(直接获取成员)
    Mat data;
    Mat grey_data;
    Mat rgba_data;
    Mat alpha_mask;// TODO
    char *name;

    vector<vector<Mat> > descriptors;// TODO, 与feature_points数目相等
    vector<Point2f> feature_points;// 特征点(全部)

    // [i],以目前图片为目标,第i个图片为参照
    vector<vector<Point2f> > matching_points;// 此图片在第i个图片的匹配点
    vector<vector<Mat> > homographies;// 此图片的单应矩阵在第i张图片的单应矩阵

    // 直线检测
    vector<LineData> img_lines;// 直线

    void init_data(const char *img_path);
    void get_img(const char *img_path);
    /** Mesh2D **/
    void get_size();
    int getGridIndexOfPoint(const Point2f & _p);
    /** MeshGrid **/
    vector<int> getBoundaryVertexIndices();// 所有边界mesh点的index
    vector<Point2f> getVertices();// 所有mesh点
    vector<vector<int> > getPolygonsIndices();// 所有mesh点线性索引
    vector<vector<int> > getTriangulationIndices();// 将矩形区域划分为两个三角形
    vector<Edge> getEdges();
    vector<vector<int> > getPolygonsNeighbors();
    vector<Point2f> getPolygonsCenter();
    vector<vector<int> > getVertexStructures();
    vector<vector<int> > getEdgeStructures();
    InterpolateVertex getInterpolateVertex(const Point2f & _p);
    /** ImageData **/
    // 已删除
};

#endif
