//#ifndef FEATUREEXTRACTION_HPP_
//#define FEATUREEXTRACTION_HPP_

#include <sqlite3.h> 
#include <Eigen/Core>
#include<tuple>
#include <colmap/base/camera.h>
#include <colmap/util/types.h>
#ifdef _MSC_VER
#if _MSC_VER >= 1600
#include <cstdint>
#else
typedef __int8 int8_t;
typedef __int16 int16_t;
typedef __int32 int32_t;
typedef __int64 int64_t;
typedef unsigned __int8 uint8_t;
typedef unsigned __int16 uint16_t;
typedef unsigned __int32 uint32_t;
typedef unsigned __int64 uint64_t;
#endif
#elif __GNUC__ >= 3
#include <cstdint>
#endif


namespace feature_extraction {
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    FeatureKeypointsBlob;

typedef Eigen::Matrix<uint32_t, Eigen::Dynamic, 2, Eigen::RowMajor>
    FeatureMatchesBlob;
typedef std::vector<std::pair<uint32_t,uint32_t>> MatchedPoints;

typedef Eigen::Matrix<double, 3,3,Eigen::RowMajor> Matrix3x3;
typedef uint32_t camera_t;




struct Matches_Geometry
{

	 char *sql="SELECT * FROM two_view_geometries;";
	 std::vector<std::pair<uint32_t,uint32_t>> pair_idx;
	 std::vector<int> rows;
	 std::vector<int> cols;// must =2
	 std::vector<MatchedPoints> points_idx;
	 std::vector<Matrix3x3> F;
	 std::vector<Matrix3x3> E;
	 std::vector<Matrix3x3> H;
	 std::tuple<uint32_t,uint32_t, int, int,MatchedPoints, Matrix3x3,Matrix3x3,Matrix3x3> operator[](size_t i) {

		auto ret = std::make_tuple(pair_idx[i].first,pair_idx[i].second, rows[i], cols[i],points_idx[i],F[i],E[i],H[i]);
	     return ret;
	    };

};
struct Feature_Keypoints
{
	char *sql="SELECT * FROM keypoints;";
	std::vector<int> images_idx;
	std::vector<int> rows;
	std::vector<int> cols;
	std::vector<FeatureKeypointsBlob> keypoints;
	std::tuple<int, int, int,FeatureKeypointsBlob> operator[](size_t i) {

			auto ret = std::make_tuple(images_idx[i], rows[i], cols[i],keypoints[i]);
		     return ret;
		    };
};


struct FeatureKeypoint
{

	  FeatureKeypoint();
	  FeatureKeypoint(const float x, const float y);
	  FeatureKeypoint(const float x, const float y, const float scale,
	                  const float orientation);
	  FeatureKeypoint(const float x, const float y, const float a11,
	                  const float a12, const float a21, const float a22);

	  static FeatureKeypoint FromParameters(const float x, const float y,
	                                        const float scale_x,
	                                        const float scale_y,
	                                        const float orientation,
	                                        const float shear);

	  // Rescale the feature location and shape size by the given scale factor.
	  void Rescale(const float scale);
	  void Rescale(const float scale_x, const float scale_y);

	  // Compute similarity shape parameters from affine shape.
	  float ComputeScale() const;
	  float ComputeScaleX() const;
	  float ComputeScaleY() const;
	  float ComputeOrientation() const;
	  float ComputeShear() const;

	  // Location of the feature, with the origin at the upper left image corner,
	  // i.e. the upper left pixel has the coordinate (0.5, 0.5).
	  float x;
	  float y;

	  // Affine shape of the feature.
	  float a11;
	  float a12;
	  float a21;
	  float a22;

};
//===================================================================================

FeatureKeypoint::FeatureKeypoint()
    : FeatureKeypoint(0, 0) {}

FeatureKeypoint::FeatureKeypoint(const float x, const float y)
    : FeatureKeypoint(x, y, 1, 0, 0, 1) {}

FeatureKeypoint::FeatureKeypoint(const float x_, const float y_,
                                 const float scale, const float orientation)
    : x(x_), y(y_) {
  CHECK_GE(scale, 0.0);
  const float scale_cos_orientation = scale * std::cos(orientation);
  const float scale_sin_orientation = scale * std::sin(orientation);
  a11 = scale_cos_orientation;
  a12 = -scale_sin_orientation;
  a21 = scale_sin_orientation;
  a22 = scale_cos_orientation;
}

FeatureKeypoint::FeatureKeypoint(const float x_, const float y_,
                                 const float a11_, const float a12_,
                                 const float a21_, const float a22_)
    : x(x_), y(y_), a11(a11_), a12(a12_), a21(a21_), a22(a22_) {}

FeatureKeypoint FeatureKeypoint::FromParameters(const float x, const float y,
                                                const float scale_x,
                                                const float scale_y,
                                                const float orientation,
                                                const float shear) {
  return FeatureKeypoint(x, y, scale_x * std::cos(orientation),
                         -scale_y * std::sin(orientation + shear),
                         scale_x * std::sin(orientation),
                         scale_y * std::cos(orientation + shear));
}

void FeatureKeypoint::Rescale(const float scale) {
  Rescale(scale, scale);
}

void FeatureKeypoint::Rescale(const float scale_x, const float scale_y) {
  CHECK_GT(scale_x, 0);
  CHECK_GT(scale_y, 0);
  x *= scale_x;
  y *= scale_y;
  a11 *= scale_x;
  a12 *= scale_y;
  a21 *= scale_x;
  a22 *= scale_y;
}

float FeatureKeypoint::ComputeScale() const {
  return (ComputeScaleX() + ComputeScaleY()) / 2.0f;
}

float FeatureKeypoint::ComputeScaleX() const {
  return std::sqrt(a11 * a11 + a21 * a21);
}

float FeatureKeypoint::ComputeScaleY() const {
  return std::sqrt(a12 * a12 + a22 * a22);
}

float FeatureKeypoint::ComputeOrientation() const {
  return std::atan2(a21, a11);
}

float FeatureKeypoint::ComputeShear() const {
  return std::atan2(-a12, a22) - ComputeOrientation();
}


typedef std::vector<FeatureKeypoint> FeatureKeypoints;
//===================================================================================
struct cameras_info
{
std::vector<camera_t> cameras_id;
std::vector<int> models;
std::vector<size_t> width;
std::vector<size_t> height;
std::vector<colmap::Camera> params;
std::vector<bool>  prior_focal_length;

};

class FeatureExtraction

{

public:

   bool connectDatabase();
   bool extract_matches();
   bool extract_keypoints();
   bool extract_camera_info();

   template <typename MatrixType>
   MatrixType MatrixFromBlob(sqlite3_stmt* sql_stmt,const int col);

   MatchedPoints MatchesFromMatrix(const FeatureMatchesBlob& blob);

   FeatureKeypoints FeatureKeypointsFromBlob(const FeatureKeypointsBlob& blob);

   colmap::Camera ReadCameraRow(sqlite3_stmt* sql_stmt);

   FeatureExtraction(bool is_single_camera_,char* datapath_); //Constructor
   virtual ~FeatureExtraction(); //Destructor

protected:

   Matches_Geometry matches;
   Feature_Keypoints keypoints;
   cameras_info cameras;
private:

  bool is_single_camera;

  char* datapath;

  char *sql_command;

  int rc;

  sqlite3 *db;
  char *zErrMsg;
  sqlite3_stmt * stmt;



};

}// namespace feature_extraction
