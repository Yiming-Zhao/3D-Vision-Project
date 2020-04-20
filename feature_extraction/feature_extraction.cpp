

#include <cstdlib>
#include <iostream>
#include <sqlite3.h> 

#include <colmap/util/option_manager.h>
#include <colmap/util/string.h>
#include <colmap/base/camera.h>

#include "feature_extraction.h"

namespace feature_extraction {

bool FeatureExtraction::connectDatabase()
{

   if(sqlite3_open(datapath, &db)) {
      fprintf(stderr, "Can't open database %s: %s\n",datapath, sqlite3_errmsg(db));
      return false;
   } else {
      fprintf(stdout, "Opened database %s successfully\n",datapath);
      return true;
   }
}

bool FeatureExtraction::extract_matches()
{
	sql_command=matches.sql;

	rc=sqlite3_prepare(db,sql_command, -1, &stmt, NULL ); 

     	if (rc != SQLITE_OK){
         	throw std::string(sqlite3_errmsg(db));
         	return false;
		}
		else{
			std::cout<<"The statement for matches extraction was prepared successfully\n"
			    <<"=======================================\n"<<std::endl;
	}

      while(sqlite3_step(stmt) == SQLITE_ROW) {
     

       int column = sqlite3_column_count(stmt);

       for(int i = 0; i < column; i++)
       {

    	  if(sqlite3_column_name(stmt,i)=="pair_id")
    	  {
    		  uint32_t pair_id=sqlite3_column_int(stmt,i);
    		  uint32_t image_id2 = pair_id % 2147483647;
    		  uint32_t image_id1 = (pair_id - image_id2) / 2147483647;
		    std::pair<uint32_t,uint32_t> images_id(image_id1,image_id2);
			  matches.pair_idx.push_back(images_id);

    	  }

          if(sqlite3_column_name(stmt,i)=="rows")
          {
        	  int rows=sqlite3_column_int(stmt,i);
        	  matches.rows.push_back(rows);
		      int cols=sqlite3_column_int(stmt,i+1);
		      matches.cols.push_back(cols);

		    FeatureMatchesBlob match_matrix=MatrixFromBlob<FeatureMatchesBlob>(stmt,i);
		      MatchedPoints points_idx = MatchesFromMatrix(match_matrix);
		      matches.points_idx.push_back(points_idx);


          }

          if(sqlite3_column_name(stmt,i)=="F")
          {
        	  auto F_ma=MatrixFromBlob<Matrix3x3>(stmt,i);
        	  matches.F.push_back(F_ma);

          }

          if(sqlite3_column_name(stmt,i)=="E")
          {
        	  auto E_ma=MatrixFromBlob<Matrix3x3>(stmt,i);
        	  matches.E.push_back(E_ma);
          }

          if(sqlite3_column_name(stmt,i)=="H")
          {
        	  auto H_ma=MatrixFromBlob<Matrix3x3>(stmt,i);
        	  matches.F.push_back(H_ma);
          }


       }
     }

    sqlite3_finalize(stmt);

    std::cout << "\n\nDatabase was closed successfully\n\n"<<std::endl;
    return true;
}
bool FeatureExtraction::extract_keypoints()
{
	sql_command="SELECT * FROM two_view_geometries;";

	rc=sqlite3_prepare(db,sql_command, -1, &stmt, NULL );

     	if (rc != SQLITE_OK){
         	throw std::string(sqlite3_errmsg(db));
         	return false;
		}
		else{
			std::cout<<"The statement for matches extraction was prepared successfully\n"
			    <<"=======================================\n"<<std::endl;
	}

      while(sqlite3_step(stmt) == SQLITE_ROW) {


       int column = sqlite3_column_count(stmt);

       for(int i = 0; i < column; i++)
       {

    	  if(sqlite3_column_name(stmt,i)=="image_id")
    	  {
    		  uint32_t image_id=sqlite3_column_int(stmt,i);

    		  keypoints.images_idx.push_back(image_id);

    	  }

          if(sqlite3_column_name(stmt,i)=="rows")
          {
        	  int rows=sqlite3_column_int(stmt,i);
        	  keypoints.rows.push_back(rows);
		      int cols=sqlite3_column_int(stmt,i+1);
		      keypoints.cols.push_back(cols);

		      FeatureKeypointsBlob Keypoints_matrix=MatrixFromBlob<FeatureKeypointsBlob>(stmt,i);
		      keypoints.keypoints.push_back(Keypoints_matrix);


          }


       }
     }

    sqlite3_finalize(stmt);

    std::cout << "\n\nDatabase was closed successfully\n\n"<<std::endl;
    return true;

}

bool FeatureExtraction::extract_camera_info()
{
	sql_command="SELECT * FROM cameras;";

			rc=sqlite3_prepare(db,sql_command, -1, &stmt, NULL );

		     	if (rc != SQLITE_OK){
		         	throw std::string(sqlite3_errmsg(db));
		         	return false;
				}
				else{
					std::cout<<"The statement for matches extraction was prepared successfully\n"
					    <<"=======================================\n"<<std::endl;
			}

		      while(sqlite3_step(stmt) == SQLITE_ROW) {



		 	    		  colmap::Camera ith_camera = ReadCameraRow(stmt);
		 	    		  cameras.params.push_back(ith_camera);
		 	    		  cameras.cameras_id.push_back(ith_camera.CameraId());
		 	    		  cameras.models.push_back(ith_camera.ModelId());
		 	    		  cameras.width.push_back(ith_camera.Width());
		 	    		  cameras.height.push_back(ith_camera.Height());
		 	    		  cameras.prior_focal_length.push_back(ith_camera.HasPriorFocalLength());

		 	    		  if(is_single_camera) break;
		       }



		    sqlite3_finalize(stmt);

		    std::cout << "\n\nDatabase was closed successfully\n\n"<<std::endl;
		    return true;
}


FeatureExtraction::FeatureExtraction(bool is_single_camera_,char* datapath_)
 : is_single_camera(is_single_camera_),datapath(datapath_)
{
	std::cout<<"The class FeatureExtraction was initialized successfully\n"
			 <<"========================================================\n"<<std::endl;
}

FeatureExtraction::~FeatureExtraction()
{

	sqlite3_close(db);
}







colmap::Camera FeatureExtraction::ReadCameraRow(sqlite3_stmt* sql_stmt) {
  colmap::Camera camera;

  camera.SetCameraId(static_cast<camera_t>(sqlite3_column_int64(sql_stmt, 0)));
  camera.SetModelId(sqlite3_column_int64(sql_stmt, 1));
  camera.SetWidth(static_cast<size_t>(sqlite3_column_int64(sql_stmt, 2)));
  camera.SetHeight(static_cast<size_t>(sqlite3_column_int64(sql_stmt, 3)));

  const size_t num_params_bytes =
      static_cast<size_t>(sqlite3_column_bytes(sql_stmt, 4));
  const size_t num_params = num_params_bytes / sizeof(double);
  CHECK_EQ(num_params, camera.NumParams());
  memcpy(camera.ParamsData(), sqlite3_column_blob(sql_stmt, 4),
         num_params_bytes);

  camera.SetPriorFocalLength(sqlite3_column_int64(sql_stmt, 5) != 0);

  return camera;
}

template <typename MatrixType>
MatrixType FeatureExtraction::MatrixFromBlob(sqlite3_stmt* sql_stmt,const int col) {

  MatrixType matrix;
  int offset;
  if(typeid(MatrixType)!=typeid(Matrix3x3))
  {
	  const size_t rows =
			  static_cast<size_t>(sqlite3_column_int64(sql_stmt, col + 0));
	  const size_t cols =
			  static_cast<size_t>(sqlite3_column_int64(sql_stmt, col + 1));
	  matrix = MatrixType(rows, cols);

	  offset=2;

  }else
  {
	  matrix = MatrixType(3,3);
	  offset=0;
  }


  const size_t num_bytes =
		  static_cast<size_t>(sqlite3_column_bytes(sql_stmt, col + offset));

  memcpy(reinterpret_cast<char*>(matrix.data()),sqlite3_column_blob(sql_stmt, col + offset), num_bytes);

  return matrix;
}





//keep it if necessary in later computation of trifocal tensor
//typedef std::vector<std::pair<uint32_t,uint32_t>> MatchedPoints;

MatchedPoints FeatureExtraction::MatchesFromMatrix(const FeatureMatchesBlob& blob) {

	MatchedPoints matches(static_cast<size_t>(blob.rows()));
  for (FeatureMatchesBlob::Index i = 0; i < blob.rows(); ++i) {
    matches[i].first = blob(i, 0);
    matches[i].second = blob(i, 1);
  }
  return matches;
}

FeatureKeypoints FeatureExtraction::FeatureKeypointsFromBlob(const FeatureKeypointsBlob& blob) {
  FeatureKeypoints keypoints(static_cast<size_t>(blob.rows()));
  if (blob.cols() == 2) {
    for (FeatureKeypointsBlob::Index i = 0; i < blob.rows(); ++i) {
      keypoints[i] = FeatureKeypoint(blob(i, 0), blob(i, 1));
    }
  } else if (blob.cols() == 4) {
    for (FeatureKeypointsBlob::Index i = 0; i < blob.rows(); ++i) {
      keypoints[i] =
          FeatureKeypoint(blob(i, 0), blob(i, 1), blob(i, 2), blob(i, 3));
    }
  } else if (blob.cols() == 6) {
    for (FeatureKeypointsBlob::Index i = 0; i < blob.rows(); ++i) {
      keypoints[i] = FeatureKeypoint(blob(i, 0), blob(i, 1), blob(i, 2),
                                     blob(i, 3), blob(i, 4), blob(i, 5));
    }
  } else {
    std::cout<< "Keypoint format not supported"<<std::endl;
  }
  return keypoints;
}


}  // namespace feature_extraction


int main(int argc, char** argv) {
    colmap::InitializeGlog(argv);

    std::string input_path;
    std::string output_path;

    colmap::OptionManager options;
    options.AddRequiredOption("input_path", &input_path);
    options.AddRequiredOption("output_path", &output_path);
    options.Parse(argc, argv);

    //TODO: process images


char* db_filename = new char[output_path.length() + strlen("database.db") + 1];

strcpy(db_filename, output_path.c_str());

strcat(db_filename, "database.db");

feature_extraction::FeatureExtraction fe(true,db_filename);



   if(!fe.connectDatabase())

   return 0;
   fe.extract_matches();
   fe.extract_keypoints();
   fe.extract_camera_info();
   return 1;




}











