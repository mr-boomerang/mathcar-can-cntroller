#include <utils/gui/objects_3d.hpp>

#include <utils/mode_printing.h>

#include <utils/fs_utils.h>

#include <utils/tinyxml2.h>

#include <utils/config.h>

#include <utils/gui/gps_to_utm.h>

#include <utils/TicToc.h>

//#include<tf/StampedTransform.h>
//#include <transform_datatypes.h>
#define VBO_ELEMS 3
#define CBO_ELEMS 3
using namespace utils;
using namespace gps_common;
/*--------------------------AxisObject Start----------------------------------*/

int AxisObject::reinit(float s, const Eigen::Matrix4f &p) {
  try {
    scale_ = s;
    pose_ = p;
    line_width_ = 1.0f;
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int AxisObject::draw_obj(pangolin::View & /*v*/) {
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  glColor3f(1.0f, 1.0f, 1.0f);
  glLineWidth(line_width_);
  pangolin::glDrawAxis(pose_, scale_);
  glColor3f(1.0f, 1.0f, 1.0f);
  glLineWidth(1.0f);
  CHECK_GL_ERROR(this);
  return RET_SUCCESS_INT;
}

/*--------------------------AxisObject End------------------------------------*/

/*--------------------------ZeroGridObject Start------------------------------*/

int ZeroGridObject::reinit(Alignment a, float scale, int num_grid) {
  a_ = a;
  scale_ = scale;
  n_grid_ = num_grid;
  line_width_ = 1.0f;
  return RET_SUCCESS_INT;
}

int ZeroGridObject::draw_obj(pangolin::View & /*v*/) {
  // glDisable(GL_DEPTH_TEST);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  try {
    glColor3fv(col_);
    glLineWidth(line_width_);
    switch (a_) {
    case XGrid:
      pangolin::glDraw_x0(scale_, n_grid_);
      break;
    case YGrid:
      pangolin::glDraw_y0(scale_, n_grid_);
      break;
    case ZGrid:
      pangolin::glDraw_z0(scale_, n_grid_);
      break;
    default:
      break;
    }
    glColor3f(1.0f, 1.0f, 1.0f);
    glLineWidth(1.0f);
    CHECK_GL_ERROR(this);
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

/*--------------------------ZeroGridObject End--------------------------------*/

/*--------------------------TrajectoryObject Start----------------------------*/

int TrajectoryObject::reinit(size_t keep, bool follow) {
  try {
    pose_.SetIdentity();
    ref_frame_.SetIdentity();
    keep_ = keep;
    follow_ = follow;
    target_frame_ = "";
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int TrajectoryObject::draw_obj(pangolin::View & /*v*/) {

  CHECK_RET(follow());
  if (cam_ != NULL)
    cam_->Apply();
  GLfloat model[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, model);

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  Eigen::Matrix4f ref_frame_pos = ref_frame_;
  // Set reference frame
  pangolin::glSetFrameOfReference(ref_frame_pos);
  /* draw points */
  glPointSize(2 * traj_size_);
  glColor3fv(traj_pts_col_);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < traj_.size(); i += 3) {
    glVertex3fv(&traj_[i]);
  }
  glEnd();

  /* draw line */
  glLineWidth(traj_size_);
  glColor3fv(traj_line_col_);
  glBegin(GL_LINES);
  for (size_t i = 3; i < traj_.size(); i += 3) {
    glVertex3fv(&traj_[i - 3]);
    glVertex3fv(&traj_[i]);
  }
  glEnd();

  glLineWidth(1.0);
  Eigen::Matrix4f pos = pose_;
  pangolin::glDrawAxis(pos, axis_size_);
  glColor3fv(traj_pts_col_);

  // 141016:1249 draws arrow..
  //

  float arrow_len = 0.3;
  float arrow_rad = 0.05;
  float arrow_head_len = arrow_len / 3;
  pangolin::glSetFrameOfReference(pos);

  // code taken from glDrawCirclePerimeter()
  const int N = 50;
  const float TAU_DIV_N = 2 * (float)M_PI / N;
  // this draws a hollow cone..
  glBegin(GL_TRIANGLE_FAN);
  glVertex3f(arrow_len, 0.0, 0.0);
  for (int i = 0; i < N * 2; i += 2) {
    glVertex3f(arrow_len - arrow_head_len, arrow_rad * cos(i * TAU_DIV_N),
               arrow_rad * sin(i * TAU_DIV_N));
  }
  glEnd();
  // this draws a cone base..ie circle
  glBegin(GL_POLYGON);
  for (int i = 0; i < N * 2; i += 2) {
    glVertex3f(arrow_len - arrow_head_len, arrow_rad * cos(i * TAU_DIV_N),
               arrow_rad * sin(i * TAU_DIV_N));
  }
  glEnd();
  // arrow base
  float arrow_base_len_rad = 0.01;
  int K = 20;
  const float TAU_DIV_K = 2 * (float)M_PI / K;
  glBegin(GL_QUAD_STRIP);
  for (int i = 0; i < K * 2; i += 2) {
    glVertex3f(arrow_len - arrow_head_len,
               arrow_base_len_rad * cos(i * TAU_DIV_K),
               arrow_base_len_rad * sin(i * TAU_DIV_K));

    glVertex3f(0, arrow_base_len_rad * cos(i * TAU_DIV_K),
               arrow_base_len_rad * sin(i * TAU_DIV_K));
  }

  glEnd();

  pangolin::glUnsetFrameOfReference();
  // Reset reference frame
  pangolin::glUnsetFrameOfReference();
  glLineWidth(1.0);
  glColor3f(1.0, 1.0, 1.0);

  return RET_SUCCESS_INT;
}

int TrajectoryObject::set_gl_cam_relative(float rel_x, float rel_y, float rel_z,
                                          pangolin::AxisDirection up) {
  if (cam_ != NULL)
    cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(
        rel_x, rel_y, rel_z, pose_(0, 0), pose_(0, 1), pose_(0, 2), up));
  return RET_SUCCESS_INT;
}

int TrajectoryObject::set_pose(const pangolin::OpenGlMatrix &pose) {
  pose_ = pose;

  traj_.push_back((float)pose(0, 3));
  traj_.push_back((float)pose(1, 3));
  traj_.push_back((float)pose(2, 3));

  if ((traj_.size() / 3) > keep_) {
    traj_.erase(traj_.begin());
    traj_.erase(traj_.begin());
    traj_.erase(traj_.begin());
  }
  return RET_SUCCESS_INT;
}

int TrajectoryObject::set_pose(double x, double y, double z) {
  traj_.push_back(x);
  traj_.push_back(y);
  traj_.push_back(z);
  return RET_SUCCESS_INT;
}

#ifdef UTILS_USE_OPENCV
int TrajectoryObject::set_pose(const cv::Mat &rot, const cv::Mat &trans) {
  if (rot.channels() != 1 || trans.channels() != 1) {
    PI$ "rotation or translation matrix don't have 1 channel." pendl;
    return 1;
  }
#ifdef HAVE_GLES
  // GLprecision will be float.
  cv::Mat rotc, transc; // rotation and translation converted.
  if (rot.depth() != CV_32F)
    rot.convertTo(rotc, CV_32F);
  else
    rotc = rot;
  if (trans.depth() != CV_32F)
    trans.convertTo(transc, CV_32F);
  else
    transc = trans;
#else
  // GLprecision will be double.
  cv::Mat rotc, transc; // rotation and translation converted.
  if (rot.depth() != CV_64F)
    rot.convertTo(rotc, CV_64F);
  else
    rotc = rot;
  if (trans.depth() != CV_64F)
    trans.convertTo(transc, CV_64F);
  else
    transc = trans;
#endif

  pangolin::OpenGlMatrix pose;

  /* rotation */
  pose(0, 0) = rotc.at<pangolin::GLprecision>(0, 0);
  pose(0, 1) = rotc.at<pangolin::GLprecision>(0, 1);
  pose(0, 2) = rotc.at<pangolin::GLprecision>(0, 2);
  pose(1, 0) = rotc.at<pangolin::GLprecision>(1, 0);
  pose(1, 1) = rotc.at<pangolin::GLprecision>(1, 1);
  pose(1, 2) = rotc.at<pangolin::GLprecision>(1, 2);
  pose(2, 0) = rotc.at<pangolin::GLprecision>(2, 0);
  pose(2, 1) = rotc.at<pangolin::GLprecision>(2, 1);
  pose(2, 2) = rotc.at<pangolin::GLprecision>(2, 2);

  /* translation */
  pose(0, 3) = transc.at<pangolin::GLprecision>(0);
  pose(1, 3) = transc.at<pangolin::GLprecision>(1);
  pose(2, 3) = transc.at<pangolin::GLprecision>(2);

  /* last row 0 0 0 1 */
  pose(3, 0) = 0.0;
  pose(3, 1) = 0.0;
  pose(3, 2) = 0.0;
  pose(3, 3) = 1.0;

  CHECK_RET(set_pose(pose));
}
#endif

int TrajectoryObject::follow() {
  if (!followed_ && cam_ != NULL) {
    cam_->Follow(pose_);
    followed_ = true;
  }
  return RET_SUCCESS_INT;
}

int TrajectoryObject::unfollow() {
  if (followed_ && cam_ != NULL) {
    cam_->Unfollow();
    followed_ = false;
  }
  return RET_SUCCESS_INT;
}
#ifdef BUILT_WITH_ROS

int TrajectoryObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                             uint32_t queue_sz, size_t keep, bool follow) {
  sub_ = n.subscribe(topic_sub, queue_sz, &TrajectoryObject::callback, this);
  CHECK_RET(reinit(keep, follow));
  return RET_SUCCESS_INT;
}

void TrajectoryObject::callback(const nav_msgs::Odometry::ConstPtr &odom) {
  geometry_msgs::Pose pose = odom->pose.pose;
  pangolin::OpenGlMatrix ogm_pose;

  pangolin::OpenGlMatrix ogm_pose_ref_frame;
  ogm_pose_ref_frame.SetIdentity();

  tf::Matrix3x3 tm(tf::Quaternion(pose.orientation.x, pose.orientation.y,
                                  pose.orientation.z, pose.orientation.w));
  tf::StampedTransform transform_in_map;
  static tf::TransformListener listener;
  tf::Vector3 origin;
  origin[0] = 0;
  origin[1] = 0;
  origin[2] = 0;

  // transform odometry data to "utm" frame.
  std::string frm = odom->child_frame_id;
  frm = odom->header.frame_id;
  std::string target_frame = "utm";
  target_frame = "utm";

  target_frame = target_frame_;

  geometry_msgs::PoseStamped pose_in;              // = pose;
  geometry_msgs::QuaternionStamped orientation_in; // = pose.orientation;
  pose_in.pose = pose;
  pose_in.header = odom->header;
  orientation_in.header = odom->header;
  orientation_in.quaternion.x = pose.orientation.x;
  orientation_in.quaternion.y = pose.orientation.y;
  orientation_in.quaternion.z = pose.orientation.z;
  orientation_in.quaternion.w = pose.orientation.w;

  geometry_msgs::PoseStamped pose_out;
  geometry_msgs::QuaternionStamped orientation_out;

  tm = tf::Matrix3x3(
      tf::Quaternion(orientation_in.quaternion.x, orientation_in.quaternion.y,
                     orientation_in.quaternion.z, orientation_in.quaternion.w));
  pose = pose_in.pose;

  if (!target_frame.empty()) {
    try {
#if 0
			// Transform input odometry's pose and orientation to the given "target_frame_" frame.
			listener.transformPose(target_frame,pose_in,
					pose_out);
			listener.transformQuaternion(target_frame,orientation_in,
					orientation_out);

			tm = tf::Matrix3x3( tf::Quaternion( orientation_out.quaternion.x,orientation_out.quaternion.y,
						orientation_out.quaternion.z,orientation_out.quaternion.w));
			pose = pose_out.pose;
#else

      // Find the transformation between current frame "frm" and target frame
      // "target_frame_",
      // set this transformation as reference frame "ref_frame_" for the current
      // TrajectoryObject,
      // (for plotting this odometry values).

      listener.lookupTransform(target_frame, frm, ros::Time(0),
                               transform_in_map);

      tf::Matrix3x3 m33_1 = transform_in_map.getBasis();
      tf::Vector3 origin1 = transform_in_map.getOrigin();

      for (int i = 0; i < 3; i++) {
        ogm_pose_ref_frame(i, 3) = 0;
        for (int j = 0; j < 3; j++)
          ogm_pose_ref_frame(i, j) = m33_1[i][j];
      }

      ogm_pose_ref_frame(0, 3) = origin1[0];
      ogm_pose_ref_frame(1, 3) = origin1[1];
      ogm_pose_ref_frame(2, 3) = origin1[2];
      ogm_pose_ref_frame(3, 0) = 0.0, ogm_pose_ref_frame(3, 1) = 0.0,
                            ogm_pose_ref_frame(3, 2) = 0.0,
                            ogm_pose_ref_frame(3, 3) = 1.0;

      ref_frame_ = ogm_pose_ref_frame;
#endif

    } catch (tf::TransformException &exception) {
      ROS_ERROR("Unable to tranform %s to %s frame : %s", frm.c_str(),
                target_frame.c_str(), exception.what());
      // return;
    }
  } else {
  }

#if 0
	// Find transformation from utm to utm_min_bound
	// Since utm values are usually large, osm_to_viz node publishes utm_min_bound transform
	// which is basically left-bottom bound of utm values read from .osm map file.
	// We subtract the utm_min_bound transform from the points in utm frame.

	try {

		listener.lookupTransform("utm", "utm_min_bound", ros::Time(0), transform_in_map);

		tf::Matrix3x3 m33 = transform_in_map.getBasis();
		origin = transform_in_map.getOrigin();


	} 
	catch(tf::TransformException &exception) 
	{
		ROS_ERROR("Error finding transform between utm and utm_min_bound frames : %s", exception.what());
		return;
	}
#endif

  // fill ogm_pose for trajector to draw
  for (int i = 0; i < 3; i++) {
    ogm_pose(i, 3) = 0;
    for (int j = 0; j < 3; j++)
      ogm_pose(i, j) = tm[i][j];
  }

  ogm_pose(0, 3) = pose.position.x;
  ogm_pose(1, 3) = pose.position.y;
  ogm_pose(2, 3) = pose.position.z;
  ogm_pose(3, 0) = 0.0, ogm_pose(3, 1) = 0.0, ogm_pose(3, 2) = 0.0,
              ogm_pose(3, 3) = 1.0;
  // subtract origin (utm min bound)
  ogm_pose(0, 3) -= origin[0];
  ogm_pose(1, 3) -= origin[1];
  ogm_pose(2, 3) -= origin[2];

  // update trajectory pose
  set_pose(ogm_pose);
}
#endif

/*--------------------------TrajectoryObject End------------------------------*/

/*--------------------------PTSObject Start-----------------------------------*/

int PTSObject::reinit(const std::string &file, utils::Color default_col,
                      double scale, int skip) {
  try {

    int sz = read_PTS_file(file, pts_, col_, default_col, scale, skip);
    if (!sz) // num of points is zero
      return false;

    ptsb_.Reinitialise(pangolin::GlArrayBuffer, sz, GL_DOUBLE, VBO_ELEMS,
                       GL_DYNAMIC_DRAW);
    colb_.Reinitialise(pangolin::GlArrayBuffer, sz, GL_UNSIGNED_BYTE, VBO_ELEMS,
                       GL_DYNAMIC_DRAW);
    CHECK_GL_ERROR("While reinitialising buffer");

    ptsb_.Upload((double *)pts_, VBO_ELEMS * sz * sizeof(double));
    colb_.Upload((unsigned char *)col_, VBO_ELEMS * sz * sizeof(unsigned char));

    CHECK_GL_ERROR("While uploading points to buffer.");

  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int PTSObject::read_PTS_file(const std::string &file, double *&points,
                             unsigned char *&col, utils::Color default_col,
                             double scale, int skip) {
  // increase by one as all the operation would be modulo of skip+1, if you
  // want to skip 2 elements, the line_no%3 == 0 is what you read, also when
  // you don't want to skip anything, skip=0 doesn't work as we divide by skip
  // to calculate num of points.
  skip++;
  int sz;
  vec_str lines = utils::read_file(file);
  if (!lines.size()) {
    PE$ "No lines in file " << ct_ylw(file) pendl;
    points = NULL;
    col = NULL;
    return RET_SUCCESS_INT;
  }
  sz = utils::str_to_val<int>(lines[0]);
  // sz + 1 because the line with number of points is still there.
  if (lines.size() != (size_t)(sz + 1)) {
    PE$ "No. of lines are not same as number of expected points in file "
        << ct_ylw(file) pendl;
    points = NULL;
    col = NULL;
    return RET_SUCCESS_INT;
  }

  sz = sz / skip;
  lines.erase(lines.begin()); // erase first line, containing num of pts.

  // 3 because PTS will contain x,y,x and r,g,b it doesn't have alpha.
  points = new double[3 * sz];
  col = new unsigned char[3 * sz];

  int pts_count = -1, col_count = -1, line_count = 0;
  for (std::string line : lines) {
    if (line_count % skip != 0) {
      line_count++;
      continue;
    }
    vec_str tokens = utils::split_str<std::string>(line, " ");
    if (tokens.size() >= 3) {
      points[++pts_count] = scale * utils::str_to_val<double>(tokens[0]);
      points[++pts_count] = scale * utils::str_to_val<double>(tokens[1]);
      points[++pts_count] = scale * utils::str_to_val<double>(tokens[2]);
    }
    // ignoring tokens[3]
    if (tokens.size() == 7) {
      col[++col_count] = utils::str_to_val<double>(tokens[4]);
      col[++col_count] = utils::str_to_val<double>(tokens[5]);
      col[++col_count] = utils::str_to_val<double>(tokens[6]);
    } else {
      float *c = default_col;
      col[++col_count] = (unsigned char)(c[0] * 255.0f);
      col[++col_count] = (unsigned char)(c[1] * 255.0f);
      col[++col_count] = (unsigned char)(c[2] * 255.0f);
    }
    line_count++;
  }
  PI$ ct_vgrn(line_count) pcommas ct_vgrn(sz) pendl;
  if (line_count / skip == sz) {
    return sz;
  }

  PE$ "Something went wrong wil parsing file and processing file "
      << ct_ylw(file) << " for loop ran " << ct_dylw(line_count) << "  times "
      << " and expected number of points are " << ct_dylw(sz) pendl;
  points = NULL;
  col = NULL;
  return RET_SUCCESS_INT;
}

int PTSObject::draw_obj(pangolin::View & /*v*/) {
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  pangolin::RenderVboCbo(ptsb_, colb_);
  CHECK_GL_ERROR("");
  return RET_SUCCESS_INT;
}

/*--------------------------PTSObject End-------------------------------------*/

/*--------------------------PointCloudObject Start----------------------------*/

int PointCloudObject::reinit(size_t num_of_frames, utils::Color col) {
  try {
    curr_idx_ = 0;
    col_ = col;

    ptsb_ = NULL;
    colb_ = NULL;
    show_ = NULL;
    cb_valid_ = NULL;

    CHECK_RET(resize(num_of_frames));
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int PointCloudObject::resize(size_t num_frames) {
  try {
    if (num_frames < 1)
      num_frames = 1;

    num_frames_ = num_frames;

    // reinit buffers
    if (ptsb_ != NULL)
      delete ptsb_;
    if (colb_ != NULL)
      delete colb_;
    ptsb_ = new pangolin::GlBuffer[num_frames_];
    colb_ = new pangolin::GlBuffer[num_frames_];
    show_ = new bool[num_frames_];
    cb_valid_ = new bool[num_frames_];

    for (size_t idx = 0; idx < num_frames_; ++idx) {
      show_[idx] = false;
    }
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int PointCloudObject::draw_obj(pangolin::View & /*v*/) {
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  for (size_t idx = 0; idx < num_frames_; ++idx) {
    if (show_[idx]) {
      if (cb_valid_[idx]) {
        pangolin::RenderVboCbo(ptsb_[idx], colb_[idx]);
      } else {
        glColor3fv(col_);
        pangolin::RenderVbo(ptsb_[idx]);
        glColor3f(1.0f, 1.0f, 1.0f);
      }
    }
    CHECK_GL_ERROR(ct_vred(idx));
  }
  return RET_SUCCESS_INT;
}

int PointCloudObject::erase(int idx) {
  show_[idx] = false;
  return RET_SUCCESS_INT;
}

int PointCloudObject::erase_last() {
  CHECK_RET(erase(prev_idx()));
  return RET_SUCCESS_INT;
}

int PointCloudObject::update(size_t num_pts, double *pts, unsigned char *col) {
  pangolin::GlBuffer *pbuf = ptsb_ + curr_idx_;
  pangolin::GlBuffer *cbuf = colb_ + curr_idx_;

  // TODO: resize
  if (!pbuf->IsValid() || pbuf->num_elements != num_pts ||
      cbuf->num_elements != num_pts || (col != NULL && !cb_valid_)) {
    reinit_buf(curr_idx_, num_pts, (col != NULL));
  }

  pbuf->Upload((double *)pts, VBO_ELEMS * num_pts * sizeof(double));
  if (col == NULL) {
    cb_valid_[curr_idx_] = false;
  } else
    cbuf->Upload((unsigned char *)col,
                 VBO_ELEMS * num_pts * sizeof(unsigned char));

  curr_idx_ = next_idx();

  return RET_SUCCESS_INT;
}
/* function overloading for float type - future use */
int PointCloudObject::update(size_t num_pts, float *pts, unsigned char *col) {
  pangolin::GlBuffer *pbuf = ptsb_ + curr_idx_;
  pangolin::GlBuffer *cbuf = colb_ + curr_idx_;

  // TODO: resize
  if (!pbuf->IsValid() || pbuf->num_elements != num_pts ||
      cbuf->num_elements != num_pts || (col != NULL && !cb_valid_)) {
    reinit_buf(curr_idx_, num_pts, (col != NULL));
  }

  pbuf->Upload((float *)pts, VBO_ELEMS * num_pts * sizeof(float));
  if (col == NULL) {
    cb_valid_[curr_idx_] = false;
  } else
    cbuf->Upload((unsigned char *)col,
                 VBO_ELEMS * num_pts * sizeof(unsigned char));

  //  curr_idx_ = next_idx();

  CHECK_RET_TO_VAR(next_idx(), curr_idx_);
  return RET_SUCCESS_INT;
}

int PointCloudObject::reinit_buf(int idx, size_t sz, bool color) {
  try {

    if (ptsb_[idx].num_elements != sz) {
      ptsb_[idx].Reinitialise(pangolin::GlArrayBuffer, sz, GL_DOUBLE, VBO_ELEMS,
                              GL_DYNAMIC_DRAW);
    }
    if (color && colb_[idx].num_elements != sz) {
      colb_[idx].Reinitialise(pangolin::GlArrayBuffer, sz, GL_UNSIGNED_BYTE,
                              VBO_ELEMS, GL_DYNAMIC_DRAW);
    }
    cb_valid_[idx] = color;

    show_[idx] = true;
    CHECK_GL_ERROR("while reiniting buf with index " << ct_red(idx));
  } catch (std::exception e) {
    PE$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

inline int PointCloudObject::next_idx() {
  int idx = curr_idx_ + 1;
  return (idx >= (int)num_frames_) ? 0 : idx;
}

inline int PointCloudObject::prev_idx() {
  int idx = curr_idx_ - 1;
  return (idx < 0) ? (num_frames_ - 1) : idx;
}

#ifdef BUILT_WITH_ROS

int PointCloudObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                             uint32_t queue_sz, size_t num_of_frames,
                             utils::Color col) {
  sub_ = n.subscribe(topic_sub, queue_sz, &PointCloudObject::callback, this);
  CHECK_RET(reinit(num_of_frames, col));
  return RET_SUCCESS_INT;
}

void PointCloudObject::callback(const sensor_msgs::PointCloud2 &pc_sub) {
  const int sn = 147;

  int point_offset = pc_sub.point_step;
  int x_offset = -1, x_field_idx = -1;
  int y_offset = -1, y_field_idx = -1;
  int z_offset = -1, z_field_idx = -1;
  int col_offset = -1, col_field_idx = -1;
  // int intensity_offset = -1, intensity_field_idx = -1;//might be needed later

  int num_bytes = (int)pc_sub.data.size();
  int num_pts = num_bytes / point_offset;

  if (NULL == pts_) {
    // since we are assuming points to contain X,Y,Z
    pts_ = new double[num_pts * VBO_ELEMS];
    pc_count_ = num_pts;
  }
  if (NULL == c_pts_)
    c_pts_ = new unsigned char[num_pts * CBO_ELEMS];

  for (unsigned int i = 0; i < pc_sub.fields.size(); ++i) {
    const std::string &name = pc_sub.fields[i].name;
    if (name == "x" || name == "X") {
      if (pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT32 &&
          pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT64) {
        PE$ "Support for Field X in PointCloud2 message is limited to float32 "
            "and float 64, when current field datatype is "
            << pc_sub.fields[i].datatype pendl;
        PN$ ct_red("Returning") pendl;
        return;
      }
      x_offset = pc_sub.fields[i].offset;
      x_field_idx = i;
    } else if (name == "y" || name == "Y") {
      if (pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT32 &&
          pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT64) {
        PE$ "Support for Field Y in PointCloud2 message is limited to float32 "
            "and float 64, when current field datatype is "
            << pc_sub.fields[i].datatype pendl;
        PN$ ct_red("Returning") pendl;
        return;
      }
      y_offset = pc_sub.fields[i].offset;
      y_field_idx = i;
    } else if (name == "z" || name == "Z") {
      if (pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT32 &&
          pc_sub.fields[i].datatype != sensor_msgs::PointField::FLOAT64) {
        PE$ "Support for Field Z in PointCloud2 message is limited to float32 "
            "and float 64, when current field datatype is "
            << (int)pc_sub.fields[i].datatype pendl;
        PN$ ct_red("Returning") pendl;
        return;
      }
      z_offset = pc_sub.fields[i].offset;
      z_field_idx = i;
    }
    // should also include handling of xyz and XYZ label
    else if (name == "rgb" || name == "RGB" || name == "Rgb" ||
             name == "rgba" || name == "RGBA" || name == "Rgba") {
      col_offset = pc_sub.fields[i].offset;
      col_field_idx = i;
    }
    // As of now we ignore intensity and assign single color if RGB is not
    // there.
    // else if (name == "intensity" || name == "I") {
    // intensity_offset = pc_sub.fields[i].offset;
    // intensity_field_idx = i;
    //}
  }

  const unsigned char *pts_ptr = pc_sub.data.data();
  SP$(sn) ct_vpur(pts_ptr) pendl;

  for (int i = 0, j = 0; i < num_bytes; i += point_offset, j++) {
    // since i is being incremented by point_offset, i is equivalent to
    // incrementing i by 1 and multiplying i by point_offset.
    const unsigned char *pt = pts_ptr + i; // i * point_offset;
    SP$(sn) ct_vpur(i) pcommas ct_vpur(point_offset) pcommas ct_vpur(pt) pendl;

    // Processing x.
    if (x_offset >= 0 && x_field_idx >= 0) {
      // In i*VBO_ELEMS+0, 0 is considered for X as opengl takes in xyz order,
      // which if soeday changes cand be handled with couple of variables.
      SP$(sn) ct_vpur(x_offset) pcommas ct_vpur(pt + x_offset) pendl;
      CHECK_RET_VOID(this->get_pt_using_field_datatype(
          pt + x_offset, pc_sub.fields[x_field_idx].datatype,
          pts_ + (j * VBO_ELEMS + 0)));
    }
    // Processing y.
    if (y_offset >= 0 && y_field_idx >= 0) {
      // In i*VBO_ELEMS+1, 1 is considered for <read reason in X>.
      SP$(sn) ct_vpur(y_offset) pcommas ct_vpur(pt + y_offset) pendl;
      CHECK_RET_VOID(this->get_pt_using_field_datatype(
          pt + y_offset, pc_sub.fields[y_field_idx].datatype,
          pts_ + (j * VBO_ELEMS + 1)));
    }
    // Processing z.
    if (z_offset >= 0 && z_field_idx >= 0) {
      // In i*VBO_ELEMS+2, 2 is considered for <read reason in X>.
      SP$(sn) ct_vpur(z_offset) pcommas ct_vpur(pt + z_offset) pendl;
      CHECK_RET_VOID(this->get_pt_using_field_datatype(
          pt + z_offset, pc_sub.fields[z_field_idx].datatype,
          pts_ + (j * VBO_ELEMS + 2)));
    }

    if (col_offset >= 0 && col_field_idx >= 0) {
      c_pts_[j * CBO_ELEMS] = pt[col_offset];         // r
      c_pts_[j * CBO_ELEMS + 1] = pt[col_offset + 1]; // g
      c_pts_[j * CBO_ELEMS + 2] = pt[col_offset + 2]; // b
      // To incorporate alpha from RGB, CBO_ELEMS would need to be made a
      // variable, will do that when need arrises.
    } else {
      // TODO: jetmaps?
      c_pts_ = NULL;
    }
  }
  update(num_pts, pts_, c_pts_);
}

int PointCloudObject::save_pointCloud(const std::string &filename) {
  try {
    std::time_t t = std::time(NULL);
    char str[100];
    std::strftime(str, sizeof(str), "%Y%m%d_%H%M%S", std::localtime(&t));
    std::string fullfilename;
    fullfilename.append(filename);
    fullfilename.append("_");
    fullfilename.append(str);

#if 0
/* print the point cloud in text format 
	first line contains - no. of points in point cloud in integer format
	next each line contains 3 double points represents x, y, z values and
	3 integer points represents r, g, b values all are delimeted by "," operator
	sample point cloud file contents: 
	768000
	nan,    nan,    nan,    36,     36,     36,
	nan,    nan,    nan,    32,     32,     32,
	nan,    nan,    nan,    30,     30,     30,
	nan,    nan,    nan,    29,     29,     29,	
	-2.38011,       -0.925979,      18.5841,        35,     35,     35,
	-2.36758,       -0.924443,      18.5533,        31,     31,     31,
	-2.35509,       -0.922913,      18.5226,        29,     29,     29,
	-2.34653,       -0.922913,      18.5226,        26,     26,     26,	
*/
		fullfilename.append(".txt");
		std::ofstream ofs(fullfilename.c_str());
		if(!ofs.is_open()) {
			PF$(100) "Couldn't open the file \"" << ct_red(fullfilename) << "\"." pendl;
			return;
		}     
		ofs << pc_count_ << "\n";
		for(int i=0; i<pc_count_; i++)
		{
			for(int c=0;c<3;c++)
			{
				ofs << pts_[i*3 + c] << ",\t";
			}
			for(int c=0;c<3;c++)
			{
				ofs << (int)c_pts_[i*3 + c]  << ",\t";
			}
			ofs << "\n";
		}
#else
    /* print the point cloud in binary format
            first 4 bytes (sizeof(int))- no. of points in point cloud
            remaining all bytes follows the below order:
            next 3*(sizeof(double)) represents x y z points
            next 3*(sizeof(int)) represents r g b color
    */
    fullfilename.append(".bin");
    std::ofstream ofs(fullfilename.c_str(), std::ios::binary);

    if (!ofs.is_open()) {
      PF$(100)
          "Couldn't open the file \"" << ct_red(fullfilename) << "\"." pendl;
      return RET_CANNOT_OPEN_FILE_INT;
    }
    ofs.write((char *)&pc_count_, sizeof(int));
    for (int i = 0; i < pc_count_; i++) {
      ofs.write((char *)&pts_[i * 3], 3 * sizeof(double));
      ofs.write((char *)&c_pts_[i * 3], 3 * sizeof(unsigned char));
    }


#endif
    DP$ " point cloud saved " pendl;
    ofs.close();
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

#endif

/*--------------------------PointCloudObject End------------------------------*/
/*--------------------------VisualizationMarkerArrayObject
 * Begin------------------------------*/

int VisualizationMarkerArrayObject::reinit(uint32_t keep) {
  cur_id = -1;
  n_keep = keep;
  marker_array_vec.clear();

  ref_frame_.SetIdentity();

  target_frame_ = "map";
  for (int i = 0; i < n_keep; i++) {
    marker_array_vec.push_back(visualization_msgs::MarkerArray());
  }

  return RET_SUCCESS_INT;
}
int VisualizationMarkerArrayObject::update(
    const visualization_msgs::MarkerArray &viz_marker_array)

{
  uint32_t next_id = get_next_id();
  visualization_msgs::MarkerArray &ma = marker_array_vec[next_id];
  ma.markers.clear();
  ma.markers.insert(ma.markers.end(), viz_marker_array.markers.begin(),
                    viz_marker_array.markers.end());

  offset_x = 0;
  offset_y = 0;
  offset_z = 0; // currently unused.

#if 1
  tf::StampedTransform transform_in_map;
  static tf::TransformListener listener;
  tf::Vector3 origin;
  origin[0] = 0;
  origin[1] = 0;
  origin[2] = 0;

  pangolin::OpenGlMatrix ogm_pose_ref_frame;
  ogm_pose_ref_frame.SetIdentity();

  // Find transformation from utm to utm_min_bound
  // Since utm values are usually large, osm_to_viz node publishes utm_min_bound
  // transform
  // which is basically left-bottom bound of utm values read from .osm map file.
  // We subtract the utm_min_bound transform from the points in utm frame.

  std::string frm = "map";
  std::string target_frame = "utm";
  try {

    frm = "utm";
    target_frame = "utm_min_bound";
    target_frame = "map";

    if (!target_frame_.empty())
      target_frame = target_frame_;

    listener.lookupTransform(target_frame, frm, ros::Time(0), transform_in_map);
    origin = transform_in_map.getOrigin();

// offset_x = origin[0];
// offset_y = origin[1];

#if 1
    tf::Matrix3x3 m33_1 = transform_in_map.getBasis();
    tf::Vector3 origin1 = transform_in_map.getOrigin();

    for (int i = 0; i < 3; i++) {
      ogm_pose_ref_frame(i, 3) = 0;
      for (int j = 0; j < 3; j++)
        ogm_pose_ref_frame(i, j) = m33_1[i][j];
    }

    ogm_pose_ref_frame(0, 3) = origin1[0];
    ogm_pose_ref_frame(1, 3) = origin1[1];
    ogm_pose_ref_frame(2, 3) = origin1[2];
    ogm_pose_ref_frame(3, 0) = 0.0, ogm_pose_ref_frame(3, 1) = 0.0,
                          ogm_pose_ref_frame(3, 2) = 0.0,
                          ogm_pose_ref_frame(3, 3) = 1.0;

    ref_frame_ = ogm_pose_ref_frame;
#endif

  } catch (tf::TransformException &exception) {
    ROS_ERROR("Error finding transform between %s and %s frames : %s",
              frm.c_str(), target_frame.c_str(), exception.what());
    return RET_EXCEPTION_INT;
  }
#endif

  return RET_SUCCESS_INT;
}

int VisualizationMarkerArrayObject::draw_obj(pangolin::View &v) {

  glEnable(GL_DEPTH_TEST);

  glDisable(GL_LIGHTING);
  Eigen::Matrix4f ref_frame_pos = ref_frame_;
  pangolin::glSetFrameOfReference(ref_frame_pos);
  // const float offset_x = 218949; // 217553.980702;
  // const float offset_y = 1931284; //1930307.09198;
  for (int m = 0; m < marker_array_vec.size(); m++) {
    visualization_msgs::MarkerArray &marker_array = marker_array_vec[m];
    for (int i = 0; i < marker_array.markers.size(); i++) {
      visualization_msgs::Marker &marker = marker_array.markers[i];

      if (marker.type == visualization_msgs::Marker::LINE_STRIP) {
        std_msgs::ColorRGBA color = marker.color;
        int n_points = marker.points.size();
        glColor4f(color.r, color.g, color.b, color.a);
        if (n_points > 0) {
          glLineWidth(marker.scale.x);
          glBegin(GL_LINES);
          for (int k = 0; k < n_points; k += 2) {
            geometry_msgs::Point pt = marker.points[k];
            geometry_msgs::Point pt1 = marker.points[k + 1];
            pt.x -= offset_x;
            pt.y -= offset_y;
            pt1.x -= offset_x;
            pt1.y -= offset_y;
            glVertex3f(pt.x, pt.y, pt.z);
            glVertex3f(pt1.x, pt1.y, pt1.z);
          }
          glEnd();
          glLineWidth(1.0);
        }
        glColor4f(1.0, 1.0, 1.0, 1.0);

      } else if (visualization_msgs::Marker::CYLINDER == marker.type) {
        geometry_msgs::Point pt = marker.pose.position;
        pt.x -= offset_x;
        pt.y -= offset_y;
        std_msgs::ColorRGBA color = marker.color;
        glColor4f(color.r, color.g, color.b, color.a);
        glPointSize(2);
        glBegin(GL_POINTS);
        glVertex3f(pt.x, pt.y, pt.z);
        glEnd();

        glColor4f(1.0, 1.0, 1.0, 1.0);
        glPointSize(1);
      }
    }
  }
  pangolin::glUnsetFrameOfReference();
  return RET_SUCCESS_INT;
}

#ifdef BUILT_WITH_ROS
int VisualizationMarkerArrayObject::reinit(const std::string &topic_sub,
                                           ros::NodeHandle &n,
                                           uint32_t queue_sz, uint32_t keep) {
  sub_ = n.subscribe(topic_sub, queue_sz,
                     &VisualizationMarkerArrayObject::callback, this);
  CHECK_RET(reinit(keep));
  return RET_SUCCESS_INT;
}

void VisualizationMarkerArrayObject::callback(
    const visualization_msgs::MarkerArray &viz_marker_array) {
  update(viz_marker_array);
}
#endif
/*--------------------------VisualizationMarkerArrayObject
 * End------------------------------*/
/*--------------------------GPSObject Start------------------------------*/

int GPSObject::reinit(size_t keep, bool follow) {
  try {
    //  pose_.SetIdentity();
    keep_ = keep;
    traj_pts_col_ = utils::Color(1.0, 1.0, 1.0);
    traj_line_col_ = utils::Color(1.0, 0.0, 1.0);
    follow_ = follow;
  } catch (std::exception e) {
    DP$ ct_vpur(e.what()) pendl;
    return RET_EXCEPTION_INT;
  }
  return RET_SUCCESS_INT;
}

int GPSObject::draw_obj(pangolin::View & /*v*/) {

  //  follow();
  if (cam_ != NULL)
    cam_->Apply();
  GLfloat model[16];
  glGetFloatv(GL_MODELVIEW_MATRIX, model);

  glEnable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  /* draw points */
  glPointSize(2 * traj_size_);
  glColor3fv(traj_pts_col_);
  glBegin(GL_POINTS);
  for (size_t i = 0; i < traj_.size(); i += 3) {
    glVertex3fv(&traj_[i]);
  }
  glEnd();

  /* draw line */
  glLineWidth(traj_size_);
  glColor3fv(traj_line_col_);
  glBegin(GL_LINES);
  for (size_t i = 3; i < traj_.size(); i += 3) {
    glVertex3fv(&traj_[i - 3]);
    glVertex3fv(&traj_[i]);
  }
  glEnd();

  glLineWidth(1.0);
  glColor3f(1.0, 1.0, 1.0);

  return RET_SUCCESS_INT;
}

int GPSObject::set_gl_cam_relative(float rel_x, float rel_y, float rel_z,
                                   pangolin::AxisDirection up) {
  if (cam_ != NULL)
    cam_->SetModelViewMatrix(pangolin::ModelViewLookAt(
        rel_x, rel_y, rel_z, pose_(0, 0), pose_(0, 1), pose_(0, 2), up));
  return RET_SUCCESS_INT;
}

int GPSObject::set_pose(double x, double y, double z) {
  //  pose_ = pose;

  traj_.push_back((double)x);
  traj_.push_back((double)y);
  traj_.push_back((double)z);

  if ((traj_.size() / 3) > keep_) {
    traj_.erase(traj_.begin());
    traj_.erase(traj_.begin());
    traj_.erase(traj_.begin());
  }
  return RET_SUCCESS_INT;
}

#ifdef BUILT_WITH_ROS

int GPSObject::reinit(const std::string &topic_sub, ros::NodeHandle &n,
                      uint32_t queue_sz, size_t keep, bool follow) {
  sub_ = n.subscribe(topic_sub, queue_sz, &GPSObject::callback, this);
  CHECK_RET(reinit(keep, follow));
  return RET_SUCCESS_INT;
}

void GPSObject::callback(const sensor_msgs::NavSatFixConstPtr &fix) {

  tf::StampedTransform transform_in_map;
  static tf::TransformListener listener;
  tf::Vector3 origin;
  origin[0] = 0;
  origin[1] = 0;
  origin[2] = 0;

  std::string frame_id, child_frame_id;
  double rot_cov;

  if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
    ROS_INFO("No fix.");
    return;
  }

  if (fix->header.stamp == ros::Time(0)) {
    return;
  }

  double northing, easting;
  std::string zone;

  LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);

  nav_msgs::Odometry odom;
  odom.header.stamp = fix->header.stamp;

  if (frame_id.empty())
    odom.header.frame_id = fix->header.frame_id;
  else
    odom.header.frame_id = frame_id;

  odom.child_frame_id = child_frame_id;

  odom.pose.pose.position.x = easting;
  odom.pose.pose.position.y = northing;
  odom.pose.pose.position.z = fix->altitude;

  odom.pose.pose.orientation.x = 0;
  odom.pose.pose.orientation.y = 0;
  odom.pose.pose.orientation.z = 0;
  odom.pose.pose.orientation.w = 1;

  // Use ENU covariance to build XYZRPY covariance
  boost::array<double, 36> covariance = {{fix->position_covariance[0],
                                          fix->position_covariance[1],
                                          fix->position_covariance[2],
                                          0,
                                          0,
                                          0,
                                          fix->position_covariance[3],
                                          fix->position_covariance[4],
                                          fix->position_covariance[5],
                                          0,
                                          0,
                                          0,
                                          fix->position_covariance[6],
                                          fix->position_covariance[7],
                                          fix->position_covariance[8],
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          rot_cov,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          rot_cov,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          0,
                                          rot_cov}};

  odom.pose.covariance = covariance;

  try {

    listener.lookupTransform("utm", "utm_min_bound", ros::Time(0),
                             transform_in_map);

    tf::Matrix3x3 m33 = transform_in_map.getBasis();
    origin = transform_in_map.getOrigin();

  } catch (tf::TransformException &exception) {
    ROS_ERROR("GPSObject Error finding transform between utm and utm_min_bound "
              "frames : %s",
              exception.what());
    return;
  }

  double utm_x = odom.pose.pose.position.x - origin[0];
  double utm_y = odom.pose.pose.position.y - origin[1];
  double utm_z = odom.pose.pose.position.z - origin[2];

  /*

  std::cout << "x:		 " <<
                  utm_x << " y:		 " <<
  utm_y << "z:		 " <<
  utm_z << std::endl;

  */
  set_pose(utm_x, utm_y, 0);
}
#endif

/*--------------------------GPSObject End------------------------------*/

/*--------------------------OSMObject Start------------------------------*/

int OSMObject::reinit(char *sample1) {
  if (!strlen(sample1)) {
    PE$ "no path is specified" pendl;
    return RET_INVALID_PATH;
  } else {
    PN$ "loading the file in the path: " << sample1 pendl;
    CHECK_RET(load_osm(sample1));
  }
  return RET_SUCCESS_INT;
}

int OSMObject::load_osm(char *sample) {
  PRINT_FUNC_ENTER;
  const int sn = 333;
  typedef struct {
    unsigned int dup_way_id;
    std::vector<unsigned int> dup_way_nd_ref;
    std::string tag;
    std::string tag_value;
  } way;

  typedef struct {
    std::vector<unsigned int> node_id;
    std::vector<double> node_lat;
    std::vector<double> node_long;
  } node;

  FILE *fp;
  node node_data;
  std::vector<way> dup_way;

  /*
  if(flag == 0)
  {
  sample = "/home/chaitanya/catkin_ws2/src/utils/osm_files/78.3/17/17.42.osm";
  }
  */

  double time;
  Tic();

  fp = fopen(sample, "r");
  if (fp == NULL) {
    PE$ "file doesn't exist in the given path" pendl;
    return RET_INVALID_FILE;
  }

  tinyxml2::XMLDocument osm_file;
  // osm_file.LoadFile("/home/chaitanya/Desktop/jubliee_45.osm");
  // osm_file.LoadFile("/home/chaitanya/Desktop/reference.osm");
  osm_file.LoadFile(fp);
  tinyxml2::XMLNode *rootnode;
  tinyxml2::XMLElement *element;
  tinyxml2::XMLElement *element1;
  tinyxml2::XMLElement *element2;
  tinyxml2::XMLElement *way_ref;
  tinyxml2::XMLElement *tag_ref;
  unsigned int id;
  unsigned int way_id;
  unsigned int way_nd_ref;
  std::string key;
  std::string value;
  std::string value1;
  double lat, min_lat, max_lat;
  double lon, min_lon, max_lon;
  int i, j, k;
  std::string zone;

  rootnode = osm_file.FirstChildElement("osm");
  element = rootnode->FirstChildElement("node");
  element1 = rootnode->FirstChildElement("way");
  element2 = rootnode->FirstChildElement("bounds");

  element2->QueryDoubleAttribute("minlat", &min_lat);
  element2->QueryDoubleAttribute("minlon", &min_lon);
  element2->QueryDoubleAttribute("maxlat", &max_lat);
  element2->QueryDoubleAttribute("maxlon", &max_lon);
  CHECK_RET(LLtoUTM(min_lat, min_lon, min_northing, min_easting, zone));
  CHECK_RET(LLtoUTM(max_lat, max_lon, max_northing, max_easting, zone));

  while (element) {
    element->QueryUnsignedAttribute("id", &id);
    node_data.node_id.push_back(id);
    element->QueryDoubleAttribute("lat", &lat);
    element->QueryDoubleAttribute("lon", &lon);
    CHECK_RET(LLtoUTM(lat, lon, northing, easting, zone));
    node_data.node_lat.push_back(easting);
    node_data.node_long.push_back(northing);
    element = element->NextSiblingElement("node");
  }

  while (element1) {
    way way_inst;
    element1->QueryUnsignedAttribute("id", &way_id);
    way_inst.dup_way_id = way_id;
    way_ref = element1->FirstChildElement("nd");
    while (way_ref) {
      way_ref->QueryUnsignedAttribute("ref", &way_nd_ref);
      way_inst.dup_way_nd_ref.push_back(way_nd_ref);
      way_ref = way_ref->NextSiblingElement("nd");
    }

    tag_ref = element1->FirstChildElement("tag");
    while (tag_ref) {
      value = "highway";
      if (tag_ref->Attribute("k") == value) {
        way_inst.tag = "highway";
        value1 = tag_ref->Attribute("v");
        way_inst.tag_value = value1;
        break;
      }
      value = "building";
      if (tag_ref->Attribute("k") == value) {
        way_inst.tag = "building";
        way_inst.tag_value = "NA Building";
        break;
      } else {
        tag_ref = tag_ref->NextSiblingElement("tag");
      }
      way_inst.tag = "NA";
      way_inst.tag_value = "NA NA";
    }
    dup_way.push_back(way_inst);
    element1 = element1->NextSiblingElement("way");
  }

  for (i = 0; i < dup_way.size(); i++) {
    final_way final_way_inst;
    final_way_inst.final_way_id = dup_way[i].dup_way_id;
    final_way_inst.final_way_tag = dup_way[i].tag;
    final_way_inst.final_way_tag_value = dup_way[i].tag_value;
    for (j = 0; j < dup_way[i].dup_way_nd_ref.size(); j++) {

      for (k = 0; k < node_data.node_id.size(); k++) {
        if (dup_way[i].dup_way_nd_ref[j] == node_data.node_id[k]) {
          final_way_inst.utm_x.push_back(node_data.node_lat[k]);
          final_way_inst.utm_y.push_back(node_data.node_long[k]);
          final_way_inst.final_node_id.push_back(node_data.node_id[k]);
        }
      }
    }
    final_way_vect.push_back(final_way_inst);
  }

  SP$(sn)
  ct_vpur(final_way_vect.size()) pcommas ct_vpur(node_data.node_id.size())
      pcommas ct_vpur(node_data.node_lat.size())
          pcommas ct_vpur(node_data.node_long.size()) pendl;
  for (int i = 0; i < final_way_vect.size(); i++) {
    SP$(sn)
    ct_vpur(final_way_vect[i].utm_x.size())
        pcommas ct_vpur(final_way_vect[i].utm_y.size()) pendl;
  }

  if (fp != NULL) {
    fclose(fp);
    fp = NULL;
  }

  time = Toc();
  PN$ "time taken:	" << time pendl;

  PRINT_FUNC_EXIT;
  return RET_SUCCESS_INT;
}

#ifdef BUILT_WITH_ROS
/*
int OSMObject::reinit(const std::string& topic_sub, ros::NodeHandle& n,
              uint32_t queue_sz )
{
char* sample1 = "";
CHECK_RET(reinit(sample1));
return RET_SUCCESS_INT;
}
*/
int OSMObject::reinit_fix(const std::string &topic_sub, ros::NodeHandle &n,
                          uint32_t queue_sz) {
  sub_ = n.subscribe(topic_sub, queue_sz, &OSMObject::fix_callback, this);
  return RET_SUCCESS_INT;
}

void OSMObject::fix_callback(const sensor_msgs::NavSatFix &fix_msg) {
  int i;
  int f1 = fix_msg.latitude;
  float offset1 = 0.01, offset2 = 0.1, f2, f3;
  double fix_northing, fix_easting, lat, lon;
  std::string zone, sample;
  char path[100], folder[10] = "osm_files";
  lat = fix_msg.latitude;
  lon = fix_msg.longitude;
  // LLtoUTM(fix_msg.latitude, fix_msg.longitude, fix_northing, fix_easting,
  // zone);
  LLtoUTM(lat, lon, fix_northing, fix_easting, zone);

  if (flag == 0) {
    first_gps_x = fix_easting;
    first_gps_y = fix_northing;
  }

  traj_obj.set_pose(fix_easting - first_gps_x, fix_northing - first_gps_y, 0);
  f2 = (int)(fix_msg.latitude / offset1) * offset1;
  f3 = (int)(fix_msg.longitude / offset2) * offset2;
#ifndef _DEPLOY_SWAHANA_SUITE_
  sprintf(path, "%s/%s/%.1f/%d/%.2f.osm", PKG_SRC, folder, f3, f1, f2);
#else
  sprintf(path, "%s/../%s/%.1f/%d/%.2f.osm", INPUT_PREFIX, folder, f3, f1, f2);
#endif
  // sprintf (path, "/home/chaitanya/Desktop/%d/%.2f.osm",f1, f2);

  if (flag == 0) {
    flag = 1;
    strcpy((char *)prev, (char *)path);
    reinit(path);
  } else {
    if (strcmp((char *)prev, (char *)path) == 0) {
      return;
    } else {
      strcpy((char *)prev, (char *)path);
      reinit(path);
    }
  }
}

#endif

/* Starting of the display part  */

int OSMObject::draw_obj(pangolin::View &v) {
  int i, j;

  glLineWidth(1);
  glColor3f(0.0f, 1.0f, 0.0f);

  glBegin(GL_LINES);
  for (i = 0; i < final_way_vect.size(); i++) {
    for (j = 0; j < final_way_vect[i].utm_x.size() - 1; j++) {
      // glVertex3f(final_way_vect[i].utm_x[j]-min_easting,final_way_vect[i].utm_y[j]-min_northing,0);
      // glVertex3f(final_way_vect[i].utm_x[j+1]-min_easting,final_way_vect[i].utm_y[j+1]-min_northing,0);
      glVertex3f(final_way_vect[i].utm_x[j] - first_gps_x,
                 final_way_vect[i].utm_y[j] - first_gps_y, 0);
      glVertex3f(final_way_vect[i].utm_x[j + 1] - first_gps_x,
                 final_way_vect[i].utm_y[j + 1] - first_gps_y, 0);
    }
  }
  glEnd();

  glPointSize(4);
  glColor3f(0.0f, 0.0f, 1.0f);
  glBegin(GL_POINTS);
  for (i = 0; i < final_way_vect.size(); i++) {
    for (j = 0; j < final_way_vect[i].utm_x.size(); j++) {
      // glVertex3f(final_way_vect[i].utm_x[j]-min_easting,final_way_vect[i].utm_y[j]-min_northing,0);
      glVertex3f(final_way_vect[i].utm_x[j] - first_gps_x,
                 final_way_vect[i].utm_y[j] - first_gps_y, 0);
    }
  }
  glEnd();

  CHECK_RET(traj_obj.draw_obj(v));

  /*
  //GPS display from NavSatFix messages

  // gps points
  glPointSize(2*traj_size_);
  glColor3fv(traj_pts_col_);
  glBegin(GL_POINTS);
  for(i=0; i<traj_obj.gps_x.size(); i+=3)
  {
  glVertex3f(traj_obj.traj_[i]-traj_obj.traj_[0],traj_obj.gps_y[i+1]-traj_obj.traj_[1],0);
  //glVertex3f(gps_x[i]-min_easting,gps_y[i]-min_northing,0);
  }
  glEnd();


    // draw line
    glLineWidth(traj_size_);
    glColor3fv(traj_line_col_);
    glBegin(GL_LINES);
    for (size_t i = 0; i < traj_obj.gps_x.size(); i+=3) {
     if(i==0) continue;
      glVertex3f(traj_obj.gps_x[i -
  1]-traj_obj.gps_x[0],traj_obj.gps_y[i-1]-traj_obj.gps_y[0],0);
      glVertex3f(traj_obj.gps_x[i]-traj_obj.gps_x[0],traj_obj.gps_y[i]-traj_obj.gps_y[0],0);
    }
    glEnd();
  */

  return RET_SUCCESS_INT;
}

#if 0
char OSMObject::UTMLetterDesignator(double Lat)
{
	char LetterDesignator;

	if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
	else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
	else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
	else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
	else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
	else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
	else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
	else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
	else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
	else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
	else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
	else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
	else if((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
	else if((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
	else if((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
	else if((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
	else if((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
	else if((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
	else if((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
	else if((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
	else LetterDesignator = 'Z';
	return LetterDesignator;
}
#endif

int OSMObject::LLtoUTM(const double Lat, const double Long, double &UTMNorthing,
                       double &UTMEasting, char *UTMZone) {
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long + 180) - int((Long + 180) / 360) * 360 - 180;

  double LatRad = Lat * RADIANS_PER_DEGREE;
  double LongRad = LongTemp * RADIANS_PER_DEGREE;
  double LongOriginRad;
  int ZoneNumber;

  ZoneNumber = int((LongTemp + 180) / 6) + 1;

  if (Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
    ZoneNumber = 32;

  // Special zones for Svalbard
  if (Lat >= 72.0 && Lat < 84.0) {
    if (LongTemp >= 0.0 && LongTemp < 9.0)
      ZoneNumber = 31;
    else if (LongTemp >= 9.0 && LongTemp < 21.0)
      ZoneNumber = 33;
    else if (LongTemp >= 21.0 && LongTemp < 33.0)
      ZoneNumber = 35;
    else if (LongTemp >= 33.0 && LongTemp < 42.0)
      ZoneNumber = 37;
  }
  // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

  // compute the UTM Zone from the latitude and longitude

  // snprintf(UTMZone, 4, "%d%c", ZoneNumber, UTMLetterDesignator(Lat));  //to
  // compute the zone of the latitude.

  eccPrimeSquared = (eccSquared) / (1 - eccSquared);

  N = a / sqrt(1 - eccSquared * sin(LatRad) * sin(LatRad));
  T = tan(LatRad) * tan(LatRad);
  C = eccPrimeSquared * cos(LatRad) * cos(LatRad);
  A = cos(LatRad) * (LongRad - LongOriginRad);

  M = a *
      ((1 - eccSquared / 4 - 3 * eccSquared * eccSquared / 64 -
        5 * eccSquared * eccSquared * eccSquared / 256) *
           LatRad -
       (3 * eccSquared / 8 + 3 * eccSquared * eccSquared / 32 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(2 * LatRad) +
       (15 * eccSquared * eccSquared / 256 +
        45 * eccSquared * eccSquared * eccSquared / 1024) *
           sin(4 * LatRad) -
       (35 * eccSquared * eccSquared * eccSquared / 3072) * sin(6 * LatRad));

  UTMEasting =
      (double)(k0 * N * (A + (1 - T + C) * A * A * A / 6 +
                         (5 - 18 * T + T * T + 72 * C - 58 * eccPrimeSquared) *
                             A * A * A * A * A / 120) +
               500000.0);

  UTMNorthing =
      (double)(k0 *
               (M +
                N * tan(LatRad) *
                    (A * A / 2 +
                     (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24 +
                     (61 - 58 * T + T * T + 600 * C - 330 * eccPrimeSquared) *
                         A * A * A * A * A * A / 720)));
  if (Lat < 0)
    UTMNorthing += 10000000.0; // 10000000 meter offset for southern hemisphere

  return RET_SUCCESS_INT;
}
int OSMObject::LLtoUTM(const double Lat, const double Long, double &UTMNorthing,
                       double &UTMEasting, std::string &UTMZone) {
  char zone_buf[] = {0, 0, 0, 0};

  CHECK_RET(LLtoUTM(Lat, Long, UTMNorthing, UTMEasting, zone_buf));

  UTMZone = zone_buf;

  return RET_SUCCESS_INT;
}

/*--------------------------OSMObject End------------------------------*/
