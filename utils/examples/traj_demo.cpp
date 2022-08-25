#include <utils/gui/view_graph.hpp>
#include <utils/gui/objects_2d.hpp>
#include <utils/gui/objects_3d.hpp>
#include <utils/gui/graph_objects.hpp>

#include <utils/fs_utils.h>
#include <utils/mode_printing.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PAP pangolin::Attach::Pix
#define PAF pangolin::Attach::Frac

#define WIN_W 1366
#define WIN_H 768

vec_str lines;;
bool thread_run;
TrajectoryObject* traj;

void thread_add_poses() {
  lines = utils::read_file("../pose.txt");
  //std::cout << __LINE__ << ": l.sz = " << lines.size() << std::endl;
  lines.erase(std::remove_if(lines.begin(), lines.end(),
                             [](std::string s) { return s.empty(); }),
              lines.end());
  //std::cout << __LINE__ << ": l.sz = " << lines.size() << std::endl;
  
  if (lines.size() % 3 != 0) {
    std::cout << "ERROR: Number of non-blank lines in pose file is not multiple"
        " of 3"<< std::endl;
    return;
  }
  int cnt = 0;
  pangolin::OpenGlMatrix pose;
  std::vector<pangolin::GLprecision> nums;
  while(thread_run && cnt < lines.size()) {
    //std::this_thread::sleep_for(std::chrono::seconds(1));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    nums = utils::split_str<pangolin::GLprecision>(lines[cnt++], " ");
    pose(0,0)=nums[0], pose(0,1)=nums[1], pose(0,2)=nums[2], pose(0,3)=nums[3];

    nums = utils::split_str<pangolin::GLprecision>(lines[cnt++], " ");
    pose(1,0)=nums[0], pose(1,1)=nums[1], pose(1,2)=nums[2], pose(1,3)=nums[3];

    nums = utils::split_str<pangolin::GLprecision>(lines[cnt++], " ");
    pose(2,0)=nums[0], pose(2,1)=nums[1], pose(2,2)=nums[2], pose(2,3)=nums[3];

    pose(3,0)=0.0, pose(3,1)=0.0, pose(3,2)=0.0, pose(3,3)=1.0;
    pose = pose.Inverse();

    traj->set_pose(pose);

  }
}

int main(int argc, char *argv[])
{
  Window.init("Test Window", WIN_W, WIN_H, &argc, &argv);

  //Empty to add it to base.
  Window.add_container("Main", PAF(0.0f), PAF(1.0f)/*0.66*/, PAF(0.0f),
                       PAF(1.0f)/*1.0*/, pangolin::LayoutEqualHorizontal, "");

  Window.add_3d_view("view3d_1", "Main");

  Viewnode("view3d_1")->set_viewcam_projection(pangolin::ProjectionMatrix(
          WIN_W, WIN_H, 0.9*WIN_W, 0.9*WIN_H, WIN_W/2, WIN_H/2, 0.1, 2000));
  Viewnode("view3d_1")->set_viewcam_pose(pangolin::ModelViewLookAt(
          1, -5, -10, 0, 0, 5, pangolin::AxisNegY));
  Viewnode("view3d_1")->set_3d_handler();

  Viewnode("view3d_1")->add_object("ygrid",
                                   new ZeroGridObject(ZeroGridObject::YGrid));
  //Viewnode("view3d_1")->add_object("or_ax", new AxisObject(5.0f));

  traj = new TrajectoryObject(4, true);
  Viewnode("view3d_1")->add_object("traj", traj);
  traj->set_gl_cam_relative(1.0f, -5.0f, 0.0f, pangolin::AxisZ);

  pangolin::OpenGlMatrix mat;
  mat(0,0) = 0.98;mat(0,1) = 0.025;mat(0,2) = 0.17;mat(0,3) = 21.01;
  mat(1,0) = -0.02;mat(1,1) = 0.99;mat(1,2) = 0.02;mat(1,3) = -0.19;
  mat(2,0) = 0.17;mat(2,1) = -0.01;mat(2,2) = 0.98;mat(2,3) = 16.06;
  mat(3,0) = 0;mat(3,1) = 0;mat(3,2) = 0;mat(3,3) = 1;
  Eigen::Matrix4d eig_m1 = mat;
  mat = mat.RotateY(-1.57)*mat;
  mat = mat.RotateZ(1.57)*mat;
  //PT$ ct_grn("view") << " = \n" << mat pendl;
  //Eigen::Matrix4d eig_m2 = mat;


#define USE_POSE_THREAD 1

#if USE_POSE_THREAD
  thread_run = true;
  std::thread th(&thread_add_poses);
#endif

  for (; !pangolin::ShouldQuit();) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDisable(GL_LIGHTING);

    //pangolin::glDrawAxis(eig_m1, 20.0);
    //pangolin::glDrawAxis(eig_m2, 20.0);
    pangolin::glDrawAxis(20.0);
    pangolin::FinishFrame();
  }

#if USE_POSE_THREAD
  thread_run = false;
  if (th.joinable()) {
    th.join();
  }
#endif

  return 0;
}
