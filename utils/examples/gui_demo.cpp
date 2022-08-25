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

#define WIN_W 640
#define WIN_H 480

int main(int argc, char *argv[])
{
  Window.init("Test Window", WIN_W, WIN_H, &argc, &argv);

#if 0
  //Empty to add it to base.
  Window.add_container("Main", PAF(0.0f), PAF(2/3.0f)/*0.66*/, PAF(0.0f),
                       PAF(1.0f)/*1.0*/, pangolin::LayoutEqualHorizontal, "");

  //This will also add to base.
  Window.add_container("view_panel", PAF(2/3.0f), PAF(1.0f)/*1.0*/, PAF(0.0f),
                       PAF(1.0f)/*1.0f*/, pangolin::LayoutEqualHorizontal);


  Window.add_2d_view("view2d_1", "Main");
  Window.add_3d_view("view3d_1", "Main");
  Window.add_2d_view("view2d_2", "view_panel");

  Window.add_2d_view("view2d_3", "view_panel");
  Window.add_2d_view("view2d_4", "view_panel");
  Window.add_2d_view("view2d_5", "view_panel");
  Window.add_2d_view("view2d_6", "view_panel");
#else
  Window.read_layout("../sample.layout");
#endif

#if 0
  ImageObject img_obj_1("/home/laxit/data/fisheye/2928323892_06303c17a8_o.jpg");
  ImageObject img_obj_2("/home/laxit/data/fisheye/3652525431_fbf5ff4a2c_b.jpg");
  ImageObject img_obj_3("/home/laxit/data/fisheye/3016071961_968cb7de10_o.jpg");
  ImageObject img_obj_4("/home/laxit/data/fisheye/16559382993_e05afc9108_b.jpg");
  VideoObject vid_obj_1("/home/laxit/data/abc.avi");

  img_obj_1.set_transparency(0.7f);
  img_obj_2.set_transparency(0.7f);
  img_obj_3.set_transparency(0.7f);
  img_obj_4.set_transparency(0.7f);
  vid_obj_1.set_transparency(0.7f);

  std::string view_name = "view2d_1";

  //Viewnode(view_name)->add_object(&img_obj_2);
  Viewnode("view2d_2")->add_object("img2", &img_obj_2);
  Viewnode("view2d_3")->add_object("img3", &img_obj_3);
  Viewnode("view2d_4")->add_object("img4", &img_obj_4);
  Viewnode("view2d_5")->add_object("vid1", &vid_obj_1);
  Viewnode("view2d_6")->add_object("img1", &img_obj_1);

  GraphObject* go = new GraphObject(utils::Color::white, true);

  //Nodes in the graph
  go->add_node("goal_broadcaster", utils::Color(utils::Color::red));
  go->add_node("current_goal", utils::Color(utils::Color::red));
  go->add_node("play_1416273497318894036", utils::Color(utils::Color::grey));
  go->add_node("goal_tf_broadcaster", utils::Color(utils::Color::hgreen));
  go->add_node("utm_odom_broadcaster", utils::Color(utils::Color::hgreen));
  go->add_node("current_pose", utils::Color(utils::Color::red));
  go->add_node("bm", utils::Color(utils::Color::cyan));
  go->add_node("camera", utils::Color(utils::Color::blue));
  go->add_node("sgbm", utils::Color(utils::Color::cyan));
  go->add_node("road_estimator", utils::Color(utils::Color::cyan));
  go->add_node("cluster_extraction", utils::Color(utils::Color::cyan));
  go->add_node("local_costmap_generator", utils::Color(utils::Color::cyan));
  go->add_node("pose_tf_broadcaster", utils::Color(utils::Color::hgreen));

  //Edges in the graph
  go->add_edge("goal_broadcaster", "current_goal");
  go->add_edge("current_goal", "goal_tf_broadcaster");
  go->add_edge("play_1416273497318894036", "goal_tf_broadcaster");
  go->add_edge("play_1416273497318894036", "utm_odom_broadcaster");
  go->add_edge("play_1416273497318894036", "current_pose");
  go->add_edge("play_1416273497318894036", "bm");
  go->add_edge("play_1416273497318894036", "camera");
  go->add_edge("camera", "sgbm");
  go->add_edge("sgbm", "road_estimator");
  go->add_edge("road_estimator", "cluster_extraction");
  go->add_edge("cluster_extraction", "local_costmap_generator");
  go->add_edge("play_1416273497318894036", "pose_tf_broadcaster");
  go->add_edge("current_pose", "pose_tf_broadcaster");

  go->draw_me_like_one_of_your_french_girls();
  Viewnode("view2d_1")->add_object("graph", go);
#endif

#if 1
  Viewnode("view3d_1")->set_viewcam_projection(pangolin::ProjectionMatrix(
          WIN_W, WIN_H, 0.9*WIN_W, 0.9*WIN_H, WIN_W/2, WIN_H/2, 0.1, 2000));
  Viewnode("view3d_1")->set_viewcam_pose(pangolin::ModelViewLookAt(
          1, -10, 10, 0, 200, 0, pangolin::AxisZ));
  Viewnode("view3d_1")->set_3d_handler();

  ZeroGridObject* zgo = new ZeroGridObject();
  zgo->set_line_width(5.5f);
  //Viewnode("view3d_1")->add_object("zgrid", zgo);
  Viewnode("view3d_1")->add_object("or_ax", new AxisObject(5.0f));
#endif

#if 1
  std::cout << "Reading points from PTS file....." << std::flush;
#if 1
  PointCloudObject* pco = new PointCloudObject(1, utils::Color::blue);
  double* pc = new double[144];
  for (int i = 0; i < 144; i+=3) {
    int pt_idx= i/3;
    pc[i] = (double)(pt_idx/6) + 1.0;
    pc[i+1] = (double)(pt_idx%6) + 1.0;
    pc[i+2] = 5.0;
    std::cout << "i = " << i << ", pi = " << pt_idx << ", x = "
        << (double)(pt_idx/6) + 1.0 << ", y = " << (double)(pt_idx%8) + 1.0 
        << ", z = " << 5.0 << std::endl;
  }
  for (int i = 0; i < 144; ++i) {
    std::cout << pc[i] << ", ";
  }
  std::cout << std::endl;
  pco->update(144, pc);
  Viewnode("view3d_1")->add_object("pc", pco);
#elif 0
  Viewnode("view3d_1")->add_object("pc",
      new PTSObject("/home/laxit/data/3d/flower.pts", utils::Color::red, 5.0));
#else
  Viewnode("view3d_1")->add_object("pc",
      new PTSObject("/home/laxit/data/3d/Site_56_college_division.pts",
                    0.2, 50));
#endif
  std::cout << "Done" << std::endl;
  Window.simple_idle_loop();
#else
  pangolin::RegisterKeyPressCallback('r',
                                     [](){
                                     std::cout << "Pressed 'r'!!" << std::endl;
                                     Viewnode("view3d_1")->remove_object("or_ax");
                                     });
  double* pts = NULL;
  unsigned char* col = NULL;
  std::string pts_file = "/home/laxit/data/3d/flower.pts";
  //if(!utils::file_accessible(pts_file, utils::FileAccessType::Read)) {
    //std::cout << "File " << pts_file << " cannot be read." << std::endl;
    //return 1;
  //}
  //std::cout << "Reading points from PTS file....." << std::flush;
  //int sz = PTSObject::read_PTS_file(pts_file, pts, col, utils::Color::red, 10);
  int sz = 1;
  //std::cout << "read " << sz << " points read. Done." << std::endl;
  //PointCloudObject* pco = new PointCloudObject(50);
  //Viewnode("view3d_1")->add_object("pc", pco);
  int cnt_per_elem = 3, array_sz = cnt_per_elem*sz, batch_sz = array_sz/50;//100;
  int cnt = 0;
  for (int i = 0; !pangolin::ShouldQuit(); i += batch_sz) {
    //if (i >= array_sz) {
      //i = 0;
      //cnt = 0;
    //}
    //pco->update(batch_sz, pts+i, col+i);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    pangolin::FinishFrame();
    cnt++;
  }
#endif

  return 0;
}
