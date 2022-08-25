#include <utils/gui/graph_objects.hpp>
#include <utils/gui/objects_2d.hpp>
#include <utils/gui/objects_3d.hpp>
#include <utils/gui/view_graph.hpp>

#include <utils/fs_utils.h>
#include <utils/mode_printing.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PAP pangolin::Attach::Pix
#define PAF pangolin::Attach::Frac

#define WIN_W 1366
#define WIN_H 768

int main(int argc, char *argv[]) {
  std::string sample = "inside example";
  print::init(&argc, &argv, true);
  Window.init("Test Window", WIN_W, WIN_H, &argc, &argv);

  // Empty to add it to base.
  Window.add_container("Main", PAF(0.0f), PAF(1.0f) /*0.66*/, PAF(0.0f),
                       PAF(1.0f) /*1.0*/, pangolin::LayoutEqualHorizontal, "");

  Window.add_2d_view("text", "Main");

  //TextScrollHandler tsh;
  TextObject *txt;
  txt = new TextObject();
  Viewnode("text")->add_object("txt", txt);
  TextScrollHandler *tsh = new TextScrollHandler(txt);

  vec_str lines = utils::read_file("../textview_demo.txt");
  PRINT_VECTOR_SEC(100, lines);

  int count = 0;
  for (; !pangolin::ShouldQuit();) {
    // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (count < (int)lines.size()) {
      SP$(100) lines[count] pendl;
      txt->update(lines[count]);
      //tsh->Mouse("text",pangolin::MouseWheelUp,0,0,false,1);
      View("text").SetHandler(tsh);
    }

    pangolin::FinishFrame();
    count++;
  }

  return 0;
}
