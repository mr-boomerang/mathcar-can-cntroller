#include <iostream>

#define USE_STL_UTIL
#include <utils/fs_utils.h>
#include <utils/str_utils.h>

using namespace utils;

int main(int argc, char** argv )
{
  std::string str = "~";
  std::cout << "Input is " << str << std::endl;
  std::cout << "isdir? " << is_dir(str) << std::endl;
  std::cout << "file exists? " <<file_exist(str) << std::endl;
  std::cout << "file write accessible? "
      << file_accessible(str, FileAccessType::Write) << std::endl;

  std::cout << "remove_trailing_slash of " << tilde_expansion(str) << " = "
      << tilde_expansion(remove_trailing_slash(str + "/")) << std::endl;
  std::cout << "remove_trailing_slash of " << tilde_expansion(str) << "/ = "
      << tilde_expansion(remove_trailing_slash(str)) << std::endl;
  
  std::cout << "after tilde expansion: " << tilde_expansion(str) << std::endl;
  std::cout << "path of containing dir: " << dir_path(str) << std::endl;
  std::cout << "basename: " << basename(str) << std::endl;

  PRINT_SEPERATOR;

  //----------------------------------------------------

  //remember to change rm -rf command at the end of main.
  int ign;
  ign = system("mkdir -p temp_fs_demo_dir");
  ign = system("mkdir -p temp_fs_demo_dir/left");
  ign = system("mkdir -p temp_fs_demo_dir/right");
  ign = system("touch temp_fs_demo_dir/left/1.jpg");
  ign = system("touch temp_fs_demo_dir/left/2.jpg");
  ign = system("touch temp_fs_demo_dir/left/3.jpg");
  ign = system("touch temp_fs_demo_dir/left/4.jpg");
  ign = system("touch temp_fs_demo_dir/left/5.jpg");
  ign = system("touch temp_fs_demo_dir/right/1.jpg");
  ign = system("touch temp_fs_demo_dir/right/2.jpg");
  ign = system("touch temp_fs_demo_dir/right/3.jpg");
  ign = system("touch temp_fs_demo_dir/right/4.jpg");
  ign = system("touch temp_fs_demo_dir/right/.5.jpg");

  std::string regex = "temp_fs_demo_dir/*t/*";
  //lot of options in this, explore it.
  std::vector<std::string> name_list = filelist_from_regex(regex);
  std::cout << join_stl(name_list) << std::endl;
  std::cout << join_stl(name_list, "\n") << std::endl;

  std::string fn = name_list[0];
  std::cout << "overwrite confirmation w/o alternate option for " << fn pendl;
  fn = get_overwrite_confirmation(fn);
  std::cout << "overwrite confirmation w/ alternate option for " << fn pendl;
  fn = get_overwrite_confirmation(fn, true);

  ign = system("rm -rf temp_fs_demo_dir");

  if(argc > 1) {
    std::vector<std::string> nl = filelist_from_regex(argv[1]);
    //std::cout << join_stl(nl, "\n") << std::endl;
  }

  std::string file_name = "file_name.ext";
  std::cout << utils::filename_without_extension(file_name) << std::endl;

  return 0;
}
