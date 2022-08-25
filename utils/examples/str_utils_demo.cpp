#include <iostream>
#define USE_STL_UTIL 1
#include <utils/str_utils.h>
#ifndef USE_STL_UTIL
#include <map>
#include <vector>
#endif

using namespace utils;

int main(void)
{
  int a = 10;
  std::string s = "1";
  bool test = false;
  float b = str_to_val<float>(s);
  std::cout << "ternary op in cout: " << (test?s:val_to_str(a)) << "\n" 
      << "converted to float val: " << b << std::endl;

  //split.
  std::cout << "split when delim is not present." << std::endl;
  std::vector<int> vs_blah;
  split_str(s, ',', &vs_blah);
  for (int i=0; i < vs_blah.size(); i++) {
    std::cout << vs_blah[i] << "  ";
  }
  std::cout << std::endl;

  //split into string.
  s = "one,two,three.";
  std::vector<std::string> vss;
  split_str(s, ',', &vss);
  for (int i=0; i < vss.size(); i++) {
    std::cout << vss[i] << "  ";
  }
  std::cout << std::endl;

  s = "1,2.5,3,one,10";
  std::cout << "\nstring to be tokenized " << s << "; with delim = " << ','
      << std::endl;

  std::cout << "\t[int]with char delim, function param\n\t\t";
  std::vector<int> vi;
  split_str(s, ',', &vi);
  for (int i=0; i < vi.size(); i++) {
    std::cout << vi[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\t[float]with char delim, assign return value\n\t\t";
  std::vector<float> vf = split_str<float>(s, ',');
  for (int i=0; i < vf.size(); i++) {
    std::cout << vf[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\t[float]with string delim, function param\n\t\t";
  vf.clear();
  split_str(s, ",", &vf);
  for (int i=0; i < vf.size(); i++) {
    std::cout << vf[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\t[float]with string delim, assign return value\n\t\t";
  vf.clear();
  vf = split_str<float>(s, ",");
  for (int i=0; i < vf.size(); i++) {
    std::cout << vf[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\t[bool]with char delim, function param\n\t\t";
  std::vector<bool> vb;
  split_str(s, ',', &vb);
  for (int i=0; i < vb.size(); i++) {
    std::cout << vb[i] << ", ";
  }
  std::cout << std::endl;

  std::cout << "\nsplit string in two" << std::endl;
  std::string s_1 = s;
  std::string s_n = s;
  std::string s_e = s+",";
  std::string s__n = s;
  std::string s__e = ","+s;

  std::cout << "\tsplit at first occurance for " << s_1 << std::endl;
  std::string str_1 = split_str_in_two(s_1, ",");
  std::cout << "\t\tfirst: " << str_1 << "\t second: " << s_1 << std::endl;

  std::cout << "\tsplit at second occurance for " << s_n << std::endl;
  std::string str_n = split_str_in_two(s_n, ",", 2);
  std::cout << "\t\tfirst: " << str_n << "\t second: " << s_n << std::endl;

  std::cout << "\tnth occurance is the last char(boundary) for " << s_e
      << std::endl;
  std::string str_e = split_str_in_two(s_e, ",", 5);
  std::cout << "\t\tfirst: " << str_e << "\t second: " << s_e << std::endl;

  std::cout << "\tsplit at second occurance from back for " << s__n
      << std::endl;
  std::string str__n = split_str_in_two(s__n, ",", -2);
  std::cout << "\t\tfirst: " << str__n << "\t second: " << s__n << std::endl;

  std::cout << "\tnth occurance from back is the first char(boundary) for "
      << s_e << std::endl;
  std::string str__e = split_str_in_two(s__e, ",", -5);
  std::cout << "\t\tfirst: " << str__e << "\t second: " << s__e << std::endl;

  std::cout << "\nsplit string in two, splitted string as params for " 
      << s << std::endl;

  std::cout << "\tby declaring normal var." << std::endl;
  std::string fi,se;
  std::cout << "\t\tres = " << split_str_in_two(s, ",", &fi, &se, 6) << "; f:"
      << fi << "; s: " << se << std::endl;

  std::cout << "\tby declaring pointer var." << std::endl;
  std::string* fi_ptr = new std::string;
  std::string* se_ptr = new std::string;
  std::cout << "\t\tres = " << split_str_in_two(s, ",on", fi_ptr, se_ptr)
      << "; f:" << *fi_ptr << "; s: " << *se_ptr << std::endl;

  std::cout << "\tno delimiter(;) found in " << s << std::endl;
  *fi_ptr = "empty"; *se_ptr = "empty";
  std::cout << "\t\tres = " << split_str_in_two(s, ";", fi_ptr, se_ptr)
      << "; f:" << *fi_ptr << "; s: " << *se_ptr << std::endl;

  std::cout << "\tdelim(0) the last char" << s << std::endl;
  *fi_ptr = "empty"; *se_ptr = "empty";
  std::cout << "\t\tres = " << split_str_in_two(s, "0", fi_ptr, se_ptr)
      << "; f:" << *fi_ptr << "; s: " << *se_ptr << std::endl;

  std::cout << "\tby declaring shared pointer var." << std::endl;
  std::shared_ptr<std::string> fi_sp = std::make_shared<std::string>();
  std::shared_ptr<std::string> se_sp = std::make_shared<std::string>();
  std::cout << "\t\tres = " << split_str_in_two(s, ",", fi_sp, se_sp, 3)
      << "; f:" << *fi_sp.get() << "; s: " << *se_sp.get() << std::endl;

  std::map<int, std::string> mis;
  mis[0] = "zero";
  mis[3] = "three";
  mis[8] = "eight";
#if USE_STL_UTIL
  //testing stl operators.
  std::cout << "\ntesting stl operators" << std::endl;
  std::cout << "\tvf = " << vf << std::endl;
  std::cout << "\tmis = " << mis << std::endl;
  std::cout << "\ntesting stl operators via stringstream" << std::endl;
  std::stringstream ss;
  ss << vf;
  std::cout << ss.str() << std::endl;
  ss.str(""); ss << mis;
  std::cout << "\ntesting stl operators via stringstream" << std::endl;
  std::cout << "\t" << ss.str() << std::endl;

  std::string stl_str = join_stl(mis);
  std::cout << "\nstring from stl container: " << std::endl;
  std::cout << "\t" << stl_str << std::endl;
  stl_str = join_stl(mis, "::");
  std::cout << "\nstring from stl container with delim: " << std::endl;
  std::cout << "\t" << stl_str << std::endl;
#else
  std::string stl_str = join_stl(mis);
  std::cout << "\nstring from stl container: " << std::endl;
  std::cout << "\t" << stl_str << std::endl;

  stl_str = join_stl(mis, "::");
  std::cout << "\nstring from stl container with delim: " << std::endl;
  std::cout << "\t" << stl_str << std::endl;
#endif

  std::cout << "-------------------case switch------------------" << std::endl;
  std::cout << "Lower and Upper:" << std::endl;
  std::string lu = "This is a string";
  std::cout << "orig: " << lu << std::endl;
  std::cout << "lower: " << utils::to_lower(lu) << std::endl;
  std::cout << "upper: " << utils::to_upper(lu) << std::endl;

  //replace
  std::cout << "-------------------replace--------------------" << std::endl;
  std::cout << "Replacing substring within string." << std::endl;
  std::string str = "This is the orig string";
  std::cout << "orig: " << str << std::endl;
  std::cout << "replace(orig,replaced): "
      << utils::replace(str, "orig", "replaced") << std::endl;

  str = "This str has repeated str, it repeats str 3 times.";
  std::cout << "orig: " << str << std::endl;
  std::cout << "Replacing str with string: " << std::endl;
  std::cout << "1 time: " << utils::replace(str, "str", "string", 1)
      << std::endl;
  std::cout << "2 time: " << utils::replace(str, "str", "string", 2)
      << std::endl;
  std::cout << "-1 time: " << utils::replace(str, "str", "string", -1)
      << std::endl;
  std::cout << "3 time: " << utils::replace(str, "str", "string", 3)
      << std::endl;
  std::cout << "5 time: " << utils::replace(str, "str", "string", 5)
      << std::endl;
  std::cout << "Replacing str with string in place: " << std::endl;
  int cnt = utils::replace_inplace(str, "str", "string");
  std::cout << "inplace: " << str << ", no. of replacements = " << cnt
      << std::endl;

  return 0;
}
