#include <utils/mode_printing.h>

#if !UTILS_USE_STL
#include <list>
#include <array>
#include <map>
#endif
/*
 * Two arguments provided would affect the behaviour of the code.
 * 
 * --vl=x, x should -1 to 3, for their meaning lookup the header file.
 *  --cond, to show how the conditional printing works.
 * */

//bool flagvalid(const char* flagname, bool val) {
  //return true;
//}

#ifdef UTILS_AVOID_GFLAGS
DEFINE_bool(cond, false, "to print conditional statements");
//DENE_validator(cond,flagvalid);
#else
bool FLAGS_cond = true;
#endif

int main(int argc, char *argv[])
{
  STL_DEFINED_STATE;
  //If using gflags then it will call gflags::ParseCommandLineFlags.
  //if this line is not called all flags would have default value, this calls
  //gflags::ParseCommandLineFlags, so you don't need to call it again.
  print::init(&argc, &argv);
  //Logger().stream() << "Check argc = "<< ct_red(argc) pendl;
  //Logger().stream() << "Check argc = "<< argc pendl;
  int val = 42;
  //FLAGS_sl=3;
  //FLAGS_sections = "42,10";

  PI$ "Info print" pendl;
  PN$ PRINT_SUCCESS << ct_red("Suc print normal in ct_red") pendl;
  //printf("Following text is in " ct_red("red\n"));
  //printf(vredp(val));
  float fval = 42.0006;
  //printf(vblupn(fval));
  PV$ PRINT_SUCCESS << ct_dred("Suc print verbose in dull ct_red") pendl;
  PW$ "Warn print" pendl;
  PE$ ct_bgrn(ct_red("Error print")) pendl;

  DP$ "Debug print" pendl;

  std::string str = "cError print";
  cPI$(FLAGS_cond) "cInfo print" pendl;
  cPN$(FLAGS_cond) PRINT_SUCCESS << "cSuc print normal" pendl;
  cPV$(FLAGS_cond) PRINT_SUCCESS << "cSuc print verbose" pendl;
  cPW$(FLAGS_cond) "cWarn print" pendl;
  cPE$(FLAGS_cond) ct_vgrn(str) pendl;

  DP$ ct_vpur((FLAGS_cond?val:0)) pendl;

  DP$ ct_vpur(FLAGS_sl) pendl;

  SP$(1) "Section 1" pendl;

#if UTILS_USE_STL
  std::vector<int> v;
  v.push_back(10);
  v.push_back(11);
  v.push_back(13);
  //PRINT_VECTOR(v);
  PRINT_STL(v);

  std::list<int> l;
  //l.push_back(11);
  //l.push_back(12);
  //l.push_back(14);
  PRINT_STL(l);

  std::array<int,3> a;
  a[0] = 16;
  a[1] = 17;
  a[2] = 19;
  PRINT_STL(a);

  std::map<int,int> m;
  m[0] = 21;
  m[4] = 22;
  m[2] = 24;
  PRINT_STL(m);
#endif

  PF$(1) "Demo of Fatal" pendl; // exit with code 1, PF$(2) will exit with code 2.

  return 0;
}
