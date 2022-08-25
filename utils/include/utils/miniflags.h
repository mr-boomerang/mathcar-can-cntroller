#ifndef MINIFLAGS_H_16MUCQ2A
#define MINIFLAGS_H_16MUCQ2A

// Flag map for parsing command line.
#include <map>
#include <string>

#define SET_VAL(type, v) *reinterpret_cast<type *>(var_) = v

namespace miniflags {

struct Flag {
  Flag(void *var, std::string name, char type, std::string desc)
      : name_(name), var_(var), type_(type), desc_(desc) {}

  bool set(std::string val);

  inline char type() { return type_; }
  inline std::string name() { return name_; }
  inline std::string description() { return desc_; }

private:
  std::string name_;
  void *var_;
  char type_; // i:int32, I:int64, b:bool, s:string, f:float, d:double
  std::string desc_;
};

extern std::map<std::string, Flag *> flag_map;
struct FlagMapAdd {
  FlagMapAdd(void *var, std::string name, char type, std::string desc) {
    flag_map[name] = new Flag(var, name, type, desc);
  }
};

// MACROS for defining flags
// def is for default value
#define DEFINE_bool(x, def, desc)                                              \
  bool FLAGS_##x = def;                                                        \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 'b', desc)
#define DEFINE_string(x, def, desc)                                            \
  std::string FLAGS_##x = def;                                                 \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 's', desc)
// when int64 is defined it would be I, this is i
#define DEFINE_int32(x, def, desc)                                             \
  int FLAGS_##x = def;                                                         \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 'i', desc)
#define DEFINE_int64(x, def, desc)                                             \
  int64_t FLAGS_##x = def;                                                     \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 'I', desc)
#define DEFINE_uint64(x, def, desc)                                            \
  uint64_t FLAGS_##x = def;                                                    \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 'U', desc)
#define DEFINE_double(x, def, desc)                                            \
  double FLAGS_##x = def;                                                      \
  miniflags::FlagMapAdd fma_##x(&FLAGS_##x, #x, 'd', desc)

#define DECLARE_bool(x) extern bool FLAGS_##x
#define DECLARE_string(x) extern std::string FLAGS_##x
// when int64 is defined it would be I, this is i
#define DECLARE_int32(x) extern int FLAGS_##x
#define DECLARE_int64(x) extern int64_t FLAGS_##x
#define DECLARE_uint64(x) extern uint64_t FLAGS_##x
#define DECLARE_double(x) extern double FLAGS_##x

void miniflags_init(int argc, char **argv);

} /* miniflags */

#endif /* end of include guard: MINIFLAGS_H_16MUCQ2A */
