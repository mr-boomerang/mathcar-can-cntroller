#pragma once

// http://stackoverflow.com/questions/87372/check-if-a-class-has-a-member-function-of-a-given-signature
// not the accepted answer at the time of writing this
#include <iostream>
#include <type_traits>
#include <utils/mode_printing.h>
// Primary template with a static assertion
// for a meaningful error message
// if it ever gets instantiated.
// We could leave it undefined if we didn't care.

#define DEFINE_FUNC_CHECKER(func_name)                                         \
  namespace checker {                                                          \
  template <typename, typename T> struct has_##func_name {                     \
    static_assert(std::integral_constant<T, false>::value,                     \
                  "Second template parameter needs to be of function type.");  \
  };                                                                           \
  template <typename C, typename Ret, typename... Args>                        \
  struct has_##func_name<C, Ret(Args...)> {                                    \
  private:                                                                     \
    template <typename T>                                                      \
    static constexpr auto check(T *) -> typename std::is_same<                 \
        decltype(std::declval<T>().func_name(std::declval<Args>()...)),        \
        Ret>::type;                                                            \
    template <typename> static constexpr std::false_type check(...);           \
    typedef decltype(check<C>(0)) type;                                        \
                                                                               \
  public:                                                                      \
    static constexpr bool value = type::value;                                 \
  };                                                                           \
  }

namespace checker {
// check if ostream objects exists or not.
template <class T>
static auto test_ostream(int) -> typename std::is_arithmetic<T>::type;

template <class T>
static auto test_ostream(int) -> typename std::enable_if<
    std::is_same<decltype(std::declval<std::ostream &>() << std::declval<T>()),
                 std::ostream &>::value &&
        !std::is_arithmetic<T>::value,
    std::true_type>::type;

template <class T>
static auto test_ostream(int) ->
    typename std::enable_if<std::is_same<decltype(std::declval<std::ostream &>()
                                                  << std::declval<T &>()),
                                         std::ostream &>::value &&
                                !std::is_arithmetic<T>::value,
                            std::true_type>::type;

template <class T>
static auto test_ostream(int) ->
    typename std::enable_if<std::is_same<decltype(std::declval<std::ostream &>()
                                                  << std::declval<const T &>()),
                                         std::ostream &>::value &&
                                !std::is_arithmetic<T>::value,
                            std::true_type>::type;

template <class> static auto test_ostream(long) -> std::false_type;

template <class T> struct has_ostream : decltype(test_ostream<T>(0)) {};
template <> struct has_ostream<std::string> : std::true_type {};
template <> struct has_ostream<const char *> : std::true_type {};

// check if istream objects exists or not.
template <class T>
static auto test_istream(int) ->
    typename std::is_same<decltype(std::declval<std::istream &>() >>
                                   std::declval<T>()),
                          std::istream &>::type;
template <class> static auto test_istream(long) -> std::false_type;

template <class T> struct has_istream : decltype(test_istream<T>(0)) {};

} // checker::

// this should be always 0, if executable code needs to be compiled created a
// seperate cpp file for it, but this code remains commented and just for
// reference.
#define FUNC_CHECKER_EXAMPLE 0
#if FUNC_CHECKER_EXAMPLE
DEFINE_FUNC_CHECKER(seerial);

struct X {
  int seerial(const std::string &) { return 42; }
};

struct Y : X {};

int main(void) {
  PN$ has_seerial<Y, int(const std::string &)>::value pendl; // will print 1
  PN$ has_seerial<Y, int(const std::string &)>::value pendl; // will print 1

  PN$ "int: " << has_stream<int>::value pendl;
  // define class blue in executable code.
  PN$ "blank class named blue: " << has_stream<blue>::value pendl;
  PN$ "std::vector: " << has_stream<std::vector<int>>::value pendl;
  return 0;
}
#endif
