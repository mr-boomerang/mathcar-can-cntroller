#pragma once

#include <iostream>
#include <map>
#include <string>

#include <utils/str_utils.h>

/** @file */

class Params; // forward declaration of ParamT.

#define PENS params_enum       // params_namespace.
#define PENS_STR "params_enum" // params_namespace.

// this namespace is supposed to have only Enum related classes.
namespace PENS {

//! Parent class to all the Enum class defined by DEFINE_PARAMS_ENUM.
/*!
 * Primary reason for existence of this class is to be parent of Enum classes
 * This allows for EnumBase* to be a value type for the map in which sample
 * object of * all the enum types declared would be stored. This helps us add
 * Enums to params even while reading from a file.
 */
class EnumBase {
public:
  //! Add object of derived type to param map.
  /*!
   * This virtual method forces the enum classes to define a method to
   * add that type of enum to the param map. Since, this is a virtual method,
   * calling this from EnumBase* will cause it to execute the dervied method,
   * which has knowledge of it's type and hence can add the parameter of correct
   * type.
   * \param param_name name the param should be identified with.
   * \param val value with which param should initialized.
   * \param p reference to the params singleton object.
   *
   */
  virtual int add_derived_as_param(const std::string &param_name,
                                   const std::string &val, Params &p) = 0;

private:
  /* data */
};

//! Template struct to identify if the type is a Params Enum or not.
/*!
 * This is the generic implementation, this defaults to false.
 * \code{.cpp}
 * params_enum::is_enum<int>::value //this will be false.
 * params_enum::is_enum<float>::value //this will be false.
 * params_enum::is_enum<std::string>::value //this will be false.
 * \endcode
 */
template <typename T, typename V = bool> struct is_enum : std::false_type {};

//! Template struct to identify if the type is a Params Enum or not.
/*!
 * This is the specialized implementation, this return true.
 * \code{.cpp}
   * DEFINE_PARAMS_ENUM(Animal, Cat, Dog);
   * DEFINE_PARAMS_ENUM(Color, Red, Green);
 * params_enum::is_enum<Animal>::value //this will be true.
 * params_enum::is_enum<Color>::value //this will be true.
 * \endcode
 */
template <typename T>
struct is_enum<
    T, typename std::enable_if<
           (!std::is_same<typename T::_enum, void>::value || // first condition
            !std::is_base_of<EnumBase, T>::value),           // second condition
           bool                                              // type
           >::type> : std::true_type {};

//! Map to hold type (string) vs an instance
/*!
 * This map has key type as std::string, which should contain demangled name of
 * the Params Enum defined (for e.g. params_enum::Color, params_enum::Animal),
 * it's map type is pointer to EnumBase, which is an instance of defined Enum.
 * \code{.cpp}
   * DEFINE_PARAMS_ENUM(Animal, Cat, Dog);
   * DEFINE_PARAMS_ENUM(Color, Red, Green);
 * \endcode
 *
 * Above e.g. would result in two elements in the map.
 *
 */
extern std::map<std::string, EnumBase *> _type_enum_map;

} /*  PENS */

//! Get the first arg in the Vardic arguments.
#define FIRST_ARG(x, ...) x

//! Define an Param Enum type.
/*!
 * This macro defines a class which contains the enum being declared. But, since
 * it's a class and not just and enum, it provides extra functionalities, such
 * as setting values from a string, outputting the value to a string, so
 * Animal::Horse would be printed as Horse and not some arbitary integer. It can
 * also provied all the values which has been declared in the Enum, as enum and
 * as string, and can append it to ostream.
 * It's various constructors and operators make the class behave as an enum
 * would have.
 * \code{.cpp}
   * DEFINE_PARAMS_ENUM(Animal, Cat, Dog); //Correct initialization
   * //Incorrect initialization, will cause compilation error.
   * DEFINE_PARAMS_ENUM(Color, Red = 1, Green, Blue);
   *
   * int main() {
   *   params_enum::Animal a = params_enum::Animal::Dog;
   *   params_enum::Animal b;//defualt initialization to Cat.
   *   params_enum::Animal c = params_enum::Animal::from_string("Dog");
   *   params_enum::Animal d(params_enum::Animal::Dog);
   *   params_enum::Animal e("Dog");
   *   e.set_from_string("Cat");
   *   params_enum::Animal* f = new params_enum::Animal("Dog");
   *
   *   params_enum::Animal::_enum enumerted = a; //enumerated = Dog.
   *
   *   std::cout << a << std::endl; //Will print Dog
   *   std::cout << (a == c) << std::endl; //1 (that is true)
   *   std::cout << (a == b) << std::endl; //0 (that is false)
   *   std::cout << (a == params_enum::Animal::Dog) << std::endl; //1
   *   std::cout << (a == params_enum::Animal::Cat) << std::endl; //0
   *
   *   int x = a;//x = 1.
   *   std::string s = c.to_string(); //s = "Dog"
   *
   *   std::vector<params_enum::Animal> va = *
 * params_enum::Animal::all_values_enum();
   *   std::vector<std::string> vs = params_enum::Animal::all_values_names();
   *   //following will print Cat;Dog (no new line added for stringstreams.)
   *   params_enum::Animal::all_values_stream(std::cout, ";");
   *
   *   params_utils.set("animal1", a);
   *   //The following would work, but not as Param Enum, i.e. you can't set
   *   //values from strings, read value from file and assign, for all the
   *   //features use the above method.
   *   params_utils.set("animal2", params_enum::Animal::Dog);
   *
   *   //Pay attention to params_type before Animal
   *   params_type::Animal pa = param_utils.get<params_enum::Animal>("animal1");
   *
   * }
 * \endcode
 *
 */
#define DEFINE_PARAMS_ENUM(Name, ...)                                          \
  namespace PENS {                                                             \
  class Name : public EnumBase {                                               \
  public:                                                                      \
    enum _enum { __VA_ARGS__ };                                                \
                                                                               \
    class execption : std::exception {};                                       \
    Name() {                                                                   \
      val_ = FIRST_ARG(__VA_ARGS__);                                           \
      init_class(get_name_enum_map(), get_enum_name_map());                    \
    }                                                                          \
                                                                               \
    Name(_enum e) {                                                            \
      val_ = e;                                                                \
      init_class(get_name_enum_map(), get_enum_name_map());                    \
    }                                                                          \
                                                                               \
    Name(const std::string &enum_str) {                                        \
      init_class(get_name_enum_map(), get_enum_name_map());                    \
      set_from_string(enum_str);                                               \
    }                                                                          \
                                                                               \
    std::string to_string() const { return get_enum_name_map()[val_]; }        \
                                                                               \
    bool operator==(const _enum &e) const { return (e == val_); }              \
                                                                               \
    bool operator==(const Name &n) const { return (val_ == n.val_); }          \
                                                                               \
    operator _enum() { return val_; }                                          \
                                                                               \
    friend std::ostream &operator<<(std::ostream &os, const Name &n) {         \
      os << n.to_string();                                                     \
      return os;                                                               \
    }                                                                          \
    int add_derived_as_param(const std::string &param_name,                    \
                             const std::string &val, Params &p) {              \
      Name n;                                                                  \
      if (!n.set_from_string_noexcept(val))                                    \
        return -1;                                                             \
      return p.set(param_name, Name::from_string(val));                        \
    }                                                                          \
                                                                               \
    static std::vector<_enum> all_values_enum() {                              \
      std::vector<_enum> ret;                                                  \
      std::map<_enum, std::string> &enum_name = get_enum_name_map();           \
      if (enum_name.size() == 0)                                               \
        init_class(get_name_enum_map(), get_enum_name_map());                  \
      std::map<_enum, std::string>::reverse_iterator elem =                    \
          enum_name.rbegin();                                                  \
      std::map<_enum, std::string>::reverse_iterator last = enum_name.rend();  \
      for (; elem != last; ++elem) {                                           \
        ret.push_back(elem->first);                                            \
      }                                                                        \
      return ret;                                                              \
    }                                                                          \
                                                                               \
    static std::vector<std::string> all_values_names() {                       \
      std::vector<std::string> ret;                                            \
      std::map<std::string, _enum> &name_enum = get_name_enum_map();           \
      if (name_enum.size() == 0)                                               \
        init_class(get_name_enum_map(), get_enum_name_map());                  \
      std::map<std::string, _enum>::reverse_iterator elem =                    \
          name_enum.rbegin();                                                  \
      std::map<std::string, _enum>::reverse_iterator last = name_enum.rend();  \
      for (; elem != last; ++elem) {                                           \
        ret.push_back(elem->first);                                            \
      }                                                                        \
      return ret;                                                              \
    }                                                                          \
                                                                               \
    static void all_values_stream(std::ostream &os,                            \
                                  const std::string &delim) {                  \
      std::map<std::string, _enum> &name_enum = get_name_enum_map();           \
      if (name_enum.size() == 0)                                               \
        init_class(get_name_enum_map(), get_enum_name_map());                  \
      std::map<std::string, _enum>::reverse_iterator elem =                    \
          name_enum.rbegin();                                                  \
      std::map<std::string, _enum>::reverse_iterator last = name_enum.rend();  \
      last--;                                                                  \
      for (; elem != last; ++elem) {                                           \
        os << elem->first << delim;                                            \
      }                                                                        \
      os << elem->first;                                                       \
    }                                                                          \
                                                                               \
    static void init_class(std::map<std::string, _enum> &name_enum,            \
                           std::map<_enum, std::string> &enum_name) {          \
      static bool init_done_ = false;                                          \
      if (init_done_)                                                          \
        return;                                                                \
      std::string str = #__VA_ARGS__;                                          \
      vec_str tokens = utils::split_str<std::string>(str, ",");                \
      std::vector<_enum> vec_enum{__VA_ARGS__};                                \
      for (size_t i = 0; i < tokens.size(); i++) {                             \
        utils::trim(tokens[i]);                                                \
        name_enum[tokens[i]] = vec_enum[i];                                    \
        enum_name[vec_enum[i]] = tokens[i];                                    \
      }                                                                        \
      init_done_ = true;                                                       \
      std::string typ = utils::demangle_type(typeid(Name).name());             \
      _type_enum_map[typ] = new Name();                                        \
    }                                                                          \
                                                                               \
    bool set_from_string_noexcept(const std::string &str) {                    \
      std::map<std::string, _enum> name_enum = get_name_enum_map();            \
      if (name_enum.find(str) != name_enum.end())                              \
        val_ = name_enum[str];                                                 \
      else                                                                     \
        return false;                                                          \
      return true;                                                             \
    }                                                                          \
                                                                               \
    void set_from_string(const std::string &str) {                             \
      std::map<std::string, _enum> name_enum = get_name_enum_map();            \
      if (name_enum.find(str) != name_enum.end())                              \
        val_ = name_enum[str];                                                 \
      else                                                                     \
        throw Name::execption();                                               \
    }                                                                          \
                                                                               \
    static Name from_string(const std::string &str) {                          \
      Name n;                                                                  \
      n.set_from_string(str);                                                  \
      return n;                                                                \
    }                                                                          \
                                                                               \
  private:                                                                     \
    _enum val_;                                                                \
                                                                               \
    static std::map<_enum, std::string> &get_enum_name_map() {                 \
      static std::map<_enum, std::string> en;                                  \
      return en;                                                               \
    }                                                                          \
                                                                               \
    static std::map<std::string, _enum> &get_name_enum_map() {                 \
      static std::map<std::string, _enum> ne;                                  \
      return ne;                                                               \
    }                                                                          \
  };                                                                           \
  static Name map_init_for_##Name;                                             \
  }                                                                            \
  namespace params_type {                                                      \
  typedef const PENS::Name &Name;                                              \
  }

#if 0
DEFINE_ENUM(Animal, TRex, Mouse, Cat, Dog, Horse);

int main(void)
{
  PENS::Animal e;
  std::cout << "Animal.e = " << e.to_string() << std::endl;
  e = PENS::Animal::Cat;
  std::cout << "Animal.e2 = " << e.to_string() << std::endl;
  PENS::Animal* ep = new PENS::Animal("Horse");
  std::cout << "Animal.ep = " << ep->to_string() << std::endl;
  PENS::Animal a = PENS::Animal::Dog;
  std::cout << "Animal.a = " << a.to_string() << std::endl;

  return 0;
}
#endif
