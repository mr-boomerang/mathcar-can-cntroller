#pragma once

#include <utils/TicToc.h>
#include <utils/check_ret.h>
#include <utils/fs_utils.h>
#include <utils/mode_printing.h>
#include <utils/str_utils.h>

#include <map>

#ifdef BOOST_NOT_CPP11
#include <boost/type_traits/is_arithmetic.hpp>
#include <boost/type_traits/is_integral.hpp>
#include <boost/type_traits/is_signed.hpp>
#include <boost/utility/enable_if.hpp>
#else
#include <type_traits>
#endif
#include <utils/boostd.h>
#include <utils/function_checker.hpp> //This will work with c++11

#include <set>
#include <typeinfo>

#include <utils/params_enum.hpp>
#ifdef BUILT_WITH_ROS
#include <utils/ros/params_ros.hpp>
#define PUBLISH_PARAMS                                                         \
  0 // change PUBLISH_PARAMS to 1 for publishing parameters updated in
    // swahana_master gui
#endif
const std::string kParamDelim = "[]";
const std::string kParamEnumDelim = "()";
const std::string kParamArrayDelim = "{}";
const std::string kParamSeperator = ",";

/** @file */

//! Macro to invoke params instance.
#define params_utils Params::instance()

//! Macro to make it easier to know which function exception occured.
#define PARAM_EXCEPT_MSG_LONG(nam, msg, func, line, file)                      \
  ParamExcept(std::string("(func: ") + std::string(func) +                     \
              std::string(", line: ") + utils::val_to_str(line) +              \
              std::string(", file: ") + utils::basename(file) +                \
              std::string(", param: ") + nam + std::string(") ") +             \
              std::string(msg))

#define PARAM_EXCEPT_MSG(nam, msg)                                             \
  PARAM_EXCEPT_MSG_LONG(nam, msg, __func__, __LINE__, __FILE__)

//! Macro to make it easier to check for invalid state of param, return in case
// invalid
#define RET_IF_INVALID(x)                                                      \
  if (ParamCommon<T>::invalid_state_) {                                        \
    return x;                                                                  \
  }

//! Macro to make it easier to check for invalid state of param, throw in case
// invalid
#define THROW_IF_INVALID                                                       \
  if (ParamCommon<T>::invalid_state_) {                                        \
    throw PARAM_EXCEPT_MSG(ParamCommon<T>::name_,                              \
                           "Param is in invalid state.");                      \
  }

//! Macro to make it easier to check for num of elems in param, i.e sz_
#define THROW_IF_IDX_OOB(idx)                                                  \
  if (idx >= ParamCommon<T>::sz_) {                                            \
    PE$ ct_vred(idx) pcommas ct_vred(ParamCommon<T>::sz_) pendl;               \
    throw PARAM_EXCEPT_MSG(ParamCommon<T>::name_, "Out of index");             \
  }

//! Macro to make it easier to check for num of elems in param, i.e sz_
#define THROW_IF_NOT_ARRAY                                                     \
  if (ParamCommon<T>::not_arr_) {                                              \
    throw PARAM_EXCEPT_MSG(ParamCommon<T>::name_, "Param is not an array.");   \
  }

//! Macro to make it easier to check for num of elems in param, i.e sz_
#define THROW_IF_ARRAY                                                         \
  if (!ParamCommon<T>::not_arr_) {                                             \
    std::string msggggg = "Param is an array with size = " +                   \
                          std::to_string(ParamCommon<T>::sz_);                 \
    throw PARAM_EXCEPT_MSG(ParamCommon<T>::name_, msggggg);                    \
  }

//! Macro to make it easier to call find_if_not_throw function
#define FIND_IF_NOT_THROW(T, var_name)                                         \
  find_if_not_throw<T>(var_name, __func__, __LINE__, __FILE__)
//! Macro to make it easier to call find_if_not_nothrow function
#define FIND_IF_NOT_NOTHROW(T, var_name)                                       \
  find_if_not_nothrow<T>(var_name, __func__, __LINE__, __FILE__)
#define FIND_IF_NOT_NOTHROW_NOVERBOSE(T, var_name)                             \
  find_if_not_nothrow<T>(var_name, __func__, __LINE__, __FILE__, false)

//! Types defined for params.
/*!
 * typedefs such as:\n
 * int => Int\n
 * float => Float...\n
 * \namespace params_type Namespace housing typedefs for convineint use of
 * params.
 */
namespace params_type {}

/*!
 * Macro to add types to param_type namespace to make using params less error
 * prone.
 */
#define DEFINE_PARAMS_TYPE(type, name)                                         \
  namespace params_type {                                                      \
  typedef const type &name;                                                    \
  /*typedef x& NonConst##name;*/                                               \
  }
DEFINE_PARAMS_TYPE(bool, Bool);
DEFINE_PARAMS_TYPE(int, Int);
DEFINE_PARAMS_TYPE(short, Short);
DEFINE_PARAMS_TYPE(long, Long);
DEFINE_PARAMS_TYPE(long long, LongLong);
DEFINE_PARAMS_TYPE(unsigned, UnsignedInt);
DEFINE_PARAMS_TYPE(unsigned short, UnsignedShort);
DEFINE_PARAMS_TYPE(unsigned long, UnsignedLong);
DEFINE_PARAMS_TYPE(unsigned long long, UnsignedLongLong);
DEFINE_PARAMS_TYPE(char, Char);
DEFINE_PARAMS_TYPE(signed char, SignedChar);
DEFINE_PARAMS_TYPE(unsigned char, UnsignedChar);
DEFINE_PARAMS_TYPE(float, Float);
DEFINE_PARAMS_TYPE(double, Double);
DEFINE_PARAMS_TYPE(long double, LongDouble);
DEFINE_PARAMS_TYPE(std::string, String);

template <typename T>
using ParamValidatorFunctor = std::function<bool(const T &)>;

//! Signature for functor to be called, 1st arg is new val, 2nd arg is old val.
template <typename T>
using ParamFunctor = std::function<void(const T &, const T &)>;

template <typename ret, typename T> using ParamSetter = std::function<ret(T &)>;

template <typename T> struct ParamSetterT {
  virtual void operator()(T &val) const = 0;
};

template <typename T, typename ret>
struct ParamSetterProxy : public ParamSetterT<T> {
public:
  ParamSetterProxy(const ParamSetter<ret, T> &func) : func_(func) {}
  void operator()(T &val) const { func_(val); }

private:
  // keepint T& lets function to be set with T, T&, const T&
  ParamSetter<ret, T> func_;
};

//! Exception class to be thrown in case of invalid use of params_utils.
/*!
 * Exception class thrown by function in Params class. Default constructor takes
 * in string object which can be read by what() function provided by the
 * std::exception class.
 */
class ParamExcept : public std::exception {
public:
  explicit ParamExcept(const std::string &msg) : msg_(msg) {}
  //! Get the message from exception
  /*!
   * \return The reason exception was caused.
   */
  virtual const char *what() const throw() { return msg_.c_str(); }
  virtual ~ParamExcept() throw() {}

private:
  /* data */
  std::string msg_;
};

//! Parent class to all the templated param class.
/*!
 * Primary reason for existence of this class is to be parent of Param class
 * which is templated. This allows for ParamT* to be a value type for the map in
 * params would be stored, ow a map would need to be defined for int, one
 * for float etc....which is not a scalable.
 */
class ParamT {
public:
  virtual ~ParamT() {}
  //! serialize the param value and properties.
  /*!
   * This virtual method forces the param classes to define a method to
   * serialize the param, which enables printing on screen and writing to file.
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param os out stream where the serialization should be directed to
   * cout/cerr for printing and stringstream for file writing.
   *
   */
  virtual void to_stream(ParamT *p, std::ostream &os) = 0;

  //! serialize the param value.
  /*!
   * This virtual method forces the param classes to define a method to
   * serialize just the param values, which enables printing on screen and
   * writing to file.
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param val value would be returned in this parameter.
   *
   */
  virtual void get_string(ParamT *p, std::string *val) = 0;
  virtual void get_string_full(ParamT *p, std::string *val) = 0;

  //! Tells if the value of Param was changed.
  bool has_changed() {
    bool changed = changed_;
    changed_ = false;
    // This statement is there for debugging, but it's expected it will be
    // called in loops, which will be a bother always, so keeping it
    // commented is the decided way to proceed.
    // PT$ ct_vpur(changed_) pcommas ct_vpur(changed) pendl;
    return changed;
  }

  //! Tells if the value of Param was changed.
  void set_changed(bool val) {
    changed_ = val;
    PI$ ct_vpur(changed_) pendl;
  }

  //! has ostream operator or no.
  const bool has_ostream_op;

protected:
  /* constructor */
  ParamT(bool changed = false, bool stream_op = false)
      : has_ostream_op(stream_op), changed_(changed) {}
  /* data */
  bool changed_;
};

//! Class for all the common things params will have.
/*!
 * Stores variable value, links, setters, validators which all the Params will
 * need and would share the implementation in most cases.
 */
template <class T> class ParamCommon : public ParamT {
public:
  //! to make ParamCommon abstract.
  virtual void pure_virtualizer() = 0;

  /* common functions */

  //! Get reference to the param value.
  /*!
   * This allows us to read parameter value in an efficient way, no need to
   * check if the value has changed at every iteration, and no need to update
   * upon a change in parameter, a change in value will be reflected
   * automatically. But this won't be thread safe.
   * \return const reference to parameter.
   * \throws ParamExcept When parameter is invalid.
   */
  // get mutex lock.
  const T &get() {
    THROW_IF_INVALID;
    return var_[0];
  }
  //! Get reference to the param value.
  /*!
   * Same as the get() function, just that in case parameter is an array, it
   * takes in index to return the reference of.
   * \return const reference to parameter at index.
   * \throws ParamExcept When parameter is invalid.
   * \throws ParamExcept When parameter is not an array.
   * \throws ParamExcept When index is Out of bounds.
   */
  const T &get(int idx) {
    THROW_IF_INVALID;
    THROW_IF_NOT_ARRAY;
    THROW_IF_IDX_OOB(idx);
    return var_[idx];
  }
  //! Get pointer to the param value.
  /*!
   * Same as the get() function, rather than function it returns pointer
   * \return const pointer to parameter or 0th val in case of array.
   * \throws ParamExcept When parameter is invalid.
   */
  const T *get_ptr() {
    THROW_IF_INVALID;
    return var_;
  }

  //! Get param value.
  /*!
   * This allows us to retrieve parameter value. This will require user to check
   * and update values at their own accord. This can be thread safe, but isn't
   * yet.
   * \return value of the parameter
   * \throws ParamExcept When parameter is invalid.
   * \throws ParamExcept When parameter is not an array.
   * \throws ParamExcept When index is Out of bounds.
   */
  T get_val(int idx = 0) {
    THROW_IF_INVALID;
    THROW_IF_IDX_OOB(idx);
    ParamT::changed_ = false;
    return var_[idx];
  }

  //! Set param interface for array parameters
  /*!
   * A typical setter for parameter value.
   * \param val const reference to the value param should be set to.
   * \return See set_common function
   * \throws ParamExcept When parameter is invalid.
   * \throws ParamExcept When index is Out of bounds.
   */
  int set(const T &val, int idx = 0) {
    THROW_IF_INVALID;
    THROW_IF_IDX_OOB(idx);
    return set_common(val, idx);
  }

  //! Set function pointer for validating the param value.
  /*!
   * Sets the function pointer (std::function<bool(const T&)>) for the
   * parameter, which can be used for validation or trigerring certain other
   * events in turn.
   * NOTE: this is called before setting the variable, if it returns false the
   * value is not set. Should be kept in mind while triggering other events.
   * \param functor std::function object for callback.
   * \return 0 on success
             1 on invalid param,
             -1 on exception thrown from below.
   * \throws When parameter is in invalid state.
   */
  int set_validator(const ParamValidatorFunctor<T> &functor) {
    RET_IF_INVALID(1);
    try {
      validator_ = functor;
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
    return RET_SUCCESS_INT;
  }

  //! Set function pointer for function to call at value set.
  /*!
   * Sets the function pointer (std::function<void(const T&)>) for the
   * parameter, which can be used for trigerring certain other
   * events in turn.
   * NOTE: this is called before setting the variable, if it returns false the
   * value is not set. Should be kept in mind while triggering other events.
   * \param functor std::function object for callback.
   * \return 0 on success
             1 on invalid param,
             -1 on exception thrown from below.
   * \throws When parameter is in invalid state.
   */
  int set_functor(const ParamFunctor<T> &functor) {
    RET_IF_INVALID(1);
    try {
      func_ = functor;
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
    return RET_SUCCESS_INT;
  }

  //! Link variables to the parameter.
  /*!
   * This allows us to create a function callback for variables kind of work
   * flow. Once variable is linked to the parameter it's value will be updated
   * once the parameters value changes. Multiple variables can be linked to a
   * parameter.
   * \param var Pointer to the variable whose value should change on change in
   * param value.
   * \return 0 on success
             1 if PARAM is in invalid state
             2 if var is NULL
             -1 on exception thrown by code below.
   * \throws When parameter is an array.
   */
  int link(T *link_var) {
    RET_IF_INVALID(1);
    THROW_IF_ARRAY;
    try {
      if (link_var == NULL) {
        PE$ "Trying to link param with null " << pendo;
        return RET_LINKING_PARAM_TO_NULL;
      }
      if (!link_var_set_)
        link_var_set_ = std::unique_ptr<std::set<T *>>(new std::set<T *>());
      link_var_set_->insert(link_var);
      *link_var = var_[0];
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
    return RET_SUCCESS_INT;
  }

  //! Unlink variable from the parameter.
  /*!
   * This allows us to create a function callback for variables kind of work
   * flow. Once variable is linked to the parameter it's value will be updated
   * once the parameters value changes. Multiple variables can be linked to a
   * parameter.
   * \param var Pointer to the variable whose value was changing on change in
   * param value, which should no longer be the case.
   * \return 0 on success
             1 if PARAM is in invalid state
             2 if var is NULL
             3 pointer not intialized.
             4 if pointer not found
             -1 on exception thrown by code
   * \throws When parameter is an array.
   */
  int unlink(T *link_var) {
    RET_IF_INVALID(1);
    THROW_IF_ARRAY;
    try {
      if (link_var == NULL) {
        PE$ "Trying to unlink null " << pendo;
        return RET_UNLINKING_PARAM_TO_NULL;
      }
      if (!link_var_set_)
        return RET_LINK_VAR_SET_NULL;
      typename std::set<T *>::iterator pos = link_var_set_->find(link_var);
      if (pos == link_var_set_->end())
        return RET_LINK_VAR_NOT_FOUND;
      link_var_set_->erase(pos);
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
    return RET_SUCCESS_INT;
  }

  //! Set function pointer for setters.
  /*!
   * Sets the function pointer (std::function<ret(T&)>) for the
   * parameter, which can be used for calling setter interface provided by the
   * class which the parameter should be linked with.
   * \param functor std::function object for callback.
   * \return 0 on success
             1 on invalid param,
             -1 on exception thrown from below.
   * \throws When parameter is an array.
   */
  template <typename ret> int add_setter(const ParamSetter<ret, T> &functor) {
    RET_IF_INVALID(1);
    THROW_IF_ARRAY;
    try {
      if (!setters_) {
        typedef std::unique_ptr<std::vector<ParamSetterT<T> *>>
            ParamSetterTVecPtr;
        ParamSetterTVecPtr pstvp(new std::vector<ParamSetterT<T> *>());
        setters_ = std::move(pstvp);
      }
      ParamSetterT<T> *setter = new ParamSetterProxy<T, ret>(functor);
      setters_->push_back(setter);
      functor(var_[0]);
      return RET_SUCCESS_INT;
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
  }

  int size() { return sz_; }

  std::string name() { return name_; }

protected:
  /* constructor */
  // changed is initially false as while construction param is in invalid state,
  // so reading it doesn't make sense, if changed is true then user might
  // attempt
  // to read it.
  ParamCommon(const std::string &name, const T *val, bool has_ostream,
              int sz = 0, bool invalid_state = true)
      : ParamT(false, has_ostream), // has_changed, has_ostream_op
        sz_((sz > 0) ? sz : 1), not_arr_(sz_ == 1), var_(new T[sz_]),
        invalid_state_(invalid_state), name_(name) {

    PI$ ct_vcyn(not_arr_) pcommas ct_vcyn(sz_) pcommas ct_vcyn(name) pendl;
    for (int i = 0; i < sz; ++i) {
      set_common(val[i], i);
    }
  }

  /* data */
  const int sz_;
  const bool not_arr_;
  T *const var_; //!< To hold variable value.
  bool invalid_state_;
  const std::string name_; // to print in debug messages.

  /* connections */
  // These are unique_ptr so that they only get created if those functions are
  // called, which will be selectively enabled.
  // link var and setters are stl containers to allow linking to multiple
  // variables and setters, not so tha tit can accomodate parama arrays.
  std::unique_ptr<std::set<T *>> link_var_set_;
  ParamValidatorFunctor<T> validator_;
  ParamFunctor<T> func_;
  std::unique_ptr<std::vector<ParamSetterT<T> *>> setters_;

private:
  /* functions */
  //! Change value for all the variables linked to this parameter.
  int set_links() {
    for (T *v : *link_var_set_) {
      *v = var_[0];
    }
    return RET_SUCCESS_INT;
  }

  //! Call the setter functions added for this parameter.
  int call_setters() {
    for (size_t i = 0; i < setters_->size(); ++i) {
      setters_->at(i)->operator()(var_[0]);
    }
    return RET_SUCCESS_INT;
  }

  //! Set param value, like, really set it.
  /*!
   * A typical setter for parameter value.
   * \param val const reference to the value param should be set to.
   * \return 0 on success
             1 if validator return false
             -1 exception thrown by code below
   */
  int set_common(const T &val, int idx) {
    const int sn = 112;
    try {
      bool validator_ret = true;
      if (validator_)
        validator_ret = validator_(val);
      SP$(sn) ct_vpur(validator_ret) pendl;

      if (!validator_ret)
        return RET_FALSE_VALIDATOR;

      T old_val = var_[idx];
      var_[idx] = val;
      SP$(sn) ct_pur("val set") pendl;
      if (link_var_set_) {
        set_links();
        SP$(sn) ct_pur("links set") pendl;
      }
      if (setters_) {
        call_setters();
        SP$(sn) ct_pur("setters called") pendl;
      }
      // May be do it in thread, but what if it doesn't exit?
      if (func_) {
        func_(var_[idx], old_val);
        SP$(sn) ct_pur("functor called") pendl;
      }

      ParamT::changed_ = true;
      invalid_state_ = false;
    } catch (std::exception e) {
      PE$ "Error while setting value " << pendo;
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }

    return RET_SUCCESS_INT;
  }
};

//! Template class to store value and properties of params.
/*!
 * Stores variable value and properties, generic type doesn't have any
 * properties of note but arithmetic type do, such as min, max, and interval.
 * Hence is templated by two typenames first T is param type, second is a int
 * coding cases requiring different treatment. Code is as follows:
 * true: Arithmetic type
 * false: General
 */
// template <class T, bool Arithmetic=utils_boostd::is_arithmetic<T>::value>
// class Param;
template <class T, typename Enable = void> class Param;

#define COND_ARITH(typ) std::is_arithmetic<typ>::value
//! Store value and properties of params of arithmetic type.
/*!
 * Class defined for parameters with arithmetic type
 * (http://en.cppreference.com/w/cpp/types/is_arithmetic)
 * This types usually have orders, and more often than not these parameters
 * would have a min value and a max value, for e.g. pixel value will have min =
 * 0 and max = 255, interval as 1, Or a param representing probability with min
 * = 0 and max = 1.0 with interval as 0.01.
 */
template <class T> class Param<T, ARITHMETIC(T)> : public ParamCommon<T> {
public:
  //! Constructor for arithmetic type param.
  /*!
   * One and only constructor which sets all the members (val, min, max,
   * interval) of the class. No default constructor available.
   */
  Param(const std::string &name, const T *val, int sz)
      : ParamCommon<T>(name, val, true, sz, true), invalid_properties_(true) {
    PI$ ct_vcyn(name) pcommas ct_vcyn(ParamT::has_ostream_op)
        pcommas ct_vcyn(checker::has_ostream<T>::value) pendl;
  }
  //! to make ParamCommon abstract and Param not abstract.
  void pure_virtualizer() {}

  //! serialize the param value and properties.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * OUTPUT FORMAT: [type,val,min,max,interval]
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param os out stream where the serialization should be directed to
   * cout/cerr for printing and stringstream for file writing.
   */
  void to_stream(ParamT *p, std::ostream &os) {
    RET_IF_INVALID();
    // since second will be T as well if it's arithmetic.
    Param<T, T> *ptr = static_cast<Param<T, T> *>(p);
    os << kParamDelim[0] << utils::demangle_type(typeid(T).name())
       << kParamSeperator;
    if (ParamCommon<T>::not_arr_) {
      os << ptr->get_val();
    } else {
      os << kParamArrayDelim[0];
      for (int i = 0; i < ParamCommon<T>::sz_; i++) {
        os << ptr->get_val(i);
        if (i != (ParamCommon<T>::sz_ - 1))
          os << kParamSeperator;
      }
      os << kParamArrayDelim[1];
    }
    os << kParamSeperator << ptr->get_min() << kParamSeperator << ptr->get_max()
       << kParamSeperator << ptr->get_interval() << kParamDelim[1];
  }

  //! serialize the param value.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param str out string where the serialization should be directed to.
   */
  void get_string(ParamT *p, std::string *str) {
    RET_IF_INVALID();
    Param<T, T> *ptr = static_cast<Param<T, T> *>(p);
    std::stringstream ss;
    ss << ptr->get_val();
    *str = ss.str();
  }

  void get_string_full(ParamT *p, std::string *str) {
    std::stringstream ss;
    to_stream(p, ss);
    *str = ss.str();
  }

  //! Set param value.
  /*!
   * A typical setter for parameter value. This validates the value input by
   * comparing it with min and max, hence providing a safe gateway to changing
   * param values.
   * NOTE: Until this method is called atleast once, parameter won't go in valid
   * state.
   * \param val const reference to the value param should be set to.
   * \return 0 on success
             1 Error in ParamCommon<T>::set
             2 if closes value was set
             3 if val is out of bounds
             -1 exception thrown by code below
   */
  int set(const T &val, int idx = 0) {
    PRINT_FUNC_ENTER;
    int ret = 0;
    const int sn = 111;
    try {
      SP$(sn) ct_vred(ParamCommon<T>::name_) pendl;
      SP$(sn) ct_vylw(val) pendl;
      SP$(sn) ct_vylw(invalid_properties_) pendl;
      if (invalid_properties_) {
        guess_properties(val);
      }

      SP$(sn)
      ct_vblu(max_) pcommas ct_vblu(min_) pcommas ct_vblu(interval_) pendl;
      if (val > max_ || val < min_)
        return RET_OUT_OF_BOUND_PARAM;

      T lv = val; // legal value
      if (interval_ != static_cast<T>(0)) {
        ret = get_closest_lv(val, &lv); // ret 2 if val changed, 0 ow
        cPI$(
            ret ==
            RET_CLOSEST_LEGAL_PARAM) "Value moved to closest legal value" pendl;
      }
      SP$(sn) ct_vblu(lv) pendl;

      ret = ParamCommon<T>::set(lv, idx);
      SP$(sn) ct_vblu(ret) pendl;

    } catch (ParamExcept e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_PARAM_EXCEPTION;
    } catch (std::exception e) {
      PE$ "Error while setting value " << pendo;
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }

    PRINT_FUNC_EXIT;
    return ret;
  }

  //! Set param properties (min, max, interval).
  /*!
   * Setter for parameter properties. All properties should be set via this
   * single interface, individual setters are not provided. A single interface
   * allows checking of validity or relationships between properties, such as
   * min < max, interval < max etc...
   * \param min const reference to the min value param can be set to.
   * \param max const reference to the max value param can be set to.
   * \param interval const reference to the interval value param would be
   * incremented by.
   * \return 0 is success
             1-3 invalid for this method (used by set function)
             4 is if max is less than min
             5 if interval is -ve
             6 if interval is greater than max - min
             -1 exception thrown by code below
   */
  int set_properties(const T &min, const T &max, const T &interval) {
    int ret = 0;
    try {
      if (max < min)
        return RET_PARAM_MAX_LESS_THAN_MIN;
      if (interval < 0)
        return RET_NEGATIVE_PARAM_INTERVAL;
      if (interval > (max - min))
        return RET_PARAM_GREATER_THAN_MAX_MIN_DIFF;

      min_ = min;
      max_ = max;
      interval_ = interval;
      invalid_properties_ = false;
      PI$ ct_blu("name") pequal ParamCommon<T>::name_ pcommas ct_vblu(min_)
          pcommas ct_vblu(max_) pcommas ct_vblu(interval_) pendl;
      // make sure var is a "legal" value based on min max and interval.
      T val;
      if (ParamCommon<T>::not_arr_) {
        val = std::min(max_, ParamCommon<T>::var_[0]);
        val = std::max(min_, val);
        PI$ ct_blu("name") pequal ParamCommon<T>::name_ pcommas ct_vblu(val)
            pendl;
        ret = set(val);
      } else {
        for (int i = 0; i < ParamCommon<T>::sz_; ++i) {
          val = std::min(max_, ParamCommon<T>::var_[i]);
          val = std::max(min_, val);
          PI$ ct_blu("name") pequal ParamCommon<T>::name_ pcommas ct_vblu(i)
              pcommas ct_vblu(val) pendl;
          if ((ret = set(val))) {
            PE$ "Error settting param at index = " << ct_red(i) pendl;
            break;
          }
        }
      }
      return RET_SUCCESS_INT;
    } catch (std::exception e) {
      PE$ ct_vpur(e.what()) << pendo;
      return RET_STD_EXCEPTION;
    }
    return ret;
  }

  //! Getter for min value.
  /*!
   * Get minimum legal value for parameter.
   * \return minimum value of parameter
   * \throws in parameter is in invalid state.
   */
  T get_min() {
    THROW_IF_INVALID;
    return min_;
  }
  //! Getter for max value.
  /*!
   * Get maximim legal value for parameter.
   * \return maximim value of parameter
   * \throws in parameter is in invalid state.
   */
  T get_max() {
    THROW_IF_INVALID;
    return max_;
  }
  //! Getter for interval value.
  /*!
   * Get interval between legal values for parameters.
   * \return interval value of parameter
   * \throws in parameter is in invalid state.
   */
  T get_interval() {
    THROW_IF_INVALID;
    return interval_;
  }

private:
  /* data */
  T min_;      //!< To hold min variable value.
  T max_;      //!< To hold max variable value.
  T interval_; //!< To hold variable increment amount.
  bool invalid_properties_;

  /* functions */
  //! Assign default value for properties.
  /*!
   * Calculate default value for min, max, interval if set is called without
   * setting properties.
   */
  void guess_properties(const T &val) {
    max_ = static_cast<T>(2) * val;

    if (utils_boostd::is_signed<T>::value)
      min_ = static_cast<T>(-2) * val;
    else
      min_ = static_cast<T>(0);

    if (std::is_same<T, bool>::value)
      interval_ = static_cast<T>(0); // false
    // for floating point values if max value is greater than 1, 0.1 is
    // interval, o.w. thenth of max.
    else if (std::is_floating_point<T>::value)
      interval_ = std::pow((T)10, -1 * std::numeric_limits<T>::digits10);
    else // it's integral
      interval_ = static_cast<T>(1);

    invalid_properties_ = false;
  }

  //! Calculate closes legal value for parameter.
  /*!
   * Given min and interval, a legal value will satisfy the formula
   * \code
   * lv = min + Integer*interval.
   * \endcode
   * If value doesn't satisfy the criteria, it will assign closest legal value
   * in that case.
   * \return 0 success
             2 changed the value
   */
  int get_closest_lv(const T &val, T *lv) { // lv is legal value
    const int sn = 112;
    int ret = 0;
    // nothing bigger than long double
    long double ans = (val - min_) / static_cast<long double>(interval_);
    long double ians = std::trunc(ans);
    SP$(sn) ct_vpur(ans) pcommas ct_vpur(ians) pendl;
    if (ians != ans) {
      T multiple = static_cast<T>(std::nearbyint(ans));
      *lv = min_ + multiple * interval_;
      SP$(sn) ct_vpur(multiple) pcommas ct_vpur(min_) pcommas ct_vpur(interval_)
          pendl;
      ret = RET_CLOSEST_LEGAL_PARAM;
    } else
      *lv = val; // which is already done.
    SP$(sn) ct_vpur(*lv) pendl;
    return ret;
  }
};

//#define ENUM_CODE "(PARAM_ENUM)"
#define COND_ENUM(typ) (PENS::is_enum<typ>::value)
//! Store value and properties of params of Enum type.
/*!
 * Class for Params Enum type, this type is specially designed to deal with
 * initialization from string. So, an enum value Red could be initialized using
 * "Red". This loading Enum params from cfg file. Normal enum would be
 * dealt with general type.
 * e.g. param_enums::Color.
 * This types have orders but it's more about certain valid values, there would
 * be min and max but might not have periodicity, and it's an effor to find min
 * and max, which generally is not cared for.
 */
template <class T>
class Param<T, ENBL_IF(COND_ENUM(T), T)> : public ParamCommon<T> {
public:
  //! Constructor for general type param.
  /*!
   * One and only constructor which sets the value. No default constructor
   * available.
   */
  Param(const std::string &name, const T *val, int sz)
      : ParamCommon<T>(name, val, true, sz, true) {
    PI$ ct_vcyn(name) pcommas ct_vcyn(ParamT::has_ostream_op) pendl;
  }
  // to make ParamCommon abstract and Param not abstract.
  void pure_virtualizer() {}

  //! serialize the param value.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * OUTPUT FORMAT: [type,val](<all the enum values, comma seperated>)
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param os out stream where the serialization should be directed to
   * cout/cerr for printing and stringstream for file writing.
   */
  void to_stream(ParamT *p, std::ostream &os) {
    Param<T, T> *param_tt = static_cast<Param<T, T> *>(p);
    os << kParamDelim[0] << utils::demangle_type(typeid(T).name())
       << kParamSeperator;
    if (ParamCommon<T>::not_arr_)
      os << param_tt->get_val();
    else {
      os << kParamArrayDelim[0];
      for (int i = 0; i < ParamCommon<T>::sz_; i++) {
        os << param_tt->get_val(i);
        if (i != (ParamCommon<T>::sz_ - 1))
          os << kParamSeperator;
      }
      os << kParamArrayDelim[1];
    }

    os << kParamDelim[1];
    // write all values in enum.
    os << kParamEnumDelim[0];
    T::all_values_stream(os, kParamSeperator);
    os << kParamEnumDelim[1];
  }

  //! serialize the param value.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param str out string where the serialization should be directed to.
   */
  void get_string(ParamT *p, std::string *str) {
    Param<T, T> *ptr = static_cast<Param<T, T> *>(p);
    std::stringstream ss;
    ss << ptr->get_val();
    *str = ss.str();
  }
  void get_string_full(ParamT *p, std::string *str) {
    std::stringstream ss;
    to_stream(p, ss);
    *str = ss.str();
  }

private:
  /* data */
};

#define COND_GENERAL(typ) (!COND_ARITH(typ) && !COND_ENUM(typ))
//! Store value and properties of params of general type.
/*!
 * Class defined for parameters of general type, i.e. general type for
 * e.g. std::string, MyClass
 * This types usually don't have orders, and hence won't have a min value and a
 * max value. So, unlike it's counterpart this just stores the value.
 */
template <class T> class Param<T, void> : public ParamCommon<T> {
public:
  //! Constructor for general type param.
  /*!
   * One and only constructor which sets the value. No default constructor
   * available.
   */
  Param(const std::string &name, const T *val, int sz)
      : ParamCommon<T>(name, val, checker::has_ostream<T>::value, sz, true) {}
  // to make ParamCommon abstract and Param not abstract.
  void pure_virtualizer() {}

  //! serialize the param value.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * OUTPUT FORMAT: [type,val]
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param os out stream where the serialization should be directed to
   * cout/cerr for printing and stringstream for file writing.
   */
  void to_stream(ParamT *p, std::ostream &os) { to_stream_impl(p, os); }

  //! serialize the param value.
  /*!
   * Get parameter information in a string, which for this case is just name,
   * type, and value. There are two implementations of this function, one for
   * the type which has ostream operator<< defined, one which doesn't.
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param str out string where the serialization should be directed to.
   */
  void get_string(ParamT *p, std::string *str) { get_string_impl(p, str); }

  void get_string_full(ParamT *p, std::string *str) {
    std::stringstream ss;
    to_stream(p, ss);
    *str = ss.str();
  }

private:
  /* data */

  /* functions */
  //! serialize the param value.
  /*!
   * Serialize the param to enable printing on screen and writing to file. \n
   * OUTPUT FORMAT: [type,val]
   * \param p pointer to parameter parent, which is converted to the specific
   * type so that it's members can be accessed.
   * \param os out stream where the serialization should be directed to
   * cout/cerr for printing and stringstream for file writing.
   */
  template <typename U = T>
  auto to_stream_impl(ParamT *p, std::ostream &os)
      -> typename std::enable_if<checker::has_ostream<U>::value &&
                                 utils_boostd::is_same<T, U>::value>::type {
    os << kParamDelim[0] << utils::demangle_type(typeid(T).name())
       << kParamSeperator;
    if (ParamCommon<T>::not_arr_)
      static_cast<Param<T, void> *>(p)->get_val();
    else {
      os << kParamArrayDelim[0];
      for (int i = 0; i < ParamCommon<T>::sz_; i++) {
        static_cast<Param<T, void> *>(p)->get_val(i);
        if (i != (ParamCommon<T>::sz_ - 1))
          os << kParamSeperator;
      }
      os << kParamArrayDelim[1];
    }
    os << kParamDelim[1];
  }

  template <typename U = T>
  auto to_stream_impl(ParamT *p, std::ostream &os)
      -> typename std::enable_if<!checker::has_ostream<U>::value &&
                                 utils_boostd::is_same<T, U>::value>::type {
    // static_assert(checker::has_ostream<T>::value, "ostream is not there");
    (void)p;
    (void)os; // no unsused variable warning.
    std::string msg = ": ostream operator not defined for type " +
                      utils::demangle_type(typeid(T).name());
    PW$ ct_pur(msg) pendl;
    // throw PARAM_EXCEPT_MSG(ParamCommon<T>::name_, msg);
  }

  //! Implementation of get_string for types having ostream operator<<
  template <typename U = T>
  auto get_string_impl(ParamT *p, std::string *str)
      -> typename std::enable_if<checker::has_ostream<U>::value &&
                                 utils_boostd::is_same<T, U>::value>::type {
    Param<T, void> *ptr = static_cast<Param<T, void> *>(p);
    std::stringstream ss;
    ss << ptr->get_val();
    *str = ss.str();
  }
  //! Implementation of get_string for types not having ostream operator<<
  template <typename U = T>
  auto get_string_impl(ParamT *p, std::string *str)
      -> typename std::enable_if<!checker::has_ostream<U>::value &&
                                 utils_boostd::is_same<T, U>::value>::type {
    (void)p;
    (void)str; // no unsused variable warning.
    std::string msg = ": istream operator not defined for type " +
                      utils::demangle_type(typeid(T).name());
    PW$ ct_cyn(msg) pendl;
  }
};

//! Parameter creation return value standardization.
#define PARAM_CREATED 1000

//! START HERE: Gateway to use params.
/*!
 * Class defined using singleton paradigm. Hence, can't explicitly create object
 * of it. Params::instance().<member_fn>(...) is the way one can use the class
 * as, but it's not that user friendly. Hence, a macros "params_utils" is
 * defined which expands to Params::instance, making the most common usage as
 * \code{.cpp} params_utils.<member_fn>(....)\endcode
 * e.g.:
 * \code{.cpp}
 * params_utils.set("param1", 10.5f);
 * params_type::Float = params_utils.get<float>("param1");
 * \endcode
 */
class Params {
public:
  //! Instantiate the class object.
  /*!
   * If an instance doesn't exist it creates it and returns it, o.w. just
   * returns the already existing object. Hence, in the whole code only one
   * object(singleton) can exist.
   * NOTE: Use params_utils instead of writing Params::instance().
   * \return reference to an object of Params.
   */
  static Params &instance() {
    static Params p;
    p.dummy();
    return p;
  }

#ifdef BUILT_WITH_ROS
  //! Start ros services to list and set params.
  /*!
   * Singleton interface to ParamService defined in params_ros.h, this method is
   * included only if utils is being compiled in ros framework(i.e. catkin).
   * \return true if services were started successfully, false o.w.
   */
  bool start_ros_service();
#endif

  //! Dump the paramater name, value, [and properties] to file.
  /*!
   * Dump all the parameters that have been set in the program to file, their
   * name, current value, [current min val, current max val, and current
   * interval]
   * valued is written to file. One line will contain info of one param, and
   * vice versa. For format of output see to_stream function in Param<T> class.
   *
   * NOTE: As of now it can only dump value if operator<< is defined with
   * ostream or it's derived classes. As of now it should give some kind of
   * error a param in list doesn't have operator<< defined.
   */
  void dump(const std::string &file_name);
  //! Load the paramater name, value, [and properties] from file.
  /*!
   * Load all the parameters that are written in a file, their
   * name, value, [min val, max val, and interval] are loaded to a program.
   * One line should contain info of one param, and * vice versa.
   * For format of output see to_stream function in Param<T> class.
   *
   * NOTE: As of now it can only load value of following type:
   * \nint\n short\n long\n long long\n unsigned\n unsigned short\n
   * unsigned long\n unsigned long long\n char\n signed char\n unsigned char\n
   * float\n double\n long double\n std::string\n bool\n Param Enums\n
   * Extending this functionality as of now is quite difficult, once a good
   * solution is figured out it will be extended.
   */
  int load(const std::string &file_name);

  //! List names of all the params.
  /*!
   * Useful if names of all existing params are needed.
   * \param delim Delimiter to seperate param values.
   * \return A "\n" sepearted string of all the names.
   */
  std::string list_names(const std::string &delim);

  //! List complete info of the params.
  /*!
   * Useful if info of all existing params are needed. Param info would contain
   * following formats for various catergories:
   * 1. General :- name: [type,val]
   * 2. Arithmetic :- name: [type,val,min,max,interval]
   * 3. Enum :- name: [type,val](<All Enum values in csv>)
   * 4. Array Arithmetic :- name: [type,{val1, val2, val3},min,max,interval]
   * 5. Array Enum :- name: [type,{val1, val2, val3}](<All Enum values in csv>)
   * 6. Array General :- name: [type,{val1, val2, val3}]
   * \param delim Delimiter to seperate param values.
   * \return String with each param info seperated by "\n".
   */
  std::string list_full(const std::string &delim);

  //! Provides complete info of a parameter.
  /*!
   * Useful if info of an existing params is needed.
   * Format for various catergories:
   * 1. General :- name: [type,val]
   * 2. Arithmetic :- name: [type,val,min,max,interval]
   * 3. Enum :- name: [type,val](<All Enum values in csv>)
   * 4. Array Arithmetic :- name: [type,{val1, val2, val3},min,max,interval]
   * 5. Array Enum :- name: [type,{val1, val2, val3}](<All Enum values in csv>)
   * 6. Array General :- name: [type,{val1, val2, val3}]
   * \param name Name of the param for which info is being enquired.
   * \return String with param's info in one of the above mentioned formats.
   */
  std::string param_info_full(const std::string &name);

  //! Set param when the value is available as std::string.
  /*!
   * Often the value of the param would be available as an string, for e.g. when
   * reading from a file with flexible format. This method will try to convert
   * the value based on type, can easily fail as well, so try it out first then
   * rely if it's useful in that particular case or not.
   * \param var_name Name of the parameter to set.
   * \param type Type of the parameter. e.g. int, float, double, char...
   * \param value Value in the string form.
   *              normal: val
   *              arith: val,min,max
   *              array: {v1, v2, v3}, [min,max,interval]
   * \return 0 success
   *         1 error while setting parameters
   *         2 error while setting properties
   */
  int set_param_from_str(const std::string &var_name, const std::string &type,
                         const std::string &value);

  //! Get reference to the param value.
  /*!
   * This allows us to read parameter value in an efficient way, no need to
   * check if the value has changed at every iteration, and no need to update
   * upon a change in parameter, a change in value will be reflected
   * automatically. But this won't be thread safe.
   * NOTE: A const reference is returned, so values can't be set directly. set
   * function would be needed to call.
   * \code{.cpp} params_type::Int p1 = params_utils.get<int>("param1");\endcode
   * \param var_name Name of the parameter to get value of.
   * \return const reference to the parameter value.
   * \throws ParamExcept When parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T> const T &get(const std::string &var_name) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_casted_ptr<T>(var_name)->get();
  }
  template <typename T> const T &get(const std::string &var_name, int idx) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_casted_ptr<T>(var_name)->get(idx);
  }

  //! Link parameter to already existing variable.
  /*!
   * This function links the parameter to an existing variable, so if something
   * changes value of parameter it also changes the value of the variable being
   * used. This is useful when the "parameter" is defined by third party which
   * the algorithm reads at every iteration, which will lead to updating
   * variable at every iteration, which is not optimized. This way we update the
   * variable as a callback, so it's value changes when paramter value is
   * changed.
   * NOTE:
   *  1. Variable should NOT be const. Can't check for that so it's upto you.
   *  2. If you change variable value, parameter value doesn't change.
   *  3. If you have changed value of variable and parameter value is changed,
   *     variable value would be overwritten.
   * \code{.cpp}
   * SomeClass sc;
   * //neither sc should be const nor mem_var
   * params_utils.link("param1", &sc.mem_var);
   * \endcode
   * \param var_name Name of the parameter to get value of.
   * \param var Pointer to the existing variable to be linked with parameter.
   * \return const reference to the parameter value.
   * \throws ParamExcept When parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T> int link(const std::string &var_name, T *var) {
    FIND_IF_NOT_THROW(T, var_name);
    if (var == NULL) {
      PE$ "Trying to link null to param " << ct_vdylw(var_name) << pendo;
      return RET_LINKING_PARAM_TO_NULL;
    }
    get_casted_ptr<T>(var_name)->link(var);
    return RET_SUCCESS_INT;
  }

  //! Get param value.
  /*!
   * This allows us to capture parameter value. This will require user to check
   * and update values at their own accord. This can be thread safe, but isn't
   * yet.
   * \code{.cpp}
   * int p1 = params_utils.get_val<int>("param1", -1);
   * int p2 = params_utils.get_val("param1", -1);
   * \endcode
   * \param var_name Name of the parameter to get value of.
   * \return current value of the parameter.
   */
  template <typename T>
  const T get_val(const std::string &var_name, T def_val) {
    if (FIND_IF_NOT_NOTHROW(T, var_name))
      return def_val;

    return get_casted_ptr<T>(var_name)->get_val();
  }
  template <typename T>
  const T get_val(const std::string &var_name, int idx, T def_val) {
    if (FIND_IF_NOT_NOTHROW(T, var_name))
      return def_val;

    return get_casted_ptr<T>(var_name)->get_val(idx);
  }

  //! Get param value, throw in case of param not find.
  /*!
   * This allows us to capture parameter value. This will require user to check
   * and update values at their own accord.
   * \code{.cpp} int p1 = params_utils.get_val_throw<int>("param1");\endcode
   * \param var_name Name of the parameter to get value of.
   * \return current value of the parameter.
   */
  template <typename T, typename U = typename utils_boostd::conditional<
                            COND_GENERAL(T), void, T>::type>
  const T get_val_throw(const std::string &var_name, int idx = 0) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_val_common<T, U>(var_name, idx);
  }

  //! Get param value as a string.
  /*!
   * This allows us to capture parameter value as a string. Since, we are
   * getting value as string we might want to get the type as well for further
   * processing.
   * \code{.cpp}
   * std::string str;
   * params_utils.get_string("param1", &str);
   * \endcode
   * \param var_name Name of the parameter to get value of.
   * \return out string where the serialization should be directed to.
   */
  std::string get_string(const std::string &var_name);

  //! Get if Param value has changed.
  /*!
   * It returns if the value of param has changed since last has_changed or
   * get_val function call(we can't say if variable has been captured by get).
   * NOTE: Other get functions like get_string, list_full etc. internally call
   * get_val, hence will cause has_changed to return false.
   * \code{.cpp}
   * bool param_changed = params_utils.has_changed("param1");
   * \endcode
   * \param var_name Name of the parameter to get value of.
   * \return (bool) true if value has not been read after set, false ow.
   */
  bool has_changed(const std::string &var_name);

  //! Set param value and properties(min, max, interval) for arithmetic type.
  /*!
   * Setter for arithmetic parameter and it's properties.
   * Properties should be set via this
   * single interface, individual setters are not provided. A single interface
   * allows checking of validity or relationships between properties, such as
   * min < max, val < max, val > min etc...
   * If parameter doesn't exist it will create it and add it to the list.
   * NOTE: This interface is only available for arithmetic type
   * (http://en.cppreference.com/w/cpp/types/is_arithmetic),
   * e.g.:
   * \code{.cpp}
   * params_utils.set("param", 15, 0, 100, 1);
   * params_utils.set("param", "10", "0", "100", "1");
   * \endcode
   * should give an compilation error if is used for non-arithmetic type.
   * \param var_name Name of the variable to be set.
   * \param val const reference to the value param should be set to.
   * \param min const reference to the min value param can be set to.
   * \param max const reference to the max value param can be set to.
   * \param interval const reference to the interval value param would be
   * incremented by.
   * \return (int) 0-8 see set_generic function
                   100-104 subtract 100 from return value and see
                           Param<T>::set_properties.
   */
  template <typename T>
  ARITHMETIC(int)
      set(const std::string &var_name, T val, T min, T max, T interval) {
    if (param_map.find(var_name) != param_map.end()) {
      PW$ "This interface is only to create parameter. Doing nothing and "
          "exiting." pendl;
      return RET_SET_PARAM_NOT_FOUND;
    }
    int ret = set_generic<T, T>(var_name, val, 0);
    cPI$(ret == PARAM_CREATED) ct_vcyn(var_name) << " created" pendl;
    if (ret == 0 || ret == PARAM_CREATED) {
      ret =
          get_casted_ptr<T>(var_name)->set_properties(min, max, interval) + 100;
    }
    // if 0-8 or whatever was returned, that whatever will be returned, but if
    // 1000(PARAM_CREATED) was returned then 0(success) will be returned.
    return (ret % PARAM_CREATED);
  }

  //! Set param value and properties(min, max, interval) for arithmetic type.
  /*!
   * Setter for arithmetic parameter and it's properties.
   * Properties should be set via this
   * single interface, individual setters are not provided. A single interface
   * allows checking of validity or relationships between properties, such as
   * min < max, val < max, val > min etc...
   * If parameter doesn't exist it will create it and add it to the list.
   * NOTE: This interface is only available for arithmetic type
   * (http://en.cppreference.com/w/cpp/types/is_arithmetic),
   * e.g.:
   * \code{.cpp}
   * params_utils.set("param", 15, 0, 100, 1);
   * params_utils.set("param", "10", "0", "100", "1");
   * \endcode
   * should give an compilation error if is used for non-arithmetic type.
   * \param var_name Name of the variable to be set.
   * \param val const reference to the value param should be set to.
   * \param min const reference to the min value param can be set to.
   * \param max const reference to the max value param can be set to.
   * \param interval const reference to the interval value param would be
   * incremented by.
   * \return (int) 0-8 see set_generic function
                   100-104 subtract 100 from return value and see
                           Param<T>::set_properties.
   */
  template <typename T>
  ARITHMETIC(int)
      set_properties(const std::string &var_name, T min, T max, T interval) {
    return get_casted_ptr<T>(var_name)->set_properties(min, max, interval);
  }

  //! Set param value for artihmetic type.
  /*!
   * Setter for parameter value. This validates the value input by
   * comparing it with min and max, hence providing a safe gateway to changing
   * param values. If a aritmetic parameter is being set for the first time then
   * it will naively set max = 2*val, min = -2*val, interval to 0(integral) or
   * 0.1(floating point)
   * NOTE: This interface is for non-bool types, for bool look at the other
   * overloaded function.
   * e.g.:
   * \code{.cpp}
   * params_utils.set("param", 15);
   * params_utils.set("param", "10");
   * params_utils.set("param", "this would be const char*");
   * params_utils.set("param", std::string("this is string."));
   * \endcode
   * NOTE: While setting an enum parameter, first create it's object and then
   * call set with object as second argument. As shown in below example.
   * \code{.cpp}
   * DEFINE_PARAMS_ENUM(SampleEnum, s1, s2);
   * int main(){
   *   params_enum::SampleEnum s1 = params_enum::SampleEnum::s1;
   *   params_enum::SampleEnum s2 = params_enum::SampleEnum::from_string("s2");
   *   params_utils.set("param_s1", s1);
   *   params_utils.set("param_s2", s2);
   * }
   * \endcode
   * \param var_name Name of the variable to be set.
   * \param val const reference to the value param should be set to.
   * \return (int) 0-8 see set_generic function
                   PARAM_CREATED(1000) on creation of parameter.
   */
  template <typename T>
  typename utils_boostd::enable_if<!std::is_same<T, bool>::value, int>::type
  set(const std::string &var_name, T val, int idx = 0) {
    int ret = set_generic<T>(var_name, val, idx);
    cPI$(ret == PARAM_CREATED) ct_vcyn(var_name) << " created" pendl;
    // if 0-8 or whatever was returned, that whatever will be returned, but if
    // 1000(PARAM_CREATED) was returned then 0(success) will be returned.
    return (ret % PARAM_CREATED);
  }
  template <typename T>
  typename utils_boostd::enable_if<!std::is_same<T, bool>::value, int>::type
  set_array(const std::string &var_name, const T *val, int sz) {
    int ret = set_array_generic<T>(var_name, val, sz);
    cPI$(ret == PARAM_CREATED) ct_vcyn(var_name) << " created" pendl;
    // if 0-8 or whatever was returned, that whatever will be returned, but if
    // 1000(PARAM_CREATED) was returned then 0(success) will be returned.
    return (ret % PARAM_CREATED);
  }
  template <typename T>
  typename utils_boostd::enable_if<!std::is_same<T, bool>::value, int>::type
  set_array(const std::string &var_name, std::initializer_list<T> val) {
    set_array(var_name, val.begin(), (int)val.size());
  }

  //! Set param value for bool type.
  /*!
   * Setter for parameter value of type bool. Third parameter is special, and
   * only useful if params are represented in gui. It will specify if the bool
   * var should be a button or a checkbox.
   * e.g.:
   * \code{.cpp}
   * params_utils.set("param", true);//by default button.
   * params_utils.set("param", false, true); //checkbox(true) not ticked(false).
   * params_utils.set("param", true, true); //checkbox(true) ticked(true).
   * params_utils.set("param", true, false); //button(false)
   * params_utils.set_array("param_arr", true); //Array created
   * params_utils.set("param_arr", false, 3); //change value of 3rd index
   * \endcode
   * \param var_name Name of the variable to be set.
   * \param val const reference to the value param should be set to.
   * \param cb_btn This bool should be represented as checkbox or button, true
   * is chechbx and false is button.
   * \return (int) true on success and false ow.
   */
  template <typename T>
  typename utils_boostd::enable_if<std::is_same<T, bool>::value, int>::type
  set(const std::string &var_name, T val, bool cb_btn) {
    if (param_map.find(var_name) != param_map.end()) {
      PW$ "This interface is only to create parameter. Doing nothing and "
          "exiting." pendl;
      return RET_SET_PARAM_NOT_FOUND;
    }
    int ret = set_generic<T>(var_name, val, 0);
    PI$ ct_vcyn(var_name) << " created" pendl;
    ret = get_casted_ptr<T>(var_name)->set_properties(false, true, cb_btn);
    return ret;
  }
  template <typename T>
  typename utils_boostd::enable_if<std::is_same<T, bool>::value, int>::type
  set(const std::string &var_name, T val, int idx = 0) {
    int ret = set_generic<T>(var_name, val, idx);
    if (ret == PARAM_CREATED) { // New parameter was created, set
      PI$ ct_vcyn(var_name) << " created" pendl;
      ret = get_casted_ptr<T>(var_name)->set_properties(false, true, false);
    }
    return ret;
  }
  template <typename T>
  typename utils_boostd::enable_if<std::is_same<T, bool>::value, int>::type
  set_array(const std::string &var_name, const T *val, int sz,
            T cb_btn = false) {
    int ret = set_array_generic<T>(var_name, val, sz);
    if (ret == PARAM_CREATED) { // New parameter was created, set
      PI$ ct_vcyn(var_name) << " created" pendl;
      ret = get_casted_ptr<T>(var_name)->set_properties(false, true, cb_btn);
    }
    return ret;
  }
  template <typename T>
  typename utils_boostd::enable_if<std::is_same<T, bool>::value, int>::type
  set_array(const std::string &var_name, std::initializer_list<T> val) {
    set_array(var_name, val.begin(), (int)val.size());
  }

  //! Set callback function for parameter.
  /*!
   * Sets the function pointer for callback to a user defined function. Function
   * should have a signature of void <fun_name>(const T& new_val). These
   * functions can be used for further event triggering.
   * Further event triggering could cause circular chain reaction, so beware.
   * e.g.:
   * \code{.cpp}
   * void btn_press(const bool& new) {
   *   //TODO: action to be take on button press, write to file toggle variable
   *   value, display message etc...
   * }
   *
   * int main() {
   *   params_utils.set("Btn", true, false);//true is val, false makes it a btn.
   *   params_utils.set_functor("Btn", &btn_press);
   *   pramas_utils.set("Btn", true); //will trigger the functor.
   * }
   *
   * \endcode
   * \param var_name Name of parameter we want to set validator for.
   * \param functor function pointer to functions of type void(T)
   * \return (int) 0-1 values returned by Param<T>::set_functor
                   -1 exception thrown by below code.
   * \throws ParamExcept Parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T>
  int set_functor(const std::string &var_name, const ParamFunctor<T> &functor) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      ret = get_casted_ptr<T>(var_name)->set_functor(functor);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  //! Set callback function for parameter.
  /*!
   * Sets the function pointer for callback to a user defined function. Function
   * should have a signature of bool <fun_name>(const T& new_val). These
   * functions could be used to for validation and further event triggering.
   * Further event triggering could cause circular chain reaction, so beware.
   * NOTE:
   * Value of parameter will be updated only if this callback return true, on
   * false set function will return false as well.
   * e.g.:
   * \code{.cpp}
   * bool validate_thershold_as_prime(const int& new) {
   *   //
   *   // body to evaluate new as prime or not.
   *   //
   *   if(<num is prime>)
   *     return true;
   *   else
   *     return false;
   * }
   *
   * int main() {
   *   params_utils.set("prime_num", 3);
   *   params_utils.set_validator("prime_num", &validate_threshold_as_prime);
   *   pramas_utils.set("prime_num", 4); //will return false, param_num = 3
   *   pramas_utils.set("prime_num", 7); //will return true, param_num = 7
   * }
   *
   * \endcode
   * \param var_name Name of parameter we want to set validator for.
   * \param functor function pointer to functions of type bool(T), which should
   * return true for valid value and false for invalid value.
   * \return (int) 0-1 values returned by Param<T>::set
                   -1 exception thrown by below code.
   * \throws ParamExcept Parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T>
  int set_validator(const std::string &var_name,
                    const ParamValidatorFunctor<T> &functor) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      ret = get_casted_ptr<T>(var_name)->set_validator(functor);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  //! Set setter function for parameter.
  /*!
   * Sets the function pointer for parameter to setter interface provided by the
   * class.
   * Function could have any return type, just the it should have one input
   * argument of the same type parameter is. In case of any other signature use
   * of std::bind should be of use. Where the parameter arg number uses
   * std::placeholders::_1, rest all have fixed value.
   * NOTE:
   * Each call to the function will add the setter to the list, so multiple
   * setters can be added but each addition might make call to "set()" will
   * become slower.
   * e.g.:
   * \code{.cpp}
   * #include <iostream>
   * #include <functional>
   *
   * int main() {
   *   PPAP ppap(...);
   *   params_utils.set("ppap", 1.414f);//param of type float
   *   ParamSetter<void, float> f = std::bind(&PPAP::setval, &ppap,
   *     std::placeholders::_1);
   *   params_utils.add_setter("ppap", f);
   *   pramas_utils.set("ppap", 3.14f);
   *   cout << ppap.getval() << endl;//should print 3.14
   * }
   *
   * \endcode
   * \param var_name param name to add the setter to.
   * \param functor std::function with ret as return type and T as the lone
   * argument
   * \return (int) 0-1 return values from Param<T>::add_setter
   *               2 caught unhandled exception from below.
   * \throws ParamExcept Parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T, typename r>
  int add_setter(const std::string &var_name,
                 const ParamSetter<r, T> &functor) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      ret = get_casted_ptr<T>(var_name)->add_setter(functor);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  //! Set setter function for parameter.
  /*!
   * Sets the function pointer for parameter to setter interface provided by the
   * class.
   * Function could have any return type, just the it should have one input
   * argument of the same type parameter is. In case of any other signature use
   * of std::bind should be of use. Where the parameter arg number uses
   * std::placeholders::_1, rest all have fixed value.
   * NOTE:
   * Each call to the function will add the setter to the list, so multiple
   * setters can be added but each addition might make call to "set()" will
   * become slower.
   * e.g.:
   * \code{.cpp}
   * #include <iostream>
   * #include <functional>
   *
   * int main() {
   *   PPAP ppap(...);
   *   params_utils.set("ppap", 1.414f);//param of type float
   *   params_utils.add_setter("ppap", &PPAP::setval, &ppap);
   *   pramas_utils.set("ppap", 3.14f);
   *   cout << ppap.getval() << endl;//should print 3.14
   * }
   *
   * \endcode
   * \param var_name param name to add the setter to.
   * \param functor function pointer to setter for calss clss, with return type
   * r and input type (T || T& || const T&)
   * \param obj object for which setter will be called.
   * \return (int) 0-1 return values from Param<T>::add_setter
   *               2 caught unhandled exception from below.
   * \throws ParamExcept Parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T, typename r, class clss>
  int add_setter(const std::string &var_name, r (clss::*functor)(T),
                 clss *obj) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      if (obj == NULL) {
        PE$ "Object provided is NULL" pendl;
        return RET_OBJECT_PROVIDED_NULL;
      }
      ParamSetter<r, T> func = std::bind(functor, obj, std::placeholders::_1);
      ret = get_casted_ptr<T>(var_name)->add_setter(func);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  template <typename T, typename r, class clss>
  int add_setter(const std::string &var_name, r (clss::*functor)(T &),
                 clss *obj) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      if (obj == NULL) {
        PE$ "Object provided is NULL" pendl;
        return RET_OBJECT_PROVIDED_NULL;
      }
      ParamSetter<r, T> func = std::bind(functor, obj, std::placeholders::_1);
      ret = get_casted_ptr<T>(var_name)->add_setter(func);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  template <typename T, typename r, class clss>
  int add_setter(const std::string &var_name, r (clss::*functor)(const T &),
                 clss *obj) {
    int ret;
    FIND_IF_NOT_THROW(T, var_name);
    try {
      if (obj == NULL) {
        PE$ "Object provided is NULL" pendl;
        return RET_OBJECT_PROVIDED_NULL;
      }
      ParamSetter<r, T> func = std::bind(functor, obj, std::placeholders::_1);
      ret = get_casted_ptr<T>(var_name)->add_setter(func);
    } catch (std::exception e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  //! Getter for min value.
  /*!
   * Get the min value of the parameter of arithmetic type, will give an
   * compiler error for non-arithmetic type.
   * \param var_name Name of the parameter min value should be returned for.
   * \return default value of the type if there is some error in conversion, min
   * value o.w.
   * \throws ParamExcept When parameter doesn't exist.
   */
  template <typename T>
  // typename utils_boostd::enable_if<utils_boostd::is_arithmetic<T>::value,
  // T>::type
  ARITHMETIC(T) get_min(const std::string &var_name) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_casted_ptr<T>(var_name)->get_min();
  }

  //! Getter for max value.
  /*!
   * Get the max value of the parameter of arithmetic type, will give an
   * compiler error for non-arithmetic type.
   * \param var_name Name of the parameter max value should be returned for.
   * \return default value of the type if there is some error in conversion, max
   * value o.w.
   * \throws ParamExcept When parameter doesn't exist.
   */
  template <typename T>
  // typename utils_boostd::enable_if<utils_boostd::is_arithmetic<T>::value,
  // T>::type
  ARITHMETIC(T) get_max(const std::string &var_name) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_casted_ptr<T>(var_name)->get_max();
  }

  //! Getter for interval value.
  /*!
   * Get the interval value of the parameter of arithmetic type, will give an
   * compiler error for non-arithmetic type.
   * \param var_name Name of the parameter interval value should be returned
   * for.
   * \return default value of the type if there is some error in conversion,
   * interval value o.w.
   * \throws ParamExcept When parameter doesn't exist.
   */
  template <typename T>
  // typename utils_boostd::enable_if<utils_boostd::is_arithmetic<T>::value,
  // T>::type
  ARITHMETIC(T) get_interval(const std::string &var_name) {
    FIND_IF_NOT_THROW(T, var_name);
    return get_casted_ptr<T>(var_name)->get_interval();
  }

  //! Get type for the parameter.
  /*!
   * \param var_name Name of the parameter type should be returned for.
   * \return type name as string, empty string if parameter doesn't exist.
   */
  std::string get_type(const std::string &var_name) {
    return param_type[var_name];
  }

  //! Check the existence of the parameter.
  /*!
   * \param var_name Name of the parameter type should be returned for.
   * \return type true as bool, false if parameter doesn't exist.
   */
  //:haha: added 290916:0951
  bool has_param(const std::string &var_name) {
    if (param_map.find(var_name) == param_map.end())
      return RET_HAS_NO_PARAM;
    return true;
  }

private:
  /* data */
  Params() {}
  std::map<std::string, ParamT *> param_map;
  std::map<std::string, std::string> param_type;

  /* functions */
  void dummy(); // dummy function to force load the library.

  //! Get pointer after cast for partial specified template.
  /*!
   * Mostly for internal use.
   * This function converts ParamT* to Param<T,T>* using dynamic cast. It
   * also validates the conversion by checking if the returned pointer is null,
   * if it's null then it throws an exception.
   * \code{.cpp}
   * Param<int,int>* v = params_utils.get_casted_ptr<int>("int_param");
   * \endcode
   * \param var_name Name of the parameter to get value of.
   * \return Pointer of type Param<T,void>.
   * \throws ParamExcept Pointer to base class(ParamT*) cannot be converted to
   * Param<T>*
   */
  template <typename T, typename U = typename utils_boostd::conditional<
                            COND_GENERAL(T), void, T>::type>
  Param<T, U> *get_casted_ptr(const std::string &var_name) {
    Param<T, U> *v = dynamic_cast<Param<T, U> *>(param_map[var_name]);
    if (v == NULL) {
      throw PARAM_EXCEPT_MSG(var_name, ": dynamic_cast returned NULL");
    }
    return v;
  }

  //! Get val implementation which is common for all get_val*.
  /*!
   * Mostly for internal use.
   * NOTE: This will fail if T doesn't have a default constructor, but if we
   * don't * return anythin then it will be in unspecified state, atleast this
   * way we * know state will be valid.
   * \param var_name Name of the parameter to get value of.
   * \param idx Index in case param is an array.
   * \param def_val It's the default value, which is either supplied by user or
   * 		    it defaults to T().
   * \return Parameter value or fi param doesn't exist default value.
   */
  template <typename T, typename U = typename utils_boostd::conditional<
                            COND_GENERAL(T), void, T>::type>
  const T get_val_common(const std::string &var_name, int idx = 0,
                         T def_val = T()) {
    try {
      Param<T, U> *param = get_casted_ptr<T>(var_name);
      param->set_changed(false);
      return param->get_val();
    } catch (ParamExcept &p) {
      DP$ ct_ylw("Except: ") << ct_vpur(p.what()) pendl;
      return def_val;
    } catch (std::exception &e) {
      DP$ ct_ylw("Except: ") << ct_vpur(e.what()) pendl;
      return def_val;
    }

    return def_val;
  }

  //! Generic function for set function (public).
  /*!
   * Setter for parameter value of all three types ARITH, ENUM, GENERAL.
   * If parameter doesn't exist it will create it and add it to the list.
   * If it exists then it will check for the type of the parameter set before,
   * if the type mismatches it will return non-zero value, if the type matches
   * it calls set for Param subtype.
   * e.g.:
   * \param val const reference to the value param should be set to.
   * \return (int) 0-4 Param<T,*>::set return values.
                   5-8 invlid for this (used by set_properties function)
                   9 Parameter type and val type are not same.
                   10 caught exception thrown from Parameter operations
                   11 generic exception was caught
                   PARAM_CREAED(1000) New parameter was created.
   */
  template <typename T, typename U = typename utils_boostd::conditional<
                            COND_GENERAL(T), void, T>::type>
  int set_generic(const std::string &var_name, const T &val, int idx = 0) {
    int ret = RET_SUCCESS_INT;
    try {
      if (param_map.find(var_name) == param_map.end()) {
        create_param<T, U>(var_name, &val);
        ret = PARAM_CREATED;
      } else if (typeid(T).name() != param_type[var_name]) {
        PE$ "type asked for: "
            << ct_vdylw(utils::demangle_type(typeid(T).name()))
            << ", param type: "
            << ct_vdylw(utils::demangle_type(param_type[var_name])) << pendo;
        ret = RET_PARAM_TYPE_DONOT_MATCH;
      } else {
        Param<T, U> *p = get_casted_ptr<T>(var_name);
        ret = p->set(val, idx);
#if PUBLISH_PARAMS
#ifdef BUILT_WITH_ROS
        PI$ "advertising Val " pendl;
        std::string new_val;
        p->get_string_full(p, &new_val);
        // SP$(sn) ct_vpur(new_val) pendl;
        ParamROS::instance().advertise_value(new_val);
        PI$ "Val advertised" pendl;
#endif
#endif
      }
    } catch (ParamExcept &e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_PARAM_EXCEPTION;
    } catch (std::exception &e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    }

    return ret;
  }

  //! Generic function for set function (public).
  /*!
   * Setter for parameter value of all three types ARITH, ENUM, GENERAL.
   * If parameter doesn't exist it will create it and add it to the list.
   * If it exists then it will check for the type of the parameter set before,
   * if the type mismatches it will return non-zero value, if the type matches
   * it calls set for Param subtype.
   * e.g.:
   * \param val const reference to the value param should be set to.
   * \return (int) 0-4 Param<T,*>::set return values.
                   5-8 invlid for this (used by set_properties function)
                   9 Parameter type and val type are not same.
                   10 caught exception thrown from Parameter operations
                   11 generic exception was caught
                   PARAM_CREAED(1000) New parameter was created.
   */
  template <typename T, typename U = typename utils_boostd::conditional<
                            COND_GENERAL(T), void, T>::type>
  int set_array_generic(const std::string &var_name, const T *val, int sz = 1) {
    PRINT_FUNC_ENTER;
    int ret = RET_SUCCESS_INT;
    try {
      if (param_map.find(var_name) == param_map.end()) {
        create_param<T, U>(var_name, val, sz);
        ret = PARAM_CREATED;
      } else if (typeid(T).name() != param_type[var_name]) {
        ret = RET_PARAM_TYPE_DONOT_MATCH;
      } else {
        Param<T, U> *p = get_casted_ptr<T>(var_name);
        for (int i = 0; i < sz; ++i) {
          ret = p->set(val[i], i);
        }
#if PUBLISH_PARAMS
#ifdef BUILT_WITH_ROS
        PI$ "advertising Val " pendl;
        std::string new_val;
        p->get_string_full(p, &new_val);
        // SP$(sn) ct_vpur(new_val) pendl;
        ParamROS::instance().advertise_value(new_val);
        PI$ "Val advertised" pendl;
#endif
#endif
      }
    } catch (ParamExcept &e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_STD_EXCEPTION;
    } catch (std::exception &e) {
      PE$ ct_ylw("except: ") << e.what() << std::endl;
      return RET_PARAM_EXCEPTION;
    }

    PRINT_FUNC_EXIT;
    return ret;
  }

  //! Create Parameter.
  /*!
   * This function creates the Parameter with arithmetic type, for
   * initialization it uses default scheme values, later it's users
   * responsibility to change it values which make more sense in the scenario.
   * \param var_name Name of the parameter to get value of.
   * \param val Initial value parameter should be assigned.
   * \return void
   */
  template <typename T,
            typename U = typename utils_boostd::conditional<
                utils_boostd::is_arithmetic<T>::value, T, void>::type>
  Param<T, U> *create_param(const std::string &var_name, const T *val,
                            int sz = 1) {
    PRINT_FUNC_ENTER;
    Param<T, U> *p = new Param<T, U>(var_name, val, sz);
    if (val != NULL) {
      for (int i = 0; i < sz; ++i) {
        p->set(val[i], i);
      }
    }
    param_map[var_name] = p;
    param_type[var_name] = typeid(T).name();
    PRINT_FUNC_EXIT;
    return p;
  }

  //! Search and check the type of parameter, throw if not found or diff type.
  /*!
   * This function should be useful in multiple functions where we look if the
   * parameter has been defined and throwing exception is the only way to
   * indicate an error has occured and not kill the program, since return value
   * has a meaning. Uncaught exception will kill the program as well, but
   * hopefully user catches the exception and gracefully exits or take alternate
   * evasive actions.
   * \param var_name Name of the parameter to get value of.
   * \return void
   * \throws ParamExcept When parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T>
  void find_if_not_throw(const std::string &var_name, std::string fn, int line,
                         std::string file) {
    if (param_map.find(var_name) == param_map.end()) {
      // PE$ "While searching for " << ct_vred(var_name) pendl;
      throw PARAM_EXCEPT_MSG_LONG(var_name, "Parmater doesn't exist.", fn, line,
                                  file);
    } else if (typeid(T).name() != param_type[var_name]) {
      PE$ "type asked for: "
          << ct_vdylw(utils::demangle_type(typeid(T).name()))
          << ", param type: "
          << ct_vdylw(utils::demangle_type(param_type[var_name])) << pendo;
      throw PARAM_EXCEPT_MSG_LONG(var_name, "Parmeter types don't match.", fn,
                                  line, file);
    }
  }

  //! Search and check the type of parameter, throw if not found or diff type.
  /*!
   * This function should be useful in multiple functions where we look if the
   * parameter has been defined and throwing exception is the only way to
   * indicate an error has occured and not kill the program, since return value
   * has a meaning. Uncaught exception will kill the program as well, but
   * hopefully user catches the exception and gracefully exits or take alternate
   * evasive actions.
   * \param var_name Name of the parameter to get value of.
   * \return void
   * \throws ParamExcept When parameter doesn't exist.
   * \throws ParamExcept Parameter type and requested type don't match.
   */
  template <typename T>
  int find_if_not_nothrow(const std::string &var_name, std::string func,
                          int line, std::string file, bool verbose = true) {
    if (param_map.find(var_name) == param_map.end()) {
      cPE$(verbose) "Param " << ct_vdylw(var_name) << " not found."
                             << " called by " << ct_ylw(func)
                             << " from line number " << ct_ylw(line)
                             << " in file " << ct_ylw(file) << std::endl;
      return RET_PARAM_NOT_FOUND;
    } else if (typeid(T).name() != param_type[var_name]) {
      cPE$(verbose) "type asked for: "
          << ct_vdylw(utils::demangle_type(typeid(T).name()))
          << ", param type: "
          << ct_vdylw(utils::demangle_type(param_type[var_name])) << pendo;
      cPE$(verbose) "Parmeter types don't match." << pendo;
      return RET_PARAM_TYPE_DONOT_MATCH;
    }
    return RET_SUCCESS_INT;
  }
};
