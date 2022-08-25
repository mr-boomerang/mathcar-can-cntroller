#pragma once

#ifdef BOOST_NOT_CPP11
#define ARITHMETIC(typ)                                                        \
  typename boost::enable_if_c<boost::is_arithmetic<T>::value, typ>::type
#define NOT_ARITHMETIC(typ)                                                    \
  typename boost::enable_if_c<!boost::is_arithmetic<T>::value, typ>::type
#define ENBL_IF(cond, typ) typename boost::enable_if<cond, typ>::type
namespace utils_boostd = boost;
#else
#define ARITHMETIC(typ)                                                        \
  typename std::enable_if<std::is_arithmetic<T>::value, typ>::type
#define NOT_ARITHMETIC(typ)                                                    \
  typename std::enable_if<!std::is_arithmetic<T>::value, typ>::type
#define ENBL_IF(cond, typ) typename std::enable_if<cond, typ>::type
// this is to get rid of error "expecting a namespace", this will not really
// change anything
namespace std {}
namespace utils_boostd = std;
#endif
