#ifndef STL_UTILS_H_FJXPFIZ1
#define STL_UTILS_H_FJXPFIZ1

#include <iostream>
#include <ostream>

#include <array>
#include <deque>
#include <forward_list>
#include <list>
#include <map>
#include <queue>
#include <set>
#include <set>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#define STL_DEF

#define DEF_SPACE                                                              \
  std::string space =                                                          \
      (os.rdbuf() == std::cout.rdbuf() || os.rdbuf() == std::cerr.rdbuf())     \
          ? " "                                                                \
          : ""

// no namespace here, generic operators, so better to keep it in global
// namespace, so that mode_printing.h etc can access it from namespace print
// et. al.

// Serialization of stl objects.
template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::vector<type> &m) {
  if (m.empty()) {
    os << "Empty vector";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::vector<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::vector<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::list<type> &m) {
  if (m.empty()) {
    os << "Empty list";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::list<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::list<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os,
                                const std::forward_list<type> &m) {
  if (m.empty()) {
    os << "Empty forward list";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::forward_list<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::forward_list<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::stack<type> &m) {
  if (m.empty()) {
    os << "Empty stack";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::stack<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::stack<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::queue<type> &m) {
  if (m.empty()) {
    os << "Empty queue";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::queue<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::queue<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::deque<type> &m) {
  if (m.empty()) {
    os << "Empty double queue";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::deque<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::deque<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os, const std::set<type> &m) {
  if (m.empty()) {
    os << "Empty set";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::set<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::set<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename type>
inline std::ostream &operator<<(std::ostream &os,
                                const std::unordered_set<type> &m) {
  if (m.empty()) {
    os << "Empty unordered set";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::unordered_set<type>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::unordered_set<type>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename val, std::size_t sz>
inline std::ostream &operator<<(std::ostream &os,
                                const std::array<val, sz> &m) {
  if (m.empty()) {
    os << "Empty array";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::array<val, sz>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::array<val, sz>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << *i << "," << space;
  }
  os << *i;
  return os;
}

template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os, const std::map<key, val> &m) {
  if (m.empty()) {
    os << "Empty map";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::map<key, val>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::map<key, val>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << "[" << i->first << ":" << i->second << "]"
       << "," << space;
  }
  os << "[" << i->first << ":" << i->second << "]";
  return os;
}

template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os,
                                const std::multimap<key, val> &m) {
  if (m.empty()) {
    os << "Empty multimap";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::multimap<key, val>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::multimap<key, val>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << "[" << i->first << ":" << i->second << "]"
       << "," << space;
  }
  os << "[" << i->first << ":" << i->second << "]";
  return os;
}

template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os,
                                const std::unordered_map<key, val> &m) {
  if (m.empty()) {
    os << "Empty unordered map";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::unordered_map<key, val>::const_iterator penultimate = m.end();
  penultimate--;
  typename std::unordered_map<key, val>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << "[" << i->first << ":" << i->second << "]"
       << "," << space;
  }
  os << "[" << i->first << ":" << i->second << "]";
  return os;
}

template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os,
                                const std::unordered_multimap<key, val> &m) {
  if (m.empty()) {
    os << "Empty unordered multimap";
    return os;
  }
  DEF_SPACE; // this defines the variable space.
  typename std::unordered_multimap<key, val>::const_iterator penultimate =
      m.end();
  penultimate--;
  typename std::unordered_multimap<key, val>::const_iterator i = m.begin();
  for (; i != penultimate; ++i) {
    os << "[" << i->first << ":" << i->second << "]"
       << "," << space;
  }
  os << "[" << i->first << ":" << i->second << "]";
  return os;
}

#ifndef PAIR_OPERATOR_DEF
#define PAIR_OPERATOR_DEF
template <typename key, typename val>
inline std::ostream &operator<<(std::ostream &os,
                                const std::pair<key, val> &p) {
  os << "[" << p.first << ":" << p.second << "]";
  return os;
}
#endif

#endif /* end of include guard: STL_UTILS_H_FJXPFIZ1 */
