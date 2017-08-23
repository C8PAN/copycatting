#ifndef SIMPLE_STRING_H
#define SIMPLE_STRING_H

#include <iostream>
#include <sstream>
#include <stdio.h>

class SimpleString {
  friend bool operator==(const SimpleString &left, const SimpleString &right);

public:
  SimpleString();
  SimpleString(const char *value);
  SimpleString(const SimpleString &other);
  ~SimpleString();

  SimpleString & operator=(const SimpleString &other);
  char *AsCharString() const;
  int size() const;

private:
  char *buffer_;   
};

template<class T>
SimpleString StringFrom(const T &other) {
  std::ostringstream os;
  os << other;
  return SimpleString(os.str().c_str());
}

template<>
inline SimpleString StringFrom(const bool &value) {
  std::ostringstream os;
  os << (value ? "true" : "false");
  return SimpleString(os.str().c_str());
}



#endif
