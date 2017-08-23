#include "simple_string.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

SimpleString::SimpleString() : buffer_(new char[1]) {
  buffer_[0] = '\0';
}

SimpleString::SimpleString(const char *other) : 
  buffer_(new char[strlen(other)+1]) {
  strcpy(buffer_, other);
}

SimpleString::SimpleString(const SimpleString &other) {
  buffer_ = new char[other.size()+1];
  strcpy(buffer_, other.buffer_);
}

SimpleString &SimpleString::operator=(const SimpleString &other) {
  delete buffer_;
  buffer_ = new char[other.size()+1];
  strcpy(buffer_, other.buffer_);
  return *this;
}

char *SimpleString::AsCharString() const {
  return buffer_;
}

int SimpleString::size() const {
  return strlen(buffer_);
}

SimpleString::~SimpleString() {
  delete [] buffer_;
}

bool operator==(const SimpleString &left, SimpleString &right) {
  return !strcmp(left.AsCharString(), right.AsCharString());
}


