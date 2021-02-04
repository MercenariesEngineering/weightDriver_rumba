#pragma once

#include "Rumba/Rumba.h"

class DictOrArray
{
public:
  DictOrArray(const rumba::Value& value);
  float as_float(const char* name, size_t index) const;
  Imath::V3f as_V3f(const char* name, size_t index) const;
  Imath::V3f as_V3f(const char* name, size_t index, const Imath::V3f& default_) const;
  rumba::Value read(const char* name, size_t index) const;
  int as_int(const char* name, size_t index) const;
  int as_int(const char* name, size_t index, int default_) const;
  bool as_bool(const char* name, size_t index, bool default_) const;
  Imath::M44f as_M44f(const char* name, size_t index) const;
private:

  rumba::Value _array_lookup(size_t i) const
  {
    if(i < _array.size())
      return _array[i];
    return rumba::Value::default_value;
  }

  bool _is_dict;
  rumba::Dict _dict;
  rumba::Array _array;
};

inline DictOrArray::DictOrArray(const rumba::Value& value) :
  _is_dict(value.is_instance("Dict")),
  _dict(value),
  _array(value)
{}

inline float DictOrArray::as_float(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_float(name) : _array_lookup(index).as_float();
}

inline Imath::V3f DictOrArray::as_V3f(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_V3f(name) : _array_lookup(index).as_V3f();
}

inline Imath::V3f DictOrArray::as_V3f(const char* name, size_t index, const Imath::V3f& default_) const
{
  return _is_dict ? _dict.as_V3f(name, default_) : (index < _array.size() ? _array.as_V3f(index) : default_);
}

inline rumba::Value DictOrArray::read(const char* name, size_t index) const
{
  return _is_dict ? _dict.read(name) : _array_lookup(index);
}

inline int DictOrArray::as_int(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_int(name) : _array_lookup(index).as_int();
}

inline int DictOrArray::as_int(const char* name, size_t index, int default_) const
{
  return _is_dict ? _dict.as_int(name, default_) : (index < _array.size() ? _array.as_int(index) : default_);
}

inline bool DictOrArray::as_bool(const char* name, size_t index, bool default_) const
{
  return _is_dict ? _dict.as_bool(name, default_) : (index < _array.size() ? _array.as_bool(index) : default_);
}

inline Imath::M44f DictOrArray::as_M44f(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_M44f(name) : _array_lookup(index).as_M44f();
}
