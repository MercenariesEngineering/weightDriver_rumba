#pragma once

#include "Maquina/Maquina.h"

class DictOrArray
{
public:
  DictOrArray(const maquina::Value& value);
  double as_double(const char* name, size_t index) const;
  const Imath::V3d& as_V3d(const char* name, size_t index) const;
  const Imath::V3d& as_V3d(const char* name, size_t index, const Imath::V3d& default_) const;
  maquina::Value read(const char* name, size_t index) const;
  int as_int(const char* name, size_t index) const;
  int as_int(const char* name, size_t index, int default_) const;
  bool as_bool(const char* name, size_t index, bool default_) const;
  const Imath::M44d& as_M44d(const char* name, size_t index) const;
private:

  maquina::Value _array_lookup(size_t i) const
  {
    if(i < _array.size())
      return _array[i];
    return maquina::Value::default_value;
  }

  bool _is_dict;
  maquina::Dict _dict;
  maquina::Array _array;
};

inline DictOrArray::DictOrArray(const maquina::Value& value) :
  _is_dict(value.is_instance("Dict")),
  _dict(value),
  _array(value)
{}

inline double DictOrArray::as_double(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_double(name) : _array_lookup(index).as_double();
}

inline const Imath::V3d& DictOrArray::as_V3d(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_V3d(name) : _array_lookup(index).as_V3d();
}

inline const Imath::V3d& DictOrArray::as_V3d(const char* name, size_t index, const Imath::V3d& default_) const
{
  return _is_dict ? _dict.as_V3d(name, default_) : (index < _array.size() ? _array.as_V3d(index) : default_);
}

inline maquina::Value DictOrArray::read(const char* name, size_t index) const
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

inline const Imath::M44d& DictOrArray::as_M44d(const char* name, size_t index) const
{
  return _is_dict ? _dict.as_M44d(name) : _array_lookup(index).as_M44d();
}
