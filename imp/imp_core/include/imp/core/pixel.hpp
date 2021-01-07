#pragma once

#include <cstdint>
#include <cmath>
#include <imp/core/pixel_enums.hpp>
#include <ze/common/types.hpp>

#ifdef WITH_CUDA
#  include<cuda_runtime_api.h>
#  define CUDA_HOST __host__
#  define CUDA_DEVICE  __device__
#else
#  define CUDA_HOST
#  define CUDA_DEVICE
#endif

namespace ze {

//------------------------------------------------------------------------------
template<typename _T>
union Pixel1
{
  using T = _T;

  struct
  {
    T x;
  };
  struct
  {
    T r;
  };
  T c[1];

  CUDA_HOST CUDA_DEVICE Pixel1() : x(0) { }
  CUDA_HOST CUDA_DEVICE Pixel1(T _x) : x(_x) { }
  CUDA_HOST CUDA_DEVICE ~Pixel1() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 1;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator+=(const Pixel1<T>& rhs)
  {
    c[0] += rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel1<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel2
{
  using T = _T;

  struct
  {
    T x,y;
  };
  struct
  {
    T r,g;
  };
  T c[2];

  CUDA_HOST CUDA_DEVICE Pixel2() : x(0), y(0) { }
  CUDA_HOST CUDA_DEVICE Pixel2(T _a) : x(_a), y(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel2(T _x, T _y) : x(_x), y(_y) { }
  CUDA_HOST CUDA_DEVICE ~Pixel2() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 2;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator+=(const Pixel2<T>& rhs)
  {
    c[0] += rhs[0];
    c[1] += rhs[1];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const Pixel2<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel2<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel3
{
  using T = _T;

  struct
  {
    T x,y,z;
  };
  struct
  {
    T r,g,b;
  };
  T c[3];

  CUDA_HOST CUDA_DEVICE Pixel3() : x(0), y(0), z(0) { }
  CUDA_HOST CUDA_DEVICE Pixel3(T _a) : x(_a), y(_a), z(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel3(T _x, T _y, T _z) : x(_x), y(_y), z(_z) { }
  CUDA_HOST CUDA_DEVICE ~Pixel3() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 3;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator+=(const Pixel3<T>& rhs)
  {
    c[0] += rhs[0];
    c[1] += rhs[1];
    c[2] += rhs[2];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    c[2] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const Pixel3<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    c[2] *= rhs[2];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    c[2] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel3<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    c[2] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
template<typename _T>
union Pixel4
{
  using T = _T;

  struct
  {
    T x,y,z,w;
  };
  struct
  {
    T r,g,b,a;
  };
  T c[4];

  CUDA_HOST CUDA_DEVICE Pixel4() : x(0), y(0), z(0), w(0) { }
  CUDA_HOST CUDA_DEVICE Pixel4(T _a) : x(_a), y(_a), z(_a), w(_a) { }
  CUDA_HOST CUDA_DEVICE Pixel4(T _x, T _y, T _z, T _w) : x(_x), y(_y), z(_z), w(_w) { }
  CUDA_HOST CUDA_DEVICE ~Pixel4() = default;

  CUDA_HOST CUDA_DEVICE constexpr std::uint8_t numDims() const {return 4;}

  CUDA_HOST CUDA_DEVICE operator T() const { return c[0]; }
  CUDA_HOST CUDA_DEVICE T& operator[](size_t i) { return c[i]; }
  CUDA_HOST CUDA_DEVICE const T& operator[](size_t i) const { return c[i]; }
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator+=(const Pixel4<T>& rhs)
  {
    c[0] += rhs[0];
    c[1] += rhs[1];
    c[2] += rhs[2];
    c[3] += rhs[3];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const Pixel1<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[0];
    c[2] *= rhs[0];
    c[3] *= rhs[0];
    return *this;
  }
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const Pixel4<T>& rhs)
  {
    c[0] *= rhs[0];
    c[1] *= rhs[1];
    c[2] *= rhs[2];
    c[3] *= rhs[3];
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator*=(const TRHS& rhs)
  {
    c[0] *= rhs;
    c[1] *= rhs;
    c[2] *= rhs;
    c[3] *= rhs;
    return *this;
  }
  template<typename TRHS>
  CUDA_HOST CUDA_DEVICE Pixel4<T>& operator/(const TRHS& rhs)
  {
    c[0] /= rhs;
    c[1] /= rhs;
    c[2] /= rhs;
    c[3] /= rhs;
    return *this;
  }
};

//------------------------------------------------------------------------------
// convenience typedefs
typedef Pixel1<uint8_t> Pixel8uC1;
typedef Pixel2<uint8_t> Pixel8uC2;
typedef Pixel3<uint8_t> Pixel8uC3;
typedef Pixel4<uint8_t> Pixel8uC4;

typedef Pixel1<int16_t> Pixel16sC1;
typedef Pixel2<int16_t> Pixel16sC2;
typedef Pixel3<int16_t> Pixel16sC3;
typedef Pixel4<int16_t> Pixel16sC4;

typedef Pixel1<uint16_t> Pixel16uC1;
typedef Pixel2<uint16_t> Pixel16uC2;
typedef Pixel3<uint16_t> Pixel16uC3;
typedef Pixel4<uint16_t> Pixel16uC4;

typedef Pixel1<int32_t> Pixel32sC1;
typedef Pixel2<int32_t> Pixel32sC2;
typedef Pixel3<int32_t> Pixel32sC3;
typedef Pixel4<int32_t> Pixel32sC4;

typedef Pixel1<uint32_t> Pixel32uC1;
typedef Pixel2<uint32_t> Pixel32uC2;
typedef Pixel3<uint32_t> Pixel32uC3;
typedef Pixel4<uint32_t> Pixel32uC4;

typedef Pixel1<float> Pixel32fC1;
typedef Pixel2<float> Pixel32fC2;
typedef Pixel3<float> Pixel32fC3;
typedef Pixel4<float> Pixel32fC4;


// vector types (same as pixel)
template<typename T> using Vec1 = Pixel1<T>;
template<typename T> using Vec2 = Pixel2<T>;
template<typename T> using Vec3 = Pixel3<T>;
template<typename T> using Vec4 = Pixel4<T>;

using Vec8uC1 = Pixel8uC1;
using Vec8uC2 = Pixel8uC2;
using Vec8uC3 = Pixel8uC3;
using Vec8uC4 = Pixel8uC4;

using Vec16uC1 = Pixel16sC1;
using Vec16uC2 = Pixel16sC2;
using Vec16uC3 = Pixel16sC3;
using Vec16uC4 = Pixel16sC4;

using Vec32sC1 = Pixel32sC1;
using Vec32sC2 = Pixel32sC2;
using Vec32sC3 = Pixel32sC3;
using Vec32sC4 = Pixel32sC4;

using Vec32uC1 = Pixel32uC1;
using Vec32uC2 = Pixel32uC2;
using Vec32uC3 = Pixel32uC3;
using Vec32uC4 = Pixel32uC4;

using Vec32fC1 = Pixel32fC1;
using Vec32fC2 = Pixel32fC2;
using Vec32fC3 = Pixel32fC3;
using Vec32fC4 = Pixel32fC4;

//------------------------------------------------------------------------------
// Pixel traits.
template<typename> struct pixel_type { static constexpr PixelType type = PixelType::undefined; };
template<> struct pixel_type <Pixel8uC1> { static constexpr PixelType type = PixelType::i8uC1; };
template<> struct pixel_type <Pixel8uC2> { static constexpr PixelType type = PixelType::i8uC2; };
template<> struct pixel_type <Pixel8uC3> { static constexpr PixelType type = PixelType::i8uC3; };
template<> struct pixel_type <Pixel8uC4> { static constexpr PixelType type = PixelType::i8uC4; };

template<> struct pixel_type <Pixel16sC1> { static constexpr PixelType type = PixelType::i16sC1; };
template<> struct pixel_type <Pixel16sC2> { static constexpr PixelType type = PixelType::i16sC2; };
template<> struct pixel_type <Pixel16sC3> { static constexpr PixelType type = PixelType::i16sC3; };
template<> struct pixel_type <Pixel16sC4> { static constexpr PixelType type = PixelType::i16sC4; };

template<> struct pixel_type <Pixel16uC1> { static constexpr PixelType type = PixelType::i16uC1; };
template<> struct pixel_type <Pixel16uC2> { static constexpr PixelType type = PixelType::i16uC2; };
template<> struct pixel_type <Pixel16uC3> { static constexpr PixelType type = PixelType::i16uC3; };
template<> struct pixel_type <Pixel16uC4> { static constexpr PixelType type = PixelType::i16uC4; };

template<> struct pixel_type <Pixel32uC1> { static constexpr PixelType type = PixelType::i32uC1; };
template<> struct pixel_type <Pixel32uC2> { static constexpr PixelType type = PixelType::i32uC2; };
template<> struct pixel_type <Pixel32uC3> { static constexpr PixelType type = PixelType::i32uC3; };
template<> struct pixel_type <Pixel32uC4> { static constexpr PixelType type = PixelType::i32uC4; };

template<> struct pixel_type <Pixel32sC1> { static constexpr PixelType type = PixelType::i32sC1; };
template<> struct pixel_type <Pixel32sC2> { static constexpr PixelType type = PixelType::i32sC2; };
template<> struct pixel_type <Pixel32sC3> { static constexpr PixelType type = PixelType::i32sC3; };
template<> struct pixel_type <Pixel32sC4> { static constexpr PixelType type = PixelType::i32sC4; };

template<> struct pixel_type <Pixel32fC1> { static constexpr PixelType type = PixelType::i32fC1; };
template<> struct pixel_type <Pixel32fC2> { static constexpr PixelType type = PixelType::i32fC2; };
template<> struct pixel_type <Pixel32fC3> { static constexpr PixelType type = PixelType::i32fC3; };
template<> struct pixel_type <Pixel32fC4> { static constexpr PixelType type = PixelType::i32fC4; };

//------------------------------------------------------------------------------
// comparison operators
template<typename T>
inline bool operator==(const Pixel1<T>& lhs, const Pixel1<T>& rhs)
{
  return (lhs.x == rhs.x);
}

//------------------------------------------------------------------------------
// dot product

template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec2<T> a, Vec2<T> b)
{
  return a.x * b.x + a.y * b.y;
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec3<T> a, Vec3<T> b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE T dot(Vec4<T> a, Vec4<T> b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

//------------------------------------------------------------------------------
// length
template<typename T>
inline CUDA_HOST CUDA_DEVICE float length(T v)
{
  return std::sqrt((float)dot(v,v));
}

//------------------------------------------------------------------------------
//normalize

template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC2 normalize(Vec2<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC2(v.x*inv_len, v.y*inv_len);
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC3 normalize(Vec3<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC3(v.x*inv_len, v.y*inv_len, v.z*inv_len);
}
template<typename T>
inline CUDA_HOST CUDA_DEVICE Vec32fC4 normalize(Vec4<T> v)
{
  float inv_len = 1.0f/length(v);
  return Vec32fC4(v.x*inv_len, v.y*inv_len, v.z*inv_len, v.w*inv_len);
}


} // namespace ze

#undef CUDA_HOST
#undef CUDA_DEVICE

