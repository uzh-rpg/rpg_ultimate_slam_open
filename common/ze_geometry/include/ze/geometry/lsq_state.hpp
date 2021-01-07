// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the ETH Zurich, Wyss Zurich, Zurich Eye nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL ETH Zurich, Wyss Zurich, Zurich Eye BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Modified: Robotics and Perception Group

#pragma once

#include <tuple>
#include <type_traits>
#include <typeinfo>
#include <iostream>
#include <glog/logging.h>
#include <ze/common/types.hpp>
#include <ze/common/manifold.hpp>

namespace ze {

namespace internal {

// -----------------------------------------------------------------------------
// Check if any element in the tuple is of dynamic size
template<typename T> struct TupleIsFixedSize;

template<typename Element>
struct TupleIsFixedSize<std::tuple<Element>>
{
 static constexpr bool is_fixed_size = (traits<Element>::dimension > 0);
};

template<typename Element, typename... Elements>
struct TupleIsFixedSize<std::tuple<Element, Elements...>>
{
  static constexpr bool is_fixed_size = (traits<Element>::dimension > 0)
           && TupleIsFixedSize<std::tuple<Elements...>>::is_fixed_size;
};

// -----------------------------------------------------------------------------
// Sum the dimensions of all elements in a tuple.
template<typename T> struct TupleGetDimension;

template<typename Element>
struct TupleGetDimension<std::tuple<Element>>
{
  static constexpr int dimension = traits<Element>::dimension;
};

template<typename Element, typename... Elements>
struct TupleGetDimension<std::tuple<Element, Elements...>>
{
  static constexpr int dimension = traits<Element>::dimension
           + TupleGetDimension<std::tuple<Elements...>>::dimension;
};

} // namespace internal

// -----------------------------------------------------------------------------
// A State can contain fixed-sized elements such as Scalars, Vector3, Transformation.
template<typename... Elements>
class State
{
public:
  using StateT = State<Elements...>;
  using StateTuple = decltype(std::tuple<Elements...>());

  enum StateSize : int
  {
    size = std::tuple_size<StateTuple>::value
  };

  enum StateDimension : int
  {
    // Dimension is -1 if State is not of fixed size (e.g. contains VectorX).
    dimension = (internal::TupleIsFixedSize<StateTuple>::is_fixed_size)
      ? internal::TupleGetDimension<StateTuple>::dimension : Eigen::Dynamic
  };

  using TangentVector = Eigen::Matrix<real_t, dimension, 1> ;
  using Jacobian = Eigen::Matrix<real_t, dimension, dimension>;

  // utility
  template <size_t i>
  using ElementType = typename std::tuple_element<i, StateTuple>::type;

  StateTuple state_;

  void print() const
  {
    printImpl<0>();
    VLOG(1) << "--\n";
  }

  void retract(const TangentVector& v)
  {
    CHECK_EQ(v.size(), getDimension());
    retractImpl<0>(v, 0);
  }

  //! Get actual dimension of the state, also if state is of dynamic size.
  int getDimension() const
  {
    return dimensionImpl<0>();
  }

  //! Get reference to element.
  template<uint32_t i>
  inline auto at() -> decltype (std::get<i>(state_)) &
  {
    return std::get<i>(state_);
  }

  //! Get const reference to element.
  template<uint32_t i>
  inline auto at() const -> decltype (std::get<i>(state_)) &
  {
    return std::get<i>(state_);
  }

  //! Returns whether one element of the state is of dynamic size.
  static constexpr bool isDynamicSize()
  {
    return (dimension == -1);
  }

  //! Returns whether element i is of dynamic size.
  template<uint32_t i>
  static constexpr bool isElementDynamicSize()
  {
    return (traits<ElementType<i>>::dimension == -1);
  }

private:
  //! @name dimension recursive implementation.
  //! @{
  template<uint32_t i, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline int dimensionImpl() const
  {
    return traits<ElementType<i>>::dimension + dimensionImpl<i+1>();
  }

  template<uint32_t i, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline int dimensionImpl() const
  {
    // Element is of dynamic size.
    return traits<ElementType<i>>::getDimension(std::get<i>(state_)) + dimensionImpl<i+1>();
  }

  template<uint32_t i,typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline int dimensionImpl() const
  {
    return traits<ElementType<i>>::dimension;
  }

  template<uint32_t i, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline int dimensionImpl() const
  {
    // Element is of dynamic size.
    return traits<ElementType<i>>::getDimension(std::get<i>(state_));
  }
  //! @}

  //! @name print recursive implementation.
  //! @{
  template<uint32_t i, typename std::enable_if<(i<size)>::type* = nullptr>
  inline void printImpl() const
  {
    using T = ElementType<i>;
    VLOG(1) << "--\nState-Index: " << i << "\n"
            << "Type = " << typeid(T).name() << "\n"
            << "Dimension = " << traits<T>::dimension << "\n"
            << "Value = \n" << std::get<i>(state_) << "\n";
    printImpl<i+1>();
  }

  template<uint32_t i, typename std::enable_if<(i>=size)>::type* = nullptr>
  inline void printImpl() const
  {}
  //! @}

  //! @name retract recursive implementation.
  //! @{
  template<uint32_t i=0, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    std::get<i>(state_) = traits<T>::retract(
           std::get<i>(state_),
           v.template segment<traits<T>::dimension>(j));
    retractImpl<i+1>(v, j + traits<T>::dimension);
  }

  template<uint32_t i=0, typename std::enable_if<(i<size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    // Element is of dynamic size.
    using T = ElementType<i>;
    const int element_dim = traits<T>::getDimension(std::get<i>(state_));
    std::get<i>(state_) = traits<T>::retract(
           std::get<i>(state_), v.segment(j, element_dim));
    retractImpl<i+1>(v, j + element_dim);
  }

  template<uint32_t i=0, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension != -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    using T = ElementType<i>;
    std::get<i>(state_) = traits<T>::retract(
           std::get<i>(state_),
           v.template segment<traits<T>::dimension>(j));
  }

  template<uint32_t i=0, typename std::enable_if<(i==size-1 && traits<ElementType<i>>::dimension == -1)>::type* = nullptr>
  inline void retractImpl(const TangentVector& v, uint32_t j)
  {
    // Element is of dynamic size.
    using T = ElementType<i>;
    const int element_dim = traits<T>::getDimension(std::get<i>(state_));
    std::get<i>(state_) = traits<T>::retract(
           std::get<i>(state_), v.segment(j, element_dim));
  }
  //! @}

};

// -----------------------------------------------------------------------------
// Manifold traits for State.
template<typename T> struct traits;

template<typename... Elements>
struct traits<State<Elements...>>
{
  typedef State<Elements...> StateT;
  enum { dimension = StateT::dimension };

  typedef Eigen::Matrix<real_t, dimension, 1> TangentVector;
  typedef Eigen::Matrix<real_t, dimension, dimension> Jacobian;

  static StateT retract(
           const StateT& origin, const TangentVector& v,
           Jacobian* H1 = nullptr, Jacobian* H2 = nullptr)
  {
    if(H1 || H2)
    {
      LOG(FATAL) << "Not implemented.";
    }
    StateT origin_perturbed = origin;
    origin_perturbed.retract(v);
    return origin_perturbed;
  }

  //! @todo(cfo): Implement Retract, Equals.
};

} // namespace ze
