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

#pragma once

//! A non-owning ring-buffer class.
//! Since it's non-owning, it's not allowed to construct or destroy elements
//! of the underlying container. Push and pop are implemented as assignment and
//! bookkeeping, respectively.
//!
//! Original: https://github.com/Quuxplusone/ring_view
//! License: MIT
//! On the subject of "API conventions for pushing into a fixed-size container",
//! see http://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4416.pdf

#include <cassert>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>
// detail
namespace detail {
template<class, bool> class ring_view_iterator;
} // namespace detail

template<class T>
struct null_popper
{
    void operator()(T&) { }
};

template<class T>
struct move_popper
{
    T operator()(T& t) { return std::move(t); }
};

// Will come with c++14
namespace std {

template< bool B, class T, class F >
using conditional_t = typename std::conditional<B,T,F>::type;

template< bool B, class T = void >
using enable_if_t = typename enable_if<B,T>::type;

} // namespace std

template<bool B>
using EnableIfB = typename std::enable_if<B, int>::type;

template<class T, size_t Capacity = 0, class Popper = move_popper<T>>
class ring_view
{
public:
  using type = ring_view<T, Capacity, Popper>;
  using value_type = T;
  using pointer = T*;
  using reference = T&;
  using const_reference = const T&;
  using size_type = std::size_t;
  using iterator = detail::ring_view_iterator<ring_view, false>;  // exposition only
  using const_iterator = detail::ring_view_iterator<ring_view, true>;  // exposition only
  using reverse_iterator = std::reverse_iterator<iterator>;
  using const_reverse_iterator = std::reverse_iterator<const_iterator>;

  ring_view() = default;

  // Dynamic Size Capacity Constructors
  // Construct a full ring_view.
  template<class ContiguousIterator, size_t C1 = Capacity, EnableIfB<C1 == 0> = 0>
  ring_view(ContiguousIterator begin,
            ContiguousIterator end,
            Popper p = Popper()) noexcept :
    data_(&*begin),
    size_(end - begin),
    capacity_(end - begin),
    front_idx_(0),
    popper_(std::move(p))
  {}

  // Construct a "partially full" ring_view.
  template<class ContiguousIterator, size_t C1 = Capacity, EnableIfB<C1 == 0> = 0>
  ring_view(ContiguousIterator begin,
            ContiguousIterator end,
            ContiguousIterator first,
            size_type size,
            Popper p = Popper()) noexcept :
    data_(&*begin),
    size_(size),
    capacity_(end - begin),
    front_idx_(first - begin),
    popper_(std::move(p))
  {}

  // Fixed Size Capacity Constructors
  template<class ContiguousIterator, size_t C1 = Capacity, EnableIfB<C1 != 0> = 0>
  ring_view(ContiguousIterator begin,
            ContiguousIterator end,
            Popper p = Popper()) noexcept :
    data_(&*begin),
    size_(end - begin),
    front_idx_(0),
    popper_(std::move(p))
  {
    CHECK((end - begin) == Capacity) << "Fixed size capacity must match data structure capacity.";
  }

  // Construct a "partially full" ring_view.
  template<class ContiguousIterator, size_t C1 = Capacity, EnableIfB<C1 != 0> = 0>
  ring_view(ContiguousIterator begin,
            ContiguousIterator end,
            ContiguousIterator first,
            size_type size,
            Popper p = Popper()) noexcept :
    data_(&*begin),
    size_(size),
    front_idx_(first - begin),
    popper_(std::move(p))
  {
    CHECK((end - begin) == Capacity) << "Fixed size capacity must match data structure capacity.";
  }

  // Notice that an iterator contains a pointer to the ring_view itself.
  // Destroying ring_view rv invalidates rv.begin(), just as with an owning
  // container.
  iterator begin() noexcept { return iterator(0, this); }
  iterator end() noexcept { return iterator(size(), this); }
  const_iterator begin() const noexcept { return cbegin(); }
  const_iterator end() const noexcept { return cend(); }
  const_iterator cbegin() const noexcept { return const_iterator(0, this); }
  const_iterator cend() const noexcept { return const_iterator(size(), this); }

  reverse_iterator rbegin() noexcept { return reverse_iterator(end()); }
  reverse_iterator rend() noexcept { return reverse_iterator(begin()); }
  const_reverse_iterator rbegin() const noexcept { return crbegin(); }
  const_reverse_iterator rend() const noexcept { return crend(); }
  const_reverse_iterator crbegin() const noexcept
  {
    return const_reverse_iterator(cend());
  }
  const_reverse_iterator crend() const noexcept
  {
    return const_reverse_iterator(cbegin());
  }

  reference front() noexcept { return *begin(); }
  reference back() noexcept { return *(end() - 1); }
  const_reference front() const noexcept { return *begin(); }
  const_reference back() const noexcept { return *(end() - 1); }

  // not in the spec:
  // reset the view to empty starting conditions.
  // Does not clear any data
  void reset(size_type front_idx = 0, size_type size = 0) noexcept
  {
    front_idx_ = front_idx;
    size_ = size;
  }
  void reset_front(size_type front_idx)
  {
    size_ = size_ + (front_idx_ - front_idx);
    front_idx_ = front_idx;
  }
  void reset_size(size_type size_)
  {
    size_ = size;
  }

  // state indicators
  bool empty() const noexcept { return size_ == 0; }
  bool full() const noexcept { return size_ == capacity_; }
  size_type size() const noexcept { return size_; }
  size_type capacity() const noexcept { return capacity_; }

  // pop_front() increments the index of the begin of the ring.
  // Notice that it does not destroy anything.
  // Calling pop_front() on an empty ring is undefined,
  // in the same way as calling pop_front() on an empty vector or list is
  // undefined.
  // Without pop_front(), you can't use ring_view as a std::queue.
  void pop_front()
  {
    assert(not empty());
    auto& elt = front_();
    increment_front_();
    popper_(elt);
  }

  void pop_back()
  {
    assert(not empty());
    auto& elt = back_();
    decrement_back_();
    return popper_(elt);
  }

  // push_back() assigns a new value to the element
  // at the end of the ring, and makes that element the
  // new back of the ring. If the ring is full before
  // the call to push_back(), we rotate the indices and
  // invalidate all iterators into the ring.
  // Without push_back(), you can't use ring_view as a std::queue.
  template<bool b=true,
           typename=std::enable_if_t<b && std::is_copy_assignable<T>::value>>
  void push_back(const T& value)
    noexcept(std::is_nothrow_copy_assignable<T>::value)
  {
    if (full()) {
      increment_front_and_back_();
    } else {
      increment_back_();
    }
    back_() = value;
  }

  template<bool b=true,
           typename=std::enable_if_t<b && std::is_move_assignable<T>::value>>
  void push_back(T&& value) noexcept(std::is_nothrow_move_assignable<T>::value)
  {
    if (full())
    {
        increment_front_and_back_();
    }
    else
    {
        increment_back_();
    }
    back_() = std::move(value);
  }

  void swap(ring_view& rhs) /*noexcept(std::__is_nothrow_swappable<Popper>::value) C++14 */
  {
    using std::swap;
    swap(data_, rhs.data_);
    swap(size_, rhs.size_);
    swap(capacity_, rhs.capacity_);
    swap(front_idx_, rhs.front_idx_);
    swap(popper_, rhs.popper_);
  }

  friend void swap(ring_view& lhs, ring_view& rhs)
  noexcept(noexcept(lhs.swap(rhs)))
  {
    lhs.swap(rhs);
  }

  // not public in proposal:
  // Access the i'th element of the ring, not of the underlying datastructure
  reference at(size_type i) noexcept
  {
    return data_[(front_idx_ + i) % capacity_];
  }
  const_reference at(size_type i) const noexcept
  {
    return data_[(front_idx_ + i) % capacity_];
  }

  // get the index after the last element of the buffer
  size_type back_idx() const noexcept
  {
    return (front_idx_ + size_ - 1) % capacity_;
  }

  //! get the index in the datastructure given the index of in the ring
  //! this function is not in the proposal.
  inline size_type container_idx(size_type idx) const noexcept
  {
    return (front_idx_ + idx) % capacity_;
  }

private:
  friend class detail::ring_view_iterator<ring_view, true>;
  friend class detail::ring_view_iterator<ring_view, false>;

  reference front_() noexcept { return *(data_ + front_idx_); }
  const_reference front_() const noexcept { return *(data_ + front_idx_); }
  reference back_() noexcept
  {
    return *(data_ + (front_idx_ + size_ - 1) % capacity_);
  }
  const_reference back_() const noexcept {
    return *(data_ + (front_idx_ + size_ - 1) % capacity_);
  }

  void increment_front_() noexcept
  {
    front_idx_ = (front_idx_ + 1) % capacity_;
    --size_;
  }

  void increment_back_() noexcept
  {
    ++size_;
  }
  void decrement_back_() noexcept
  {
    --size_;
  }

  void increment_front_and_back_() noexcept
  {
      front_idx_ = (front_idx_ + 1) % capacity_;
  }

  T *data_;
  size_type size_;
  std::conditional_t<Capacity == 0, const size_type, size_type> capacity_ =
      Capacity;
  size_type front_idx_;
  Popper popper_;
};

namespace detail {

//! the Iterator is 0-indexed, thus, relative to the starting point
//! of the ring.
template<class RV, bool is_const>
class ring_view_iterator
{
public:
  using type = ring_view_iterator<RV, is_const>;
  using size_type = typename RV::size_type;

  using value_type = typename RV::value_type;
  using difference_type = std::ptrdiff_t;
  using pointer = typename std::conditional_t<is_const,
                                              const value_type, value_type>*;
  using reference = typename std::conditional_t<is_const,
                                                const value_type, value_type>&;
  using iterator_category = std::random_access_iterator_tag;

  ring_view_iterator() = default;

  // not in spec:
  // the index of the iterator in the container (this is relative to the
  // starting point of the ring). Not the index in the ring
  size_type index() const { return idx_; }
  // get the index in the underlying container
  size_type container_index() const {return rv_->container_idx(idx_); }

  reference operator*() const noexcept { return rv_->at(idx_); }
  ring_view_iterator& operator++() noexcept { ++idx_; return *this; }
  ring_view_iterator operator++(int) noexcept
  {
    auto r(*this); ++*this; return r;
  }
  ring_view_iterator& operator--() noexcept { --idx_; return *this; }
  ring_view_iterator operator--(int) noexcept
  {
    auto r(*this); --*this; return r;
  }

  friend ring_view_iterator& operator+=(ring_view_iterator& it, int i) noexcept
  {
    it.idx_ += i; return it;
  }
  friend ring_view_iterator& operator-=(ring_view_iterator& it, int i) noexcept
  {
    it.idx_ -= i; return it;
  }

  friend ring_view_iterator& operator+=(ring_view_iterator& it,
                                        ring_view_iterator& it2) noexcept
  {
    it.idx_ += it2.idx_; return it;
  }
  friend ring_view_iterator& operator-=(ring_view_iterator& it,
                                        ring_view_iterator& it2) noexcept
  {
    it.idx_ -= it2.idx_; return it;
  }


  friend ring_view_iterator operator+(ring_view_iterator it, int i) noexcept
  {
    it += i; return it;
  }
  friend ring_view_iterator operator-(ring_view_iterator it, int i) noexcept
  {
    it -= i; return it;
  }

  template<bool C>
  bool operator==(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ == rhs.idx_;
  }
  template<bool C>
  bool operator!=(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ != rhs.idx_;
  }
  template<bool C>
  bool operator<(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ < rhs.idx_;
  }
  template<bool C>
  bool operator<=(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ <= rhs.idx_;
  }
  template<bool C>
  bool operator>(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ > rhs.idx_;
  }
  template<bool C>
  bool operator>=(const ring_view_iterator<RV,C>& rhs) const noexcept
  {
    return idx_ >= rhs.idx_;
  }

private:
  friend RV;
  // standard proposal does not declare these:
  // allows us to apply operates across const_iterator and iterator
  friend class ring_view_iterator<RV, true>;
  friend class ring_view_iterator<RV, false>;

  ring_view_iterator(size_type idx,
                     std::conditional_t<is_const, const RV, RV> *rv) noexcept
    : idx_(idx), rv_(rv) {}

  size_type idx_;
  std::conditional_t<is_const, const RV, RV> *rv_;
};

} // namespace detail
