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

#include <mutex>
#include <condition_variable>
#include <ze/common/noncopyable.hpp>

namespace ze {

/*!
 * @brief Thread-safe FIFO for an arbitrary number of writer and reader threads.
 *
 * The object class must be <default constructible> and <assignable>.
 *
 * The implementation blocks on read() and timedRead() if the queue is empty.
 * For timedRead() and nonBlockingRead(), a default object is returned if the
 * queue is empty.
 *
 * write() will block until the queue no longer is full.
 * nonBlockingWrite() will fail if the queue is full.
 * timedWrite() will fail if the queue is full and no object is read until the
 * timeout occurs.
 *
 * Capacity defines the buffer capacity. Note that this only defines the raw
 * buffer capacity. The actual capacity is one item less, as one sentinel slot
 * is used to differentiate between full and empty.
 *
 * If Erase is true, the element in the buffer will be overwritten with a null
 * object when the element is read.
 * If MoveSemantics is true, elements can be moved out of the buffer without
 * explicit need to clear the buffer content afterwards.
 **/
template <class T, unsigned Capacity, bool Erase=true, bool MoveSemantics=true>
class ThreadSafeFifo : Noncopyable
{
public:

  typedef std::mutex Mutex;
  typedef std::lock_guard<Mutex> LockGuard;
  typedef std::unique_lock<Mutex> UniqueLock;
  typedef std::condition_variable ConditionVariable;

  ThreadSafeFifo();
  ~ThreadSafeFifo() = default;

  /*!
   * Creates a lock for the queue and returns it.
   * Useful for external synchronization schemes.
   **/
  UniqueLock getLock() const { return UniqueLock(mutex_); }

  /*!
   * Returns the reader condition variable.
   * Useful for external synchronization schemes.
   **/
  ConditionVariable& readerConditionVariable() { return read_cond_; }

  /*!
   * Returns the writer condition variable.
   * Useful for external synchronization schemes.
   **/
  ConditionVariable& writerConditionVariable() { return write_cond_; }

  /*!
   * @name Status
   **/
  //@{

  bool empty() const;
  bool empty(UniqueLock& lock) const;

  bool full() const;
  bool full(UniqueLock& lock) const;

  unsigned size() const;
  unsigned size(UniqueLock& lock) const;

  //@} // Status

  /*!
   * @name Data Access
   **/
  //@{

  /*!
   * Writes the data element to the buffer.
   * This method will block while the buffer is full.
   **/
  //@{
  void write(const T& data);
  void write(const T& data, UniqueLock& lock);
  void write(T&& data);
  void write(T&& data, UniqueLock& lock);
  //@}

  /*!
   * Writes the data element to the buffer if the buffer is not full.
   * Returns whether the data was written or not.
   **/
  //@{
  bool nonBlockingWrite(const T& data);
  bool nonBlockingWrite(const T& data, UniqueLock& lock);
  bool nonBlockingWrite(T&& data);
  bool nonBlockingWrite(T&& data, UniqueLock& lock);
  //@}

  /*!
   * Writes the data element to the buffer.
   * This method will block for the specified timeout if the buffer is full.
   * Returns whether data was written or not.
   **/
  //@{
  bool timedWrite(const T& data, unsigned timeout);
  bool timedWrite(const T& data, unsigned timeout, UniqueLock& lock);
  bool timedWrite(T&& data, unsigned timeout);
  bool timedWrite(T&& data, unsigned timeout, UniqueLock& lock);
  //@}

  /*!
   * Returns the next element from the queue.
   * The method blocks until data is available.
   **/
  //@{
  T read();
  T read(UniqueLock& lock);
  //@}

  /*!
   * Reads the next element (if available) into the provided variable.
   * Returns whether data was read or not.
   **/
  //@{
  bool nonBlockingRead(T& data);
  bool nonBlockingRead(T& data, UniqueLock& lock);
  //@}

  /*!
   * Reads the next element (if available) into the provided variable.
   * This method will block for the specified timeout if no data is available.
   * Returns whether data was read or not.
   **/
  //@{
  bool timedRead(T& data, unsigned timeout);
  bool timedRead(T& data, unsigned timeout, UniqueLock& lock);
  //@}

  /*!
   * Clears the content of the queue.
   **/
  //@{
  void clear();
  void clear(UniqueLock& lock);
  //@}

  //@} // Data Access

private:

  bool _empty() const;
  bool _full() const;
  unsigned _size() const;
  void _write(const T& data);
  void _write(T&& data);
  T _read();
  void _clear();
  unsigned _nextIndex(unsigned index) const;
  bool _notEmpty() const;
  bool _notFull() const;

  mutable Mutex mutex_;
  ConditionVariable read_cond_;
  ConditionVariable write_cond_;

  std::array<T, Capacity> buf_;
  unsigned tail_; // writer end
  unsigned head_; // reader end

}; // ThreadSafeFifo

//------------------------------------------------------------------------------
// implementation
//

template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::ThreadSafeFifo()
  : mutex_()
  , read_cond_()
  , write_cond_()
  , buf_()
  , tail_(0)
  , head_(0)
{
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::empty() const
{
  LockGuard lock(mutex_);
  return _empty();
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool
ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::empty(UniqueLock& lock) const
{
  return _empty();
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::full() const
{
  LockGuard lock(mutex_);
  return _full();
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool
ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::full(UniqueLock& lock) const
{
  return _full();
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
unsigned ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::size() const
{
  LockGuard lock(mutex_);
  return _size();
}

//------------------------------------------------------------------------------
template <typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
unsigned
ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::size(UniqueLock& lock) const
{
  return _size();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::write(const T& data)
{
  UniqueLock lock(mutex_);
  write(data, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void
ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::write(const T& data,
                                                         UniqueLock& lock)
{
  write_cond_.wait(lock, [this]{ return _notFull(); });
  _write(data);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::write(T&& data)
{
  UniqueLock lock(mutex_);
  write(std::forward<T>(data), lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::write(T&& data,
                                                              UniqueLock& lock)
{
  write_cond_.wait(lock, [this]{ return _notFull(); });
  _write(std::forward<T>(data));
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::nonBlockingWrite(const T& data)
{
  UniqueLock lock(mutex_);
  return nonBlockingWrite(data, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::nonBlockingWrite(const T& data, UniqueLock& lock)
{
  if (_full())
  {
    return false;
  }
  _write(data);
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::nonBlockingWrite(T&& data)
{
  UniqueLock lock(mutex_);
  return nonBlockingWrite(std::forward<T>(data), lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::nonBlockingWrite(T&& data, UniqueLock& lock)
{
  if (_full())
  {
    return false;
  }

  _write(std::forward<T>(data));
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedWrite(const T& data, unsigned timeout)
{
  UniqueLock lock(mutex_);
  return timedWrite(data, timeout, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedWrite(const T& data, unsigned timeout, UniqueLock& lock)
{
  if (!write_cond_.wait_for(lock, std::chrono::milliseconds(timeout),
                                     [this]{ return _notFull(); }))
  {
    return false;
  }
  _write(data);
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedWrite(T&& data, unsigned timeout)
{
  UniqueLock lock(mutex_);
  return timedWrite(std::forward<T>(data), timeout, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedWrite(T&& data, unsigned timeout, UniqueLock& lock)
{
  if (!write_cond_.wait_for(lock, std::chrono::milliseconds(timeout),
                              [this]{ return _notFull(); }))
  {
    return false;
  }

  _write(std::forward<T>(data));
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
T ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::read()
{
  UniqueLock lock(mutex_);
  return read(lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
T ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::read(UniqueLock& lock)
{
  read_cond_.wait(lock, [this]{ return _notEmpty(); });

  return _read();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::nonBlockingRead(T& data)
{
  UniqueLock lock(mutex_);
  return nonBlockingRead(data, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::nonBlockingRead(T& data, UniqueLock& lock)
{
  if (_empty())
  {
    return false;
  }

  data = _read();
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedRead(T& data, unsigned timeout)
{
  UniqueLock lock(mutex_);
  return timedRead(data, timeout, lock);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::timedRead(T& data, unsigned timeout, UniqueLock& lock)
{
  if (!read_cond_.wait_for(lock, std::chrono::milliseconds(timeout),
                           [this]{ return _notEmpty(); }))
  {
    return false;
  }

  data = _read();
  return true;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::clear()
{
  LockGuard lock(mutex_);
  _clear();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::clear(UniqueLock& lock)
{
  _clear();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_empty() const
{
  return (tail_ == head_);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_full() const
{
  return (_nextIndex(tail_) == head_);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
unsigned ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_size() const
{
  return (tail_ < head_) ? ((tail_ + Capacity) - head_) : (tail_ - head_);
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_write(const T& data)
{
  buf_[tail_] = data;
  tail_ = _nextIndex(tail_);
  read_cond_.notify_one();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_write(T&& data)
{
  buf_[tail_] = std::forward<T>(data);
  tail_ = _nextIndex(tail_);
  read_cond_.notify_one();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
T ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_read()
{
  T data = std::forward<T>(buf_[head_]);
  if (Erase and not MoveSemantics)
  {
    buf_[head_] = T();
  }
  head_ = _nextIndex(head_);

  write_cond_.notify_one();
  return data;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
void ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_clear()
{
  if (Erase) {
    buf_.fill(T());
  }

  tail_ = 0;
  head_ = 0;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
unsigned ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>
::_nextIndex(unsigned index) const
{
  index++;
  if (index == Capacity)
  {
    index = 0;
  }
  return index;
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_notEmpty() const
{
  return !_empty();
}

//------------------------------------------------------------------------------
template<typename T, unsigned Capacity, bool Erase, bool MoveSemantics>
bool ThreadSafeFifo<T, Capacity, Erase, MoveSemantics>::_notFull() const
{
  return !_full();
}

} // namespace ze
