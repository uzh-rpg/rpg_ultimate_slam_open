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

#include <memory>
#include <atomic>
#include <thread>
#include <string>

#include <ze/common/test_entrypoint.hpp>
#include <ze/common/test_thread_blocking.hpp>
#include <ze/common/thread_safe_fifo.hpp>

using namespace ::ze;

// unnamed namespace for internal stuff
namespace {

std::atomic<unsigned> s_num_live(0);
std::atomic<unsigned> s_counter(0);

constexpr unsigned c_num_objects_per_thread = 12000;

class TestObject
{
public:
  TestObject()
  {
    s_num_live.fetch_add(1, std::memory_order_relaxed);
    counter_ = s_counter.fetch_add(1, std::memory_order_relaxed);
  }
  ~TestObject()
  {
    s_num_live.fetch_sub(1, std::memory_order_relaxed);
  }
  unsigned counter() const
  {
    return counter_;
  }
private:
  unsigned counter_;
}; // class TestObject

typedef std::shared_ptr<TestObject> TestObjectPtr;
typedef ThreadSafeFifo<TestObjectPtr, 8> TestObjectQueue;
typedef ThreadSafeFifo<TestObjectPtr, 512> TestObjectQueue2;

//------------------------------------------------------------------------------
TestObjectPtr createObj()
{
  return std::make_shared<TestObject>();
}

//------------------------------------------------------------------------------
class BlockingReadTest : public BlockingTest
{
public:
  BlockingReadTest() : queue_() { }
  ~BlockingReadTest() { }
  virtual void performBlockingAction(unsigned testId);
  virtual void performUnblockingAction(unsigned testId);
private:
  TestObjectQueue queue_;
}; // class BlockingReadTest

//------------------------------------------------------------------------------
void BlockingReadTest::performBlockingAction(unsigned testId)
{
  TestObjectPtr obj = queue_.read();
  EXPECT_TRUE(obj.get() != nullptr);
}

//------------------------------------------------------------------------------
void BlockingReadTest::performUnblockingAction(unsigned testId)
{
  queue_.write(createObj());
}

//------------------------------------------------------------------------------
class BlockingWriteTest : public BlockingTest {
public:
  BlockingWriteTest();
  ~BlockingWriteTest() { }
  virtual void performBlockingAction(unsigned testId);
  virtual void performUnblockingAction(unsigned testId);
private:
  TestObjectQueue queue_;
};

//------------------------------------------------------------------------------
BlockingWriteTest::BlockingWriteTest()
  : queue_()
{
  for (unsigned i = 0; i < 7; ++i)
  {
    queue_.write(createObj());
  }
}

//------------------------------------------------------------------------------
void BlockingWriteTest::performBlockingAction(unsigned testId)
{
  queue_.write(createObj());
}

//------------------------------------------------------------------------------
void BlockingWriteTest::performUnblockingAction(unsigned testId)
{
  TestObjectPtr obj = queue_.read();
  EXPECT_TRUE(obj.get() != nullptr);
}

//------------------------------------------------------------------------------
class WriterThread1
{
public:
  WriterThread1(TestObjectQueue2& queue)
    : queue_(queue)
  {
  }
  void operator()()
  {
    for (unsigned i = 0; i < c_num_objects_per_thread; ++i)
    {
      queue_.write(createObj());
    }
  }
private:
  TestObjectQueue2& queue_;
}; // class WriterThread1

//------------------------------------------------------------------------------
class WriterThread2
{
public:
  WriterThread2(TestObjectQueue2& queue)
  : queue_(queue)
  {
  }
  void operator()()
  {
    TestObjectQueue::UniqueLock lock(queue_.getLock());
    for (unsigned i = 0; i < c_num_objects_per_thread; ++i)
    {
      queue_.write(createObj(), lock);
    }
  }
private:
  TestObjectQueue2& queue_;
}; // class WriterThread2

//------------------------------------------------------------------------------
class ReaderThread
{
public:
  ReaderThread(TestObjectQueue2& queue, unsigned& counter)
    : queue_(queue)
    , counter_(counter)
    , run_(true)
  {
  }
  void operator()()
  {
    TestObjectPtr obj;
    while (true) {
      bool result = queue_.timedRead(obj, 100);
      if (!result)
      {
        if (!run_)
        {
          break;
        }
        continue;
      }
      EXPECT_TRUE(obj.get() != nullptr);
      counter_++;
    }
  }
  void stop() { run_ = false; }
private:
  TestObjectQueue2& queue_;
  unsigned& counter_;
  volatile bool run_;
}; // class ReaderThread

//------------------------------------------------------------------------------
void runThreadTest()
{
  TestObjectQueue2 queue;

  unsigned c1 = 0;
  unsigned c2 = 0;

  WriterThread1 w1(queue);
  WriterThread2 w2(queue);
  WriterThread1 w3(queue);
  WriterThread2 w4(queue);
  ReaderThread r1(queue, c1);
  ReaderThread r2(queue, c2);

  std::thread tw1(&WriterThread1::operator(), &w1);
  std::thread tw2(&WriterThread2::operator(), &w2);
  std::thread tw3(&WriterThread1::operator(), &w3);
  std::thread tw4(&WriterThread2::operator(), &w4);
  std::thread tr1(&ReaderThread::operator(), &r1);
  std::thread tr2(&ReaderThread::operator(), &r2);

  tw1.join();
  tw2.join();
  tw3.join();
  tw4.join();

  r1.stop();
  r2.stop();

  tr1.join();
  tr2.join();

  unsigned total = c1 + c2;
  EXPECT_EQ(4 * c_num_objects_per_thread, total);
  EXPECT_TRUE(queue.empty());
}

} // unnamed namespace

TEST(ThreadSafeFifo, Default)
{
  TestObjectQueue queue;

  //
  // write objects until the queue is full, then clear it again
  //

  s_counter = 0;
  EXPECT_TRUE(queue.empty());
  for (unsigned i = 0; i < 7; ++i)
  {
    EXPECT_EQ(i, queue.size());
    queue.write(createObj());
  }
  EXPECT_EQ(7, queue.size());
  EXPECT_EQ(7, s_num_live);
  EXPECT_TRUE(queue.full());
  EXPECT_FALSE(queue.nonBlockingWrite(createObj()));
  EXPECT_FALSE(queue.timedWrite(createObj(), 1));

  for (unsigned i = 0; i < 7; ++i)
  {
    TestObjectPtr obj = queue.read();
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i, obj->counter());
  }

  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(0, queue.size());
  EXPECT_EQ(0, s_num_live);

  TestObjectPtr obj;
  EXPECT_FALSE(queue.nonBlockingRead(obj));
  EXPECT_FALSE(queue.timedRead(obj, 1));

  s_counter=0;

  //
  // write objects until the queue is full
  // then read some objects
  // then write some more
  // then read all remaining
  //

  // write objects until the queue is full
  for (unsigned i = 0; i < 7; ++i)
  {
    TestObjectPtr obj = createObj();
    queue.write(obj); // use const reference type
  }
  EXPECT_EQ(7, queue.size());
  EXPECT_EQ(7, s_num_live);

  // read the events from the queue
  for (unsigned i = 0; i < 4; ++i)
  {
    TestObjectPtr obj = queue.read();
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i, obj->counter());
  }

  EXPECT_EQ(3, queue.size());
  EXPECT_EQ(3, s_num_live);

  for (unsigned i = 0; i < 2; ++i)
  {
    queue.write(createObj());
  }

  EXPECT_EQ(5, queue.size());
  EXPECT_EQ(5, s_num_live);

  for (unsigned i = 0; i < 5; ++i)
  {
    TestObjectPtr obj = queue.read();
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i+4, obj->counter());
  }

  EXPECT_EQ(0, queue.size());
  EXPECT_EQ(0, s_num_live);

  //
  // use non-blocking reads and writes
  //

  s_counter = 0;

  for (unsigned i = 0; i < 7; ++i)
  {
    EXPECT_TRUE(queue.nonBlockingWrite(createObj()));
  }
  EXPECT_FALSE(queue.nonBlockingWrite(createObj()));
  EXPECT_EQ(7, s_num_live);

  for (unsigned i = 0; i < 4; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.nonBlockingRead(obj));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i, obj->counter());
  }
  for (unsigned i = 0; i < 2; ++i)
  {
    EXPECT_TRUE(queue.nonBlockingWrite(createObj()));
  }
  EXPECT_EQ(5, queue.size());
  EXPECT_EQ(5, s_num_live);
  for (unsigned i = 0; i < 3; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.nonBlockingRead(obj));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i+4, obj->counter());
  }
  for (unsigned i = 0; i < 2; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.nonBlockingRead(obj));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i+8, obj->counter()); // 7 previous reads, plus one failed write
  }
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(0, queue.size());
  EXPECT_EQ(0, s_num_live);

  EXPECT_FALSE(queue.nonBlockingRead(obj));

  //
  // use timed reads and writes
  //

  s_counter = 0;

  for (unsigned i = 0; i < 7; ++i)
  {
    EXPECT_TRUE(queue.timedWrite(createObj(), 1));
  }
  EXPECT_FALSE(queue.timedWrite(createObj(), 1));
  EXPECT_EQ(7, s_num_live);

  for (unsigned i = 0; i < 4; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.timedRead(obj, 1));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i, obj->counter());
  }
  for (unsigned i = 0; i < 2; ++i)
  {
    EXPECT_TRUE(queue.timedWrite(createObj(), 1));
  }
  EXPECT_EQ(5, queue.size());
  EXPECT_EQ(5, s_num_live);
  for (unsigned i = 0; i < 3; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.timedRead(obj, 1));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i+4, obj->counter());
  }
  for (unsigned i = 0; i < 2; ++i)
  {
    TestObjectPtr obj;
    EXPECT_TRUE(queue.timedRead(obj, 1));
    ASSERT_TRUE(obj.get() != nullptr);
    EXPECT_EQ(i+8, obj->counter()); // 7 previous reads, plus one failed write
  }
  EXPECT_TRUE(queue.empty());
  EXPECT_EQ(0, queue.size());
  EXPECT_EQ(0, s_num_live);

  EXPECT_FALSE(queue.timedRead(obj, 1));

  //
  // test the clear() functionality
  //

  for (unsigned i = 0; i < 7; ++i)
  {
    queue.write(createObj());
  }
  EXPECT_EQ(7, s_num_live);
  EXPECT_FALSE(queue.empty());
  EXPECT_TRUE(queue.full());
  queue.clear();
  EXPECT_EQ(0, s_num_live);
  EXPECT_TRUE(queue.empty());
  EXPECT_FALSE(queue.full());
}

TEST(ThreadSafeFifo, StringTest)
{
  ThreadSafeFifo<std::string, 16> queue;

  queue.write("a");
  queue.write("b");
  queue.write("c");

  EXPECT_EQ("a", queue.read());
  EXPECT_EQ("b", queue.read());
  EXPECT_EQ("c", queue.read());

  for (unsigned i = 0; i < 3; i++) {
    queue.write("x");
    queue.write("y");
    queue.write("z");
  }

  EXPECT_EQ(9, queue.size());

  for (unsigned i = 0; i < 3; i++) {
    EXPECT_EQ("x", queue.read());
    EXPECT_EQ("y", queue.read());
    EXPECT_EQ("z", queue.read());
  }
}

TEST(ThreadSafeFifo, BlockingReadTest)
{
  EXPECT_EQ(0, s_num_live);
  {
    BlockingReadTest test;
    test.runBlockingTest(0, 200);
  }
  EXPECT_EQ(0, s_num_live);
}

TEST(ThreadSafeFifo, BlockingWriteTest)
{
  EXPECT_EQ(0, s_num_live);
  {
    BlockingWriteTest test;
    EXPECT_EQ(7, s_num_live);
    test.runBlockingTest(0, 200);
  }
  EXPECT_EQ(0, s_num_live);
}

TEST(ThreadSafeFifo, ThreadTest)
{
  s_counter = 0;
  runThreadTest();
}

ZE_UNITTEST_ENTRYPOINT
