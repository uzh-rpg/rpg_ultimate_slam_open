#pragma once

#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <functional>
#include <algorithm>

#include <ze/common/logging.hpp>
#include <ze/common/types.hpp>
#include <imp/core/size.hpp>
#include <imp/core/types.hpp>

namespace ze {

//--------------------------------------------------------------------------
template <typename Pixel, int memaddr_align=32, bool align_rows=true>
struct MemoryStorage
{
public:
  MemoryStorage() = delete;
  virtual ~MemoryStorage() = delete;

  /**
   * @brief isAligned checks if the given data pointer \a p is aligned according to \a memaddr_align
   */
  static bool isAligned(void* p)
  {
    return reinterpret_cast<intptr_t>(p) % memaddr_align == 0;
  }

  /**
   * @brief alignedAlloc allocates an aligned block of memory
   * @param num_elements Number of (minimum) allocated elements
   * @param init_with_zeros Flag if the memory elements should be zeroed out (default=false).
   *
   * @note Internally we use the C11 function aligned_alloc although there
   *       are also alignment functions in C++11 but aligned_alloc is the only
   *       one where we don't have to mess around with allocated bigger chuncks of
   *       memory and shifting the start address accordingly. If you know a
   *       better approach using e.g. std::align(), let me know.
   */
  static Pixel* alignedAlloc(const uint32_t num_elements,
                             bool init_with_zeros=false)
  {
    CHECK_GT(num_elements, 0u) << "Failed to allocate memory: num_elements=0";

    // restrict the memory address alignment to be in the interval ]0,128] and
    // of power-of-two using the 'complement and compare' method
    assert((memaddr_align != 0) && memaddr_align <= 128 &&
           ((memaddr_align & (~memaddr_align + 1)) == memaddr_align));

    const uint32_t memory_size = sizeof(Pixel) * num_elements;

    Pixel* p_data_aligned;
    int ret = posix_memalign((void**)&p_data_aligned, memaddr_align, memory_size);
    // alternatively one could use -- benchmark before you change this
    // Pixel* p_data_aligned =
    //     (Pixel*)aligned_alloc(memaddr_align, memory_size);

    if (p_data_aligned == nullptr || ret != 0)
    {
      throw std::bad_alloc();
    }

    if (init_with_zeros)
    {
      std::fill(p_data_aligned, p_data_aligned+num_elements, Pixel(0));
    }

    return (Pixel*)p_data_aligned;
  }

  /**
   * @brief alignedAlloc allocates an aligned block of memory that guarantees to host the image of size \a size
   * @param size Image size
   * @param pitch Row alignment [bytes] if padding is needed.
   * @param init_with_zeros Flag if the memory elements should be zeroed out (default=false).
   * @return
   */
  static Pixel* alignedAlloc(
      ze::Size2u size, uint32_t* pitch, bool init_with_zeros=false)
  {
    CHECK_GT(size.width(), 0u);
    CHECK_GT(size.height(), 0u);

    // restrict the memory address alignment to be in the interval ]0,128] and
    // of power-of-two using the 'complement and compare' method
    assert((memaddr_align != 0) && memaddr_align <= 128 &&
           ((memaddr_align & (~memaddr_align + 1)) == memaddr_align));

    // check if the width allows a correct alignment of every row, otherwise add padding
    const uint32_t width_bytes = size.width() * sizeof(Pixel);
    // bytes % memaddr_align = 0 for bytes=n*memaddr_align is the reason for
    // the decrement in the following compution:
    const uint32_t bytes_to_add = (memaddr_align-1) - ((width_bytes-1) % memaddr_align);
    const uint32_t pitched_width = size.width() + bytes_to_add/sizeof(Pixel);
    *pitch = width_bytes + bytes_to_add;
    return alignedAlloc(pitched_width*size.height(), init_with_zeros);
  }


  /**
   * @brief free releases the pixel \a buffer
   * @param buffer
   */
  static void free(Pixel* buffer)
  {
    free(buffer);
  }
}; // struct MemoryStorage

/**
 * @brief The MemoryDeallocator struct offers the ability to have custom deallocation methods.
 *
 * The Deallocator struct can be used as e.g. having custom deallocations with
 * shared pointers. Furthermore it enables the usage of external memory buffers
 * using shared pointers but not taking ownership of the memory. Be careful when
 * doing so as an application would behave badly if the memory got deleted although
 * we are still using it.
 *
 */
template<typename Pixel>
class MemoryDeallocator
{
public:
  // Default custom deleter assuming we use arrays (new PixelType[length])
  MemoryDeallocator()
  {
    f_ = [](Pixel* p) {free(p);};
  }

  // allow us to define a custom deallocator
  explicit MemoryDeallocator(std::function<void(Pixel*)> const &f)
    : f_(f)
  { }

  void operator()(Pixel* p) const
  {
    f_(p);
  }

private:
  std::function< void(Pixel* )> f_;
}; // MemoryDeallocator

} // imp

