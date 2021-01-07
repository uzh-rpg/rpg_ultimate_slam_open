#include <imp/core/linearmemory.hpp>

#include <cstring>
#include <algorithm>

namespace ze {

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const uint32_t& length)
  : LinearMemoryBase(length)
  , data_(Memory::alignedAlloc(this->length()))
{
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(const LinearMemory<Pixel>& from)
  : LinearMemoryBase(from)
{
  CHECK(from.data_);
  data_.reset(Memory::alignedAlloc(this->length()));
  std::copy(from.data_.get(), from.data_.get()+from.length(), data_.get());
}

//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>::LinearMemory(Pixel* host_data,
                                  const uint32_t& length,
                                  bool use_ext_data_pointer)
  : LinearMemoryBase(length)
{
  CHECK(host_data);

  if(use_ext_data_pointer)
  {
    // This uses the external data pointer and stores it as a 'reference':
    // memory won't be managed by us!
    auto dealloc_nop = [](Pixel*) { ; };
    data_ = std::unique_ptr<Pixel, Deallocator>(
          host_data, Deallocator(dealloc_nop));
  }
  else
  {
    // allocates an internal data pointer and copies the external data it.
    data_.reset(Memory::alignedAlloc(this->length()));
    std::copy(host_data, host_data+length, data_.get());
  }
}

//-----------------------------------------------------------------------------
template<typename Pixel>
Pixel* LinearMemory<Pixel>::data(uint32_t offset)
{
  CHECK_LE(offset, this->length());
  return &(data_.get()[offset]);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
const Pixel* LinearMemory<Pixel>::data(uint32_t offset) const
{
  CHECK_LE(offset, this->length());
  return reinterpret_cast<const Pixel*>(&(data_.get()[offset]));
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::setValue(const Pixel& value)
{
  std::fill(data_.get()+roi_.x(),
            data_.get()+this->roi().x()+this->roi().length(),
            value);
}

//-----------------------------------------------------------------------------
template<typename Pixel>
void LinearMemory<Pixel>::copyTo(LinearMemory<Pixel>& dst)
{
  CHECK_EQ(this->length(), dst.length());
  std::copy(data_.get(), data_.get()+this->length(), dst.data_.get());
}


//-----------------------------------------------------------------------------
template<typename Pixel>
LinearMemory<Pixel>& LinearMemory<Pixel>::operator=(Pixel rhs)
{
  this->setValue(rhs);
  return *this;
}


//=============================================================================
// Explicitely instantiate the desired classes
template class LinearMemory<ze::Pixel8uC1>;
template class LinearMemory<ze::Pixel8uC2>;
template class LinearMemory<ze::Pixel8uC3>;
template class LinearMemory<ze::Pixel8uC4>;

template class LinearMemory<ze::Pixel16uC1>;
template class LinearMemory<ze::Pixel16uC2>;
template class LinearMemory<ze::Pixel16uC3>;
template class LinearMemory<ze::Pixel16uC4>;

template class LinearMemory<ze::Pixel32uC1>;
template class LinearMemory<ze::Pixel32uC2>;
template class LinearMemory<ze::Pixel32uC3>;
template class LinearMemory<ze::Pixel32uC4>;

template class LinearMemory<ze::Pixel32sC1>;
template class LinearMemory<ze::Pixel32sC2>;
template class LinearMemory<ze::Pixel32sC3>;
template class LinearMemory<ze::Pixel32sC4>;

template class LinearMemory<ze::Pixel32fC1>;
template class LinearMemory<ze::Pixel32fC2>;
template class LinearMemory<ze::Pixel32fC3>;
template class LinearMemory<ze::Pixel32fC4>;

} // namespace ze
