#pragma once

// from Simon's multi-agent mapping framework
#define IMP_USING_INTERNAL_SMART_POINTERS(TypeName) \
  using Ptr = std::shared_ptr<TypeName>;              \
  using ConstPtr = std::shared_ptr<const TypeName>;   \
  using UniquePtr = std::unique_ptr<TypeName>;        \
  void defineUsingInternalSharedPointers##__FILE__##__LINE__(void)

#define IMP_USING_EXTERNAL_SMART_POINTERS(TypeName) \
  using Ptr = std::shared_ptr<TypeName>;              \
  using ConstPtr = std::shared_ptr<const TypeName>;   \
  using UniquePtr = std::unique_ptr<TypeName>;        \
  void defineUsingInternalSharedPointers##__FILE__##__LINE__(void)


