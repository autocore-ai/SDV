#include <type_traits>

namespace zenoh {
  namespace flow {
    struct GeometryMsgsVector3;
    struct GeometryMsgsTwist;
  }
}

namespace zenoh {
namespace flow {
#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsVector3
#define CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsVector3
struct GeometryMsgsVector3 final {
  double x;
  double y;
  double z;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsVector3

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsTwist
#define CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsTwist
struct GeometryMsgsTwist final {
  ::zenoh::flow::GeometryMsgsVector3 linear;
  ::zenoh::flow::GeometryMsgsVector3 angular;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$GeometryMsgsTwist
} // namespace flow
} // namespace zenoh
