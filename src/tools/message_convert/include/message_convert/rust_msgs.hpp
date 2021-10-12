#pragma once
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <iterator>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace rust {
inline namespace cxxbridge1 {
// #include "rust/cxx.h"

#ifndef CXXBRIDGE1_PANIC
#define CXXBRIDGE1_PANIC
template <typename Exception>
void panic [[noreturn]] (const char *msg);
#endif // CXXBRIDGE1_PANIC

struct unsafe_bitcopy_t;

namespace {
template <typename T>
class impl;
} // namespace

class Opaque;

template <typename T>
::std::size_t size_of();
template <typename T>
::std::size_t align_of();

#ifndef CXXBRIDGE1_RUST_STRING
#define CXXBRIDGE1_RUST_STRING
class String final {
public:
  String() noexcept;
  String(const String &) noexcept;
  String(String &&) noexcept;
  ~String() noexcept;

  String(const std::string &);
  String(const char *);
  String(const char *, std::size_t);
  String(const char16_t *);
  String(const char16_t *, std::size_t);

  String &operator=(const String &) &noexcept;
  String &operator=(String &&) &noexcept;

  explicit operator std::string() const;

  const char *data() const noexcept;
  std::size_t size() const noexcept;
  std::size_t length() const noexcept;
  bool empty() const noexcept;

  const char *c_str() noexcept;

  std::size_t capacity() const noexcept;
  void reserve(size_t new_cap) noexcept;

  using iterator = char *;
  iterator begin() noexcept;
  iterator end() noexcept;

  using const_iterator = const char *;
  const_iterator begin() const noexcept;
  const_iterator end() const noexcept;
  const_iterator cbegin() const noexcept;
  const_iterator cend() const noexcept;

  bool operator==(const String &) const noexcept;
  bool operator!=(const String &) const noexcept;
  bool operator<(const String &) const noexcept;
  bool operator<=(const String &) const noexcept;
  bool operator>(const String &) const noexcept;
  bool operator>=(const String &) const noexcept;

  void swap(String &) noexcept;

  String(unsafe_bitcopy_t, const String &) noexcept;

private:
  friend void swap(String &lhs, String &rhs) noexcept { lhs.swap(rhs); }

  std::array<std::uintptr_t, 3> repr;
};
#endif // CXXBRIDGE1_RUST_STRING

#ifndef CXXBRIDGE1_RUST_SLICE
#define CXXBRIDGE1_RUST_SLICE
namespace detail {
template <bool>
struct copy_assignable_if {};

template <>
struct copy_assignable_if<false> {
  copy_assignable_if() noexcept = default;
  copy_assignable_if(const copy_assignable_if &) noexcept = default;
  copy_assignable_if &operator=(const copy_assignable_if &) &noexcept = delete;
  copy_assignable_if &operator=(copy_assignable_if &&) &noexcept = default;
};
} // namespace detail

template <typename T>
class Slice final
    : private detail::copy_assignable_if<std::is_const<T>::value> {
public:
  using value_type = T;

  Slice() noexcept;
  Slice(T *, std::size_t count) noexcept;

  Slice &operator=(const Slice<T> &) &noexcept = default;
  Slice &operator=(Slice<T> &&) &noexcept = default;

  T *data() const noexcept;
  std::size_t size() const noexcept;
  std::size_t length() const noexcept;
  bool empty() const noexcept;

  T &operator[](std::size_t n) const noexcept;
  T &at(std::size_t n) const;
  T &front() const noexcept;
  T &back() const noexcept;

  Slice(const Slice<T> &) noexcept = default;
  ~Slice() noexcept = default;

  class iterator;
  iterator begin() const noexcept;
  iterator end() const noexcept;

  void swap(Slice &) noexcept;

private:
  class uninit;
  Slice(uninit) noexcept;
  friend impl<Slice>;
  friend void sliceInit(void *, const void *, std::size_t) noexcept;
  friend void *slicePtr(const void *) noexcept;
  friend std::size_t sliceLen(const void *) noexcept;

  std::array<std::uintptr_t, 2> repr;
};

template <typename T>
class Slice<T>::iterator final {
public:
  using iterator_category = std::random_access_iterator_tag;
  using value_type = T;
  using difference_type = std::ptrdiff_t;
  using pointer = typename std::add_pointer<T>::type;
  using reference = typename std::add_lvalue_reference<T>::type;

  reference operator*() const noexcept;
  pointer operator->() const noexcept;
  reference operator[](difference_type) const noexcept;

  iterator &operator++() noexcept;
  iterator operator++(int) noexcept;
  iterator &operator--() noexcept;
  iterator operator--(int) noexcept;

  iterator &operator+=(difference_type) noexcept;
  iterator &operator-=(difference_type) noexcept;
  iterator operator+(difference_type) const noexcept;
  iterator operator-(difference_type) const noexcept;
  difference_type operator-(const iterator &) const noexcept;

  bool operator==(const iterator &) const noexcept;
  bool operator!=(const iterator &) const noexcept;
  bool operator<(const iterator &) const noexcept;
  bool operator<=(const iterator &) const noexcept;
  bool operator>(const iterator &) const noexcept;
  bool operator>=(const iterator &) const noexcept;

private:
  friend class Slice;
  void *pos;
  std::size_t stride;
};

template <typename T>
Slice<T>::Slice() noexcept {
  sliceInit(this, reinterpret_cast<void *>(align_of<T>()), 0);
}

template <typename T>
Slice<T>::Slice(T *s, std::size_t count) noexcept {
  assert(s != nullptr || count == 0);
  sliceInit(this,
            s == nullptr && count == 0
                ? reinterpret_cast<void *>(align_of<T>())
                : const_cast<typename std::remove_const<T>::type *>(s),
            count);
}

template <typename T>
T *Slice<T>::data() const noexcept {
  return reinterpret_cast<T *>(slicePtr(this));
}

template <typename T>
std::size_t Slice<T>::size() const noexcept {
  return sliceLen(this);
}

template <typename T>
std::size_t Slice<T>::length() const noexcept {
  return this->size();
}

template <typename T>
bool Slice<T>::empty() const noexcept {
  return this->size() == 0;
}

template <typename T>
T &Slice<T>::operator[](std::size_t n) const noexcept {
  assert(n < this->size());
  auto ptr = static_cast<char *>(slicePtr(this)) + size_of<T>() * n;
  return *reinterpret_cast<T *>(ptr);
}

template <typename T>
T &Slice<T>::at(std::size_t n) const {
  if (n >= this->size()) {
    panic<std::out_of_range>("rust::Slice index out of range");
  }
  return (*this)[n];
}

template <typename T>
T &Slice<T>::front() const noexcept {
  assert(!this->empty());
  return (*this)[0];
}

template <typename T>
T &Slice<T>::back() const noexcept {
  assert(!this->empty());
  return (*this)[this->size() - 1];
}

template <typename T>
typename Slice<T>::iterator::reference
Slice<T>::iterator::operator*() const noexcept {
  return *static_cast<T *>(this->pos);
}

template <typename T>
typename Slice<T>::iterator::pointer
Slice<T>::iterator::operator->() const noexcept {
  return static_cast<T *>(this->pos);
}

template <typename T>
typename Slice<T>::iterator::reference Slice<T>::iterator::operator[](
    typename Slice<T>::iterator::difference_type n) const noexcept {
  auto ptr = static_cast<char *>(this->pos) + this->stride * n;
  return *reinterpret_cast<T *>(ptr);
}

template <typename T>
typename Slice<T>::iterator &Slice<T>::iterator::operator++() noexcept {
  this->pos = static_cast<char *>(this->pos) + this->stride;
  return *this;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::iterator::operator++(int) noexcept {
  auto ret = iterator(*this);
  this->pos = static_cast<char *>(this->pos) + this->stride;
  return ret;
}

template <typename T>
typename Slice<T>::iterator &Slice<T>::iterator::operator--() noexcept {
  this->pos = static_cast<char *>(this->pos) - this->stride;
  return *this;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::iterator::operator--(int) noexcept {
  auto ret = iterator(*this);
  this->pos = static_cast<char *>(this->pos) - this->stride;
  return ret;
}

template <typename T>
typename Slice<T>::iterator &Slice<T>::iterator::operator+=(
    typename Slice<T>::iterator::difference_type n) noexcept {
  this->pos = static_cast<char *>(this->pos) + this->stride * n;
  return *this;
}

template <typename T>
typename Slice<T>::iterator &Slice<T>::iterator::operator-=(
    typename Slice<T>::iterator::difference_type n) noexcept {
  this->pos = static_cast<char *>(this->pos) - this->stride * n;
  return *this;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::iterator::operator+(
    typename Slice<T>::iterator::difference_type n) const noexcept {
  auto ret = iterator(*this);
  ret.pos = static_cast<char *>(this->pos) + this->stride * n;
  return ret;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::iterator::operator-(
    typename Slice<T>::iterator::difference_type n) const noexcept {
  auto ret = iterator(*this);
  ret.pos = static_cast<char *>(this->pos) - this->stride * n;
  return ret;
}

template <typename T>
typename Slice<T>::iterator::difference_type
Slice<T>::iterator::operator-(const iterator &other) const noexcept {
  auto diff = std::distance(static_cast<char *>(other.pos),
                            static_cast<char *>(this->pos));
  return diff / static_cast<long int>(this->stride);
}

template <typename T>
bool Slice<T>::iterator::operator==(const iterator &other) const noexcept {
  return this->pos == other.pos;
}

template <typename T>
bool Slice<T>::iterator::operator!=(const iterator &other) const noexcept {
  return this->pos != other.pos;
}

template <typename T>
bool Slice<T>::iterator::operator<(const iterator &other) const noexcept {
  return this->pos < other.pos;
}

template <typename T>
bool Slice<T>::iterator::operator<=(const iterator &other) const noexcept {
  return this->pos <= other.pos;
}

template <typename T>
bool Slice<T>::iterator::operator>(const iterator &other) const noexcept {
  return this->pos > other.pos;
}

template <typename T>
bool Slice<T>::iterator::operator>=(const iterator &other) const noexcept {
  return this->pos >= other.pos;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::begin() const noexcept {
  iterator it;
  it.pos = slicePtr(this);
  it.stride = size_of<T>();
  return it;
}

template <typename T>
typename Slice<T>::iterator Slice<T>::end() const noexcept {
  iterator it = this->begin();
  it.pos = static_cast<char *>(it.pos) + it.stride * this->size();
  return it;
}

template <typename T>
void Slice<T>::swap(Slice &rhs) noexcept {
  std::swap(*this, rhs);
}
#endif // CXXBRIDGE1_RUST_SLICE

#ifndef CXXBRIDGE1_RUST_BITCOPY_T
#define CXXBRIDGE1_RUST_BITCOPY_T
struct unsafe_bitcopy_t final {
  explicit unsafe_bitcopy_t() = default;
};
#endif // CXXBRIDGE1_RUST_BITCOPY_T

#ifndef CXXBRIDGE1_RUST_VEC
#define CXXBRIDGE1_RUST_VEC
template <typename T>
class Vec final {
public:
  using value_type = T;

  Vec() noexcept;
  Vec(std::initializer_list<T>);
  Vec(const Vec &);
  Vec(Vec &&) noexcept;
  ~Vec() noexcept;

  Vec &operator=(Vec &&) &noexcept;
  Vec &operator=(const Vec &) &;

  std::size_t size() const noexcept;
  bool empty() const noexcept;
  const T *data() const noexcept;
  T *data() noexcept;
  std::size_t capacity() const noexcept;

  const T &operator[](std::size_t n) const noexcept;
  const T &at(std::size_t n) const;
  const T &front() const noexcept;
  const T &back() const noexcept;

  T &operator[](std::size_t n) noexcept;
  T &at(std::size_t n);
  T &front() noexcept;
  T &back() noexcept;

  void reserve(std::size_t new_cap);
  void push_back(const T &value);
  void push_back(T &&value);
  template <typename... Args>
  void emplace_back(Args &&...args);

  using iterator = typename Slice<T>::iterator;
  iterator begin() noexcept;
  iterator end() noexcept;

  using const_iterator = typename Slice<const T>::iterator;
  const_iterator begin() const noexcept;
  const_iterator end() const noexcept;
  const_iterator cbegin() const noexcept;
  const_iterator cend() const noexcept;

  void swap(Vec &) noexcept;

  Vec(unsafe_bitcopy_t, const Vec &) noexcept;

private:
  void reserve_total(std::size_t new_cap) noexcept;
  void set_len(std::size_t len) noexcept;
  void drop() noexcept;

  friend void swap(Vec &lhs, Vec &rhs) noexcept { lhs.swap(rhs); }

  std::array<std::uintptr_t, 3> repr;
};

template <typename T>
Vec<T>::Vec(std::initializer_list<T> init) : Vec{} {
  this->reserve_total(init.size());
  std::move(init.begin(), init.end(), std::back_inserter(*this));
}

template <typename T>
Vec<T>::Vec(const Vec &other) : Vec() {
  this->reserve_total(other.size());
  std::copy(other.begin(), other.end(), std::back_inserter(*this));
}

template <typename T>
Vec<T>::Vec(Vec &&other) noexcept : repr(other.repr) {
  new (&other) Vec();
}

template <typename T>
Vec<T>::~Vec() noexcept {
  this->drop();
}

template <typename T>
Vec<T> &Vec<T>::operator=(Vec &&other) &noexcept {
  this->drop();
  this->repr = other.repr;
  new (&other) Vec();
  return *this;
}

template <typename T>
Vec<T> &Vec<T>::operator=(const Vec &other) & {
  if (this != &other) {
    this->drop();
    new (this) Vec(other);
  }
  return *this;
}

template <typename T>
bool Vec<T>::empty() const noexcept {
  return this->size() == 0;
}

template <typename T>
T *Vec<T>::data() noexcept {
  return const_cast<T *>(const_cast<const Vec<T> *>(this)->data());
}

template <typename T>
const T &Vec<T>::operator[](std::size_t n) const noexcept {
  assert(n < this->size());
  auto data = reinterpret_cast<const char *>(this->data());
  return *reinterpret_cast<const T *>(data + n * size_of<T>());
}

template <typename T>
const T &Vec<T>::at(std::size_t n) const {
  if (n >= this->size()) {
    panic<std::out_of_range>("rust::Vec index out of range");
  }
  return (*this)[n];
}

template <typename T>
const T &Vec<T>::front() const noexcept {
  assert(!this->empty());
  return (*this)[0];
}

template <typename T>
const T &Vec<T>::back() const noexcept {
  assert(!this->empty());
  return (*this)[this->size() - 1];
}

template <typename T>
T &Vec<T>::operator[](std::size_t n) noexcept {
  assert(n < this->size());
  auto data = reinterpret_cast<char *>(this->data());
  return *reinterpret_cast<T *>(data + n * size_of<T>());
}

template <typename T>
T &Vec<T>::at(std::size_t n) {
  if (n >= this->size()) {
    panic<std::out_of_range>("rust::Vec index out of range");
  }
  return (*this)[n];
}

template <typename T>
T &Vec<T>::front() noexcept {
  assert(!this->empty());
  return (*this)[0];
}

template <typename T>
T &Vec<T>::back() noexcept {
  assert(!this->empty());
  return (*this)[this->size() - 1];
}

template <typename T>
void Vec<T>::reserve(std::size_t new_cap) {
  this->reserve_total(new_cap);
}

template <typename T>
void Vec<T>::push_back(const T &value) {
  this->emplace_back(value);
}

template <typename T>
void Vec<T>::push_back(T &&value) {
  this->emplace_back(std::move(value));
}

template <typename T>
template <typename... Args>
void Vec<T>::emplace_back(Args &&...args) {
  auto size = this->size();
  this->reserve_total(size + 1);
  ::new (reinterpret_cast<T *>(reinterpret_cast<char *>(this->data()) +
                               size * size_of<T>()))
      T(std::forward<Args>(args)...);
  this->set_len(size + 1);
}

template <typename T>
typename Vec<T>::iterator Vec<T>::begin() noexcept {
  return Slice<T>(this->data(), this->size()).begin();
}

template <typename T>
typename Vec<T>::iterator Vec<T>::end() noexcept {
  return Slice<T>(this->data(), this->size()).end();
}

template <typename T>
typename Vec<T>::const_iterator Vec<T>::begin() const noexcept {
  return this->cbegin();
}

template <typename T>
typename Vec<T>::const_iterator Vec<T>::end() const noexcept {
  return this->cend();
}

template <typename T>
typename Vec<T>::const_iterator Vec<T>::cbegin() const noexcept {
  return Slice<const T>(this->data(), this->size()).begin();
}

template <typename T>
typename Vec<T>::const_iterator Vec<T>::cend() const noexcept {
  return Slice<const T>(this->data(), this->size()).end();
}

template <typename T>
void Vec<T>::swap(Vec &rhs) noexcept {
  using std::swap;
  swap(this->repr, rhs.repr);
}

template <typename T>
Vec<T>::Vec(unsafe_bitcopy_t, const Vec &bits) noexcept : repr(bits.repr) {}
#endif // CXXBRIDGE1_RUST_VEC

#ifndef CXXBRIDGE1_IS_COMPLETE
#define CXXBRIDGE1_IS_COMPLETE
namespace detail {
namespace {
template <typename T, typename = std::size_t>
struct is_complete : std::false_type {};
template <typename T>
struct is_complete<T, decltype(sizeof(T))> : std::true_type {};
} // namespace
} // namespace detail
#endif // CXXBRIDGE1_IS_COMPLETE

#ifndef CXXBRIDGE1_LAYOUT
#define CXXBRIDGE1_LAYOUT
class layout {
  template <typename T>
  friend std::size_t size_of();
  template <typename T>
  friend std::size_t align_of();
  template <typename T>
  static typename std::enable_if<std::is_base_of<Opaque, T>::value,
                                 std::size_t>::type
  do_size_of() {
    return T::layout::size();
  }
  template <typename T>
  static typename std::enable_if<!std::is_base_of<Opaque, T>::value,
                                 std::size_t>::type
  do_size_of() {
    return sizeof(T);
  }
  template <typename T>
  static
      typename std::enable_if<detail::is_complete<T>::value, std::size_t>::type
      size_of() {
    return do_size_of<T>();
  }
  template <typename T>
  static typename std::enable_if<std::is_base_of<Opaque, T>::value,
                                 std::size_t>::type
  do_align_of() {
    return T::layout::align();
  }
  template <typename T>
  static typename std::enable_if<!std::is_base_of<Opaque, T>::value,
                                 std::size_t>::type
  do_align_of() {
    return alignof(T);
  }
  template <typename T>
  static
      typename std::enable_if<detail::is_complete<T>::value, std::size_t>::type
      align_of() {
    return do_align_of<T>();
  }
};

template <typename T>
std::size_t size_of() {
  return layout::size_of<T>();
}

template <typename T>
std::size_t align_of() {
  return layout::align_of<T>();
}
#endif // CXXBRIDGE1_LAYOUT
} // namespace cxxbridge1
} // namespace rust

namespace zenoh_flow {
  namespace autoware_auto {
    struct builtin_interfaces_Time;
    struct autoware_auto_msgs_HighLevelControlCommand;
    struct autoware_auto_msgs_VehicleStateReport;
    struct autoware_auto_msgs_StationaryLockingCommand;
    struct autoware_auto_msgs_VehicleOdometry;
    struct geometry_msgs_Point;
    struct autoware_auto_msgs_VehicleKinematicState;
    enum class autoware_auto_msgs_VehicleStateCommand_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_DetectedObjects;
    enum class autoware_auto_msgs_VehicleStateReport_Constants : ::std::uint8_t;
    struct geometry_msgs_PoseWithCovarianceStamped;
    enum class autoware_auto_msgs_Trajectory_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_HADMapRoute;
    struct autoware_auto_msgs_DiagnosticHeader;
    struct autoware_auto_msgs_HazardLightsReport;
    struct autoware_auto_msgs_TrackedObjectKinematics;
    struct autoware_auto_msgs_WipersCommand;
    struct autoware_auto_msgs_RawControlCommand;
    struct autoware_auto_msgs_TrajectoryPoint;
    struct std_msgs_Header;
    struct autoware_auto_msgs_Trajectory;
    struct autoware_auto_msgs_GearReport;
    enum class autoware_auto_msgs_TurnIndicatorsReport_Constants : ::std::uint8_t;
    struct geometry_msgs_Point32;
    enum class autoware_auto_msgs_HazardLightsCommand_Constants : ::std::uint8_t;
    enum class autoware_auto_msgs_WipersReport_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_AckermannLateralCommand;
    struct builtin_interfaces_Duration;
    struct geometry_msgs_Accel;
    struct autoware_auto_msgs_RelativePositionWithCovarianceStamped;
    struct autoware_auto_msgs_Complex32;
    struct autoware_auto_msgs_ObjectClassification;
    struct autoware_auto_msgs_TurnIndicatorsReport;
    enum class autoware_auto_msgs_DetectedObjectKinematics_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_HandBrakeCommand;
    struct autoware_auto_msgs_PointXYZIF;
    enum class autoware_auto_msgs_TurnIndicatorsCommand_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_HADMapBin;
    struct autoware_auto_msgs_RoutePoint;
    struct autoware_auto_msgs_PredictedObjectKinematics;
    enum class autoware_auto_msgs_HazardLightsReport_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_TurnIndicatorsCommand;
    struct autoware_auto_msgs_TrackedObjects;
    struct autoware_auto_msgs_MapPrimitive;
    struct autoware_auto_msgs_HandBrakeReport;
    struct autoware_auto_msgs_HornReport;
    struct geometry_msgs_Vector3;
    struct autoware_auto_msgs_PredictedObject;
    struct geometry_msgs_Polygon;
    struct geometry_msgs_Twist;
    struct autoware_auto_msgs_PredictedPath;
    struct autoware_auto_msgs_Quaternion32;
    struct autoware_auto_msgs_PointClusters;
    struct geometry_msgs_TwistWithCovariance;
    struct geometry_msgs_Pose;
    struct geometry_msgs_Transform;
    struct autoware_auto_msgs_ClassifiedRoi;
    struct autoware_auto_msgs_HazardLightsCommand;
    struct autoware_auto_msgs_Shape;
    struct autoware_auto_msgs_Route;
    struct autoware_auto_msgs_HeadlightsReport;
    struct autoware_auto_msgs_AckermannControlCommand;
    struct geometry_msgs_Quaternion;
    enum class autoware_auto_msgs_ObjectClassification_Constants : ::std::uint8_t;
    enum class autoware_auto_msgs_Route_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_HADMapSegment;
    struct autoware_auto_msgs_ControlDiagnostic;
    struct autoware_auto_msgs_BoundingBoxArray;
    struct geometry_msgs_PoseWithCovariance;
    enum class autoware_auto_msgs_BoundingBox_Constants : ::std::uint8_t;
    enum class autoware_auto_msgs_HADMapBin_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_VehicleStateCommand;
    enum class autoware_auto_msgs_HeadlightsReport_Constants : ::std::uint8_t;
    struct geometry_msgs_AccelWithCovariance;
    struct autoware_auto_msgs_DetectedObject;
    enum class autoware_auto_msgs_HeadlightsCommand_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_WheelEncoder;
    struct autoware_auto_msgs_HornCommand;
    enum class autoware_auto_msgs_TrackedObjectKinematics_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_VehicleControlCommand;
    struct autoware_auto_msgs_LongitudinalCommand;
    enum class autoware_auto_msgs_BoundingBoxArray_Constants : ::std::uint16_t;
    struct autoware_auto_msgs_TrackedObject;
    struct autoware_auto_msgs_PredictedObjects;
    struct autoware_auto_msgs_BoundingBox;
    enum class autoware_auto_msgs_GearReport_Constants : ::std::uint8_t;
    enum class autoware_auto_msgs_PointXYZIF_Constants : ::std::uint16_t;
    struct autoware_auto_msgs_WipersReport;
    struct autoware_auto_msgs_DetectedObjectKinematics;
    enum class autoware_auto_msgs_WipersCommand_Constants : ::std::uint8_t;
    struct autoware_auto_msgs_HeadlightsCommand;
    struct autoware_auto_msgs_ClassifiedRoiArray;
  }
}

namespace zenoh_flow {
namespace autoware_auto {
#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Time
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Time
struct builtin_interfaces_Time final {
  ::std::int32_t sec;
  ::std::uint32_t nanosec;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Time

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HighLevelControlCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HighLevelControlCommand
struct autoware_auto_msgs_HighLevelControlCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  float velocity_mps;
  float curvature;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HighLevelControlCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport
struct autoware_auto_msgs_VehicleStateReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t fuel;
  ::std::uint8_t blinker;
  ::std::uint8_t headlight;
  ::std::uint8_t wiper;
  ::std::uint8_t gear;
  ::std::uint8_t mode;
  bool hand_brake;
  bool horn;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_StationaryLockingCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_StationaryLockingCommand
struct autoware_auto_msgs_StationaryLockingCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  bool avoid_stationary_locking;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_StationaryLockingCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleOdometry
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleOdometry
struct autoware_auto_msgs_VehicleOdometry final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  float velocity_mps;
  float front_wheel_angle_rad;
  float rear_wheel_angle_rad;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleOdometry

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point
struct geometry_msgs_Point final {
  double x;
  double y;
  double z;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$std_msgs_Header
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$std_msgs_Header
struct std_msgs_Header final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::rust::String frame_id;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$std_msgs_Header

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Duration
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Duration
struct builtin_interfaces_Duration final {
  ::std::int32_t sec;
  ::std::uint32_t nanosec;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$builtin_interfaces_Duration

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Complex32
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Complex32
struct autoware_auto_msgs_Complex32 final {
  float real;
  float imag;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Complex32

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrajectoryPoint
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrajectoryPoint
struct autoware_auto_msgs_TrajectoryPoint final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Duration time_from_start;
  float x;
  float y;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 heading;
  float longitudinal_velocity_mps;
  float lateral_velocity_mps;
  float acceleration_mps2;
  float heading_rate_rps;
  float front_wheel_angle_rad;
  float rear_wheel_angle_rad;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrajectoryPoint

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Vector3
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Vector3
struct geometry_msgs_Vector3 final {
  double x;
  double y;
  double z;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Vector3

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Quaternion
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Quaternion
struct geometry_msgs_Quaternion final {
  double x;
  double y;
  double z;
  double w;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Quaternion

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Transform
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Transform
struct geometry_msgs_Transform final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Vector3 translation;
  ::zenoh_flow::autoware_auto::geometry_msgs_Quaternion rotation;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Transform

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleKinematicState
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleKinematicState
struct autoware_auto_msgs_VehicleKinematicState final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint state;
  ::zenoh_flow::autoware_auto::geometry_msgs_Transform delta;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleKinematicState

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand_Constants
enum class autoware_auto_msgs_VehicleStateCommand_Constants : ::std::uint8_t {
  BLINKER_NO_COMMAND = 0,
  BLINKER_OFF = 1,
  BLINKER_LEFT = 2,
  BLINKER_RIGHT = 3,
  BLINKER_HAZARD = 4,
  HEADLIGHT_NO_COMMAND = 0,
  HEADLIGHT_OFF = 1,
  HEADLIGHT_ON = 2,
  HEADLIGHT_HIGH = 3,
  WIPER_NO_COMMAND = 0,
  WIPER_OFF = 1,
  WIPER_LOW = 2,
  WIPER_HIGH = 3,
  WIPER_CLEAN = 14,
  GEAR_NO_COMMAND = 0,
  GEAR_DRIVE = 1,
  GEAR_REVERSE = 2,
  GEAR_PARK = 3,
  GEAR_LOW = 4,
  GEAR_NEUTRAL = 5,
  MODE_NO_COMMAND = 0,
  MODE_AUTONOMOUS = 1,
  MODE_MANUAL = 2,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjects
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjects
struct autoware_auto_msgs_DetectedObjects final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_DetectedObject> objects;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjects

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport_Constants
enum class autoware_auto_msgs_VehicleStateReport_Constants : ::std::uint8_t {
  BLINKER_OFF = 1,
  BLINKER_LEFT = 2,
  BLINKER_RIGHT = 3,
  BLINKER_HAZARD = 4,
  HEADLIGHT_OFF = 1,
  HEADLIGHT_ON = 2,
  HEADLIGHT_HIGH = 3,
  WIPER_OFF = 1,
  WIPER_LOW = 2,
  WIPER_HIGH = 3,
  WIPER_CLEAN = 14,
  GEAR_DRIVE = 1,
  GEAR_REVERSE = 2,
  GEAR_PARK = 3,
  GEAR_LOW = 4,
  GEAR_NEUTRAL = 5,
  MODE_AUTONOMOUS = 1,
  MODE_MANUAL = 2,
  MODE_DISENGAGED = 3,
  MODE_NOT_READY = 4,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateReport_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Pose
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Pose
struct geometry_msgs_Pose final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Point position;
  ::zenoh_flow::autoware_auto::geometry_msgs_Quaternion orientation;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Pose

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovariance
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovariance
struct geometry_msgs_PoseWithCovariance final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Pose pose;
  ::std::array<double, 36> covariance;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovariance

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovarianceStamped
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovarianceStamped
struct geometry_msgs_PoseWithCovarianceStamped final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovariance pose;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_PoseWithCovarianceStamped

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory_Constants
enum class autoware_auto_msgs_Trajectory_Constants : ::std::uint8_t {
  CAPACITY = 100,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RoutePoint
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RoutePoint
struct autoware_auto_msgs_RoutePoint final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Point position;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_Complex32 heading;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RoutePoint

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapRoute
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapRoute
struct autoware_auto_msgs_HADMapRoute final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint start_point;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_RoutePoint goal_point;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_HADMapSegment> segments;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapRoute

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DiagnosticHeader
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DiagnosticHeader
struct autoware_auto_msgs_DiagnosticHeader final {
  ::rust::String name;
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time data_stamp;
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time computation_start;
  ::zenoh_flow::autoware_auto::builtin_interfaces_Duration runtime;
  ::std::uint32_t iterations;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DiagnosticHeader

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport
struct autoware_auto_msgs_HazardLightsReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Twist
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Twist
struct geometry_msgs_Twist final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Vector3 linear;
  ::zenoh_flow::autoware_auto::geometry_msgs_Vector3 angular;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Twist

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_TwistWithCovariance
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_TwistWithCovariance
struct geometry_msgs_TwistWithCovariance final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Twist twist;
  ::std::array<double, 36> covariance;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_TwistWithCovariance

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Accel
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Accel
struct geometry_msgs_Accel final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Vector3 linear;
  ::zenoh_flow::autoware_auto::geometry_msgs_Vector3 angular;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Accel

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_AccelWithCovariance
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_AccelWithCovariance
struct geometry_msgs_AccelWithCovariance final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Accel accel;
  ::std::array<double, 36> covariance;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_AccelWithCovariance

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics
struct autoware_auto_msgs_TrackedObjectKinematics final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Point centroid_position;
  ::std::array<double, 9> position_covariance;
  ::zenoh_flow::autoware_auto::geometry_msgs_Quaternion orientation;
  ::std::uint8_t orientation_availability;
  ::zenoh_flow::autoware_auto::geometry_msgs_TwistWithCovariance twist;
  ::zenoh_flow::autoware_auto::geometry_msgs_AccelWithCovariance acceleration;
  bool is_stationary;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand
struct autoware_auto_msgs_WipersCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t command;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RawControlCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RawControlCommand
struct autoware_auto_msgs_RawControlCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint32_t throttle;
  ::std::uint32_t brake;
  ::std::int32_t front_steer;
  ::std::int32_t rear_steer;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RawControlCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory
struct autoware_auto_msgs_Trajectory final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint> points;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Trajectory

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport
struct autoware_auto_msgs_GearReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport_Constants
enum class autoware_auto_msgs_TurnIndicatorsReport_Constants : ::std::uint8_t {
  DISABLE = 1,
  ENABLE_LEFT = 2,
  ENABLE_RIGHT = 3,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point32
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point32
struct geometry_msgs_Point32 final {
  float x;
  float y;
  float z;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Point32

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand_Constants
enum class autoware_auto_msgs_HazardLightsCommand_Constants : ::std::uint8_t {
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE = 2,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand_Constants

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport_Constants
enum class autoware_auto_msgs_WipersReport_Constants : ::std::uint8_t {
  DISABLE = 1,
  ENABLE_LOW = 2,
  ENABLE_HIGH = 3,
  ENABLE_INT_1 = 4,
  ENABLE_INT_2 = 5,
  ENABLE_INT_3 = 6,
  ENABLE_INT_4 = 7,
  ENABLE_INT_5 = 8,
  ENABLE_INT_6 = 9,
  ENABLE_INT_7 = 10,
  ENABLE_INT_8 = 11,
  ENABLE_INT_9 = 12,
  ENABLE_INT_10 = 13,
  ENABLE_CLEAN = 14,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannLateralCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannLateralCommand
struct autoware_auto_msgs_AckermannLateralCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  float steering_tire_angle;
  float steering_tire_rotation_rate;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannLateralCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RelativePositionWithCovarianceStamped
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RelativePositionWithCovarianceStamped
struct autoware_auto_msgs_RelativePositionWithCovarianceStamped final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::String child_frame_id;
  ::zenoh_flow::autoware_auto::geometry_msgs_Point position;
  ::std::array<double, 9> covariance;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_RelativePositionWithCovarianceStamped

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification
struct autoware_auto_msgs_ObjectClassification final {
  ::std::uint8_t classification;
  float probability;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport
struct autoware_auto_msgs_TurnIndicatorsReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsReport

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics_Constants
enum class autoware_auto_msgs_DetectedObjectKinematics_Constants : ::std::uint8_t {
  UNAVAILABLE = 0,
  SIGN_UNKNOWN = 1,
  AVAILABLE = 2,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeCommand
struct autoware_auto_msgs_HandBrakeCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  bool active;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF
struct autoware_auto_msgs_PointXYZIF final {
  float x;
  float y;
  float z;
  float intensity;
  ::std::uint16_t id;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand_Constants
enum class autoware_auto_msgs_TurnIndicatorsCommand_Constants : ::std::uint8_t {
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE_LEFT = 2,
  ENABLE_RIGHT = 3,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin
struct autoware_auto_msgs_HADMapBin final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::std::uint8_t map_format;
  ::rust::String format_version;
  ::rust::String map_version;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjectKinematics
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjectKinematics
struct autoware_auto_msgs_PredictedObjectKinematics final {
  ::zenoh_flow::autoware_auto::geometry_msgs_PoseWithCovariance initial_pose;
  ::zenoh_flow::autoware_auto::geometry_msgs_TwistWithCovariance initial_twist;
  ::zenoh_flow::autoware_auto::geometry_msgs_AccelWithCovariance initial_acceleration;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_PredictedPath> predicted_paths;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjectKinematics

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport_Constants
enum class autoware_auto_msgs_HazardLightsReport_Constants : ::std::uint8_t {
  DISABLE = 1,
  ENABLE = 2,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsReport_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand
struct autoware_auto_msgs_TurnIndicatorsCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t command;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TurnIndicatorsCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjects
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjects
struct autoware_auto_msgs_TrackedObjects final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_TrackedObject> objects;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjects

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_MapPrimitive
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_MapPrimitive
struct autoware_auto_msgs_MapPrimitive final {
  ::std::int64_t id;
  ::rust::String primitive_type;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_MapPrimitive

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeReport
struct autoware_auto_msgs_HandBrakeReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  bool report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HandBrakeReport

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornReport
struct autoware_auto_msgs_HornReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  bool report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornReport

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObject
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObject
struct autoware_auto_msgs_PredictedObject final {
  ::std::uint64_t object_id;
  float existence_probability;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_ObjectClassification> classification;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_PredictedObjectKinematics kinematics;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_Shape> shape;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObject

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Polygon
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Polygon
struct geometry_msgs_Polygon final {
  ::rust::Vec<::zenoh_flow::autoware_auto::geometry_msgs_Point32> points;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$geometry_msgs_Polygon

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedPath
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedPath
struct autoware_auto_msgs_PredictedPath final {
  ::rust::Vec<::zenoh_flow::autoware_auto::geometry_msgs_Pose> path;
  ::zenoh_flow::autoware_auto::builtin_interfaces_Duration time_step;
  float confidence;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedPath

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Quaternion32
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Quaternion32
struct autoware_auto_msgs_Quaternion32 final {
  float x;
  float y;
  float z;
  float w;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Quaternion32

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointClusters
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointClusters
struct autoware_auto_msgs_PointClusters final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_PointXYZIF> points;
  ::rust::Vec<::std::uint32_t> cluster_boundary;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PointClusters

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoi
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoi
struct autoware_auto_msgs_ClassifiedRoi final {
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_ObjectClassification> classifications;
  ::zenoh_flow::autoware_auto::geometry_msgs_Polygon polygon;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoi

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand
struct autoware_auto_msgs_HazardLightsCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t command;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HazardLightsCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Shape
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Shape
struct autoware_auto_msgs_Shape final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Polygon polygon;
  float height;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Shape

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Route
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Route
struct autoware_auto_msgs_Route final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint start_point;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_TrajectoryPoint goal_point;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_MapPrimitive> primitives;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_Route

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport
struct autoware_auto_msgs_HeadlightsReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_LongitudinalCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_LongitudinalCommand
struct autoware_auto_msgs_LongitudinalCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  float speed;
  float acceleration;
  float jerk;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_LongitudinalCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannControlCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannControlCommand
struct autoware_auto_msgs_AckermannControlCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_AckermannLateralCommand lateral;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_LongitudinalCommand longitudinal;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_AckermannControlCommand

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification_Constants
enum class autoware_auto_msgs_ObjectClassification_Constants : ::std::uint8_t {
  UNKNOWN = 0,
  CAR = 1,
  TRUCK = 2,
  TRAILER = 3,
  MOTORCYCLE = 4,
  BICYCLE = 5,
  PEDESTRIAN = 6,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_ObjectClassification_Constants

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Route_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Route_Constants
enum class autoware_auto_msgs_Route_Constants : ::std::uint8_t {
  CAPACITY = 100,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_Route_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapSegment
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapSegment
struct autoware_auto_msgs_HADMapSegment final {
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_MapPrimitive> primitives;
  ::std::int64_t preferred_primitive_id;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapSegment

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ControlDiagnostic
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ControlDiagnostic
struct autoware_auto_msgs_ControlDiagnostic final {
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_DiagnosticHeader diag_header;
  bool new_trajectory;
  ::rust::String trajectory_source;
  ::rust::String pose_source;
  float lateral_error_m;
  float longitudinal_error_m;
  float velocity_error_mps;
  float acceleration_error_mps2;
  float yaw_error_rad;
  float yaw_rate_error_rps;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ControlDiagnostic

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray
struct autoware_auto_msgs_BoundingBoxArray final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_BoundingBox> boxes;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox_Constants
enum class autoware_auto_msgs_BoundingBox_Constants : ::std::uint8_t {
  NO_LABEL = 0,
  CAR = 1,
  PEDESTRIAN = 2,
  CYCLIST = 3,
  MOTORCYCLE = 4,
  NO_SIGNAL = 0,
  LEFT_SIGNAL = 1,
  RIGHT_SIGNAL = 2,
  BRAKE = 3,
  POSE_X = 0,
  POSE_Y = 1,
  VELOCITY = 2,
  HEADING = 3,
  TURN_RATE = 4,
  SIZE_X = 5,
  SIZE_Y = 6,
  ACCELERATION = 7,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox_Constants

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin_Constants
enum class autoware_auto_msgs_HADMapBin_Constants : ::std::uint8_t {
  MAP_FORMAT_LANELET2 = 0,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HADMapBin_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand
struct autoware_auto_msgs_VehicleStateCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t blinker;
  ::std::uint8_t headlight;
  ::std::uint8_t wiper;
  ::std::uint8_t gear;
  ::std::uint8_t mode;
  bool hand_brake;
  bool horn;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleStateCommand

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport_Constants
enum class autoware_auto_msgs_HeadlightsReport_Constants : ::std::uint8_t {
  DISABLE = 1,
  ENABLE_LOW = 2,
  ENABLE_HIGH = 3,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsReport_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics
struct autoware_auto_msgs_DetectedObjectKinematics final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Point centroid_position;
  ::std::array<double, 9> position_covariance;
  bool has_position_covariance;
  ::zenoh_flow::autoware_auto::geometry_msgs_Quaternion orientation;
  ::std::uint8_t orientation_availability;
  ::zenoh_flow::autoware_auto::geometry_msgs_TwistWithCovariance twist;
  bool has_twist;
  bool has_twist_covariance;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObjectKinematics

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObject
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObject
struct autoware_auto_msgs_DetectedObject final {
  float existence_probability;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_ObjectClassification> classification;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_DetectedObjectKinematics kinematics;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_Shape shape;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_DetectedObject

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand_Constants
enum class autoware_auto_msgs_HeadlightsCommand_Constants : ::std::uint8_t {
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE_LOW = 2,
  ENABLE_HIGH = 3,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WheelEncoder
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WheelEncoder
struct autoware_auto_msgs_WheelEncoder final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  float speed_mps;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WheelEncoder

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornCommand
struct autoware_auto_msgs_HornCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  bool active;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HornCommand

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics_Constants
enum class autoware_auto_msgs_TrackedObjectKinematics_Constants : ::std::uint8_t {
  UNAVAILABLE = 0,
  SIGN_UNKNOWN = 1,
  AVAILABLE = 2,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObjectKinematics_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleControlCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleControlCommand
struct autoware_auto_msgs_VehicleControlCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  float long_accel_mps2;
  float velocity_mps;
  float front_wheel_angle_rad;
  float rear_wheel_angle_rad;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_VehicleControlCommand

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray_Constants
enum class autoware_auto_msgs_BoundingBoxArray_Constants : ::std::uint16_t {
  CAPACITY = 256,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBoxArray_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObject
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObject
struct autoware_auto_msgs_TrackedObject final {
  ::std::uint64_t object_id;
  float existence_probability;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_ObjectClassification> classification;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_TrackedObjectKinematics kinematics;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_Shape> shape;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_TrackedObject

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjects
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjects
struct autoware_auto_msgs_PredictedObjects final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_PredictedObject> objects;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_PredictedObjects

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox
struct autoware_auto_msgs_BoundingBox final {
  ::zenoh_flow::autoware_auto::geometry_msgs_Point32 centroid;
  ::zenoh_flow::autoware_auto::geometry_msgs_Point32 size;
  ::zenoh_flow::autoware_auto::autoware_auto_msgs_Quaternion32 orientation;
  float velocity;
  float heading;
  float heading_rate;
  ::std::array<::zenoh_flow::autoware_auto::geometry_msgs_Point32, 4> corners;
  ::std::array<float, 8> variance;
  float value;
  ::std::uint8_t vehicle_label;
  ::std::uint8_t signal_label;
  float class_likelihood;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_BoundingBox

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport_Constants
enum class autoware_auto_msgs_GearReport_Constants : ::std::uint8_t {
  DRIVE = 1,
  DRIVE_2 = 2,
  DRIVE_3 = 3,
  DRIVE_4 = 4,
  DRIVE_5 = 5,
  DRIVE_6 = 6,
  DRIVE_7 = 7,
  DRIVE_8 = 8,
  DRIVE_9 = 9,
  DRIVE_10 = 10,
  DRIVE_11 = 11,
  DRIVE_12 = 12,
  DRIVE_13 = 13,
  DRIVE_14 = 14,
  DRIVE_15 = 15,
  DRIVE_16 = 16,
  DRIVE_17 = 17,
  DRIVE_18 = 18,
  REVERSE = 19,
  REVERSE_2 = 20,
  PARK = 21,
  LOW = 22,
  LOW_2 = 23,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_GearReport_Constants

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF_Constants
enum class autoware_auto_msgs_PointXYZIF_Constants : ::std::uint16_t {
  END_OF_SCAN_ID = 65535,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_PointXYZIF_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport
struct autoware_auto_msgs_WipersReport final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t report;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersReport

#ifndef CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand_Constants
#define CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand_Constants
enum class autoware_auto_msgs_WipersCommand_Constants : ::std::uint8_t {
  NO_COMMAND = 0,
  DISABLE = 1,
  ENABLE_LOW = 2,
  ENABLE_HIGH = 3,
  ENABLE_INT_1 = 4,
  ENABLE_INT_2 = 5,
  ENABLE_INT_3 = 6,
  ENABLE_INT_4 = 7,
  ENABLE_INT_5 = 8,
  ENABLE_INT_6 = 9,
  ENABLE_INT_7 = 10,
  ENABLE_INT_8 = 11,
  ENABLE_INT_9 = 12,
  ENABLE_INT_10 = 13,
  ENABLE_CLEAN = 14,
};
#endif // CXXBRIDGE1_ENUM_zenoh_flow$autoware_auto$autoware_auto_msgs_WipersCommand_Constants

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand
struct autoware_auto_msgs_HeadlightsCommand final {
  ::zenoh_flow::autoware_auto::builtin_interfaces_Time stamp;
  ::std::uint8_t command;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_HeadlightsCommand

#ifndef CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoiArray
#define CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoiArray
struct autoware_auto_msgs_ClassifiedRoiArray final {
  ::zenoh_flow::autoware_auto::std_msgs_Header header;
  ::rust::Vec<::zenoh_flow::autoware_auto::autoware_auto_msgs_ClassifiedRoi> rois;

  using IsRelocatable = ::std::true_type;
};
#endif // CXXBRIDGE1_STRUCT_zenoh_flow$autoware_auto$autoware_auto_msgs_ClassifiedRoiArray
} // namespace autoware_auto
} // namespace zenoh_flow
