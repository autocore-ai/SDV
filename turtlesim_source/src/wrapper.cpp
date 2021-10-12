#include "source.hpp"
#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <initializer_list>
#include <iterator>
#include <memory>
#include <new>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace rust
{
  inline namespace cxxbridge1
  {
    // #include "rust/cxx.h"

#ifndef CXXBRIDGE1_PANIC
#define CXXBRIDGE1_PANIC
    template <typename Exception>
    void panic [[noreturn]] (const char *msg);
#endif // CXXBRIDGE1_PANIC

    struct unsafe_bitcopy_t;

    namespace
    {
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
    class String final
    {
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
    namespace detail
    {
      template <bool>
      struct copy_assignable_if
      {
      };

      template <>
      struct copy_assignable_if<false>
      {
        copy_assignable_if() noexcept = default;
        copy_assignable_if(const copy_assignable_if &) noexcept = default;
        copy_assignable_if &operator=(const copy_assignable_if &) &noexcept = delete;
        copy_assignable_if &operator=(copy_assignable_if &&) &noexcept = default;
      };
    } // namespace detail

    template <typename T>
    class Slice final
        : private detail::copy_assignable_if<std::is_const<T>::value>
    {
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
    class Slice<T>::iterator final
    {
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
    Slice<T>::Slice() noexcept
    {
      sliceInit(this, reinterpret_cast<void *>(align_of<T>()), 0);
    }

    template <typename T>
    Slice<T>::Slice(T *s, std::size_t count) noexcept
    {
      assert(s != nullptr || count == 0);
      sliceInit(this,
                s == nullptr && count == 0
                    ? reinterpret_cast<void *>(align_of<T>())
                    : const_cast<typename std::remove_const<T>::type *>(s),
                count);
    }

    template <typename T>
    T *Slice<T>::data() const noexcept
    {
      return reinterpret_cast<T *>(slicePtr(this));
    }

    template <typename T>
    std::size_t Slice<T>::size() const noexcept
    {
      return sliceLen(this);
    }

    template <typename T>
    std::size_t Slice<T>::length() const noexcept
    {
      return this->size();
    }

    template <typename T>
    bool Slice<T>::empty() const noexcept
    {
      return this->size() == 0;
    }

    template <typename T>
    T &Slice<T>::operator[](std::size_t n) const noexcept
    {
      assert(n < this->size());
      auto ptr = static_cast<char *>(slicePtr(this)) + size_of<T>() * n;
      return *reinterpret_cast<T *>(ptr);
    }

    template <typename T>
    T &Slice<T>::at(std::size_t n) const
    {
      if (n >= this->size())
      {
        panic<std::out_of_range>("rust::Slice index out of range");
      }
      return (*this)[n];
    }

    template <typename T>
    T &Slice<T>::front() const noexcept
    {
      assert(!this->empty());
      return (*this)[0];
    }

    template <typename T>
    T &Slice<T>::back() const noexcept
    {
      assert(!this->empty());
      return (*this)[this->size() - 1];
    }

    template <typename T>
    typename Slice<T>::iterator::reference
    Slice<T>::iterator::operator*() const noexcept
    {
      return *static_cast<T *>(this->pos);
    }

    template <typename T>
    typename Slice<T>::iterator::pointer
    Slice<T>::iterator::operator->() const noexcept
    {
      return static_cast<T *>(this->pos);
    }

    template <typename T>
    typename Slice<T>::iterator::reference Slice<T>::iterator::operator[](
        typename Slice<T>::iterator::difference_type n) const noexcept
    {
      auto ptr = static_cast<char *>(this->pos) + this->stride * n;
      return *reinterpret_cast<T *>(ptr);
    }

    template <typename T>
    typename Slice<T>::iterator &Slice<T>::iterator::operator++() noexcept
    {
      this->pos = static_cast<char *>(this->pos) + this->stride;
      return *this;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::iterator::operator++(int) noexcept
    {
      auto ret = iterator(*this);
      this->pos = static_cast<char *>(this->pos) + this->stride;
      return ret;
    }

    template <typename T>
    typename Slice<T>::iterator &Slice<T>::iterator::operator--() noexcept
    {
      this->pos = static_cast<char *>(this->pos) - this->stride;
      return *this;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::iterator::operator--(int) noexcept
    {
      auto ret = iterator(*this);
      this->pos = static_cast<char *>(this->pos) - this->stride;
      return ret;
    }

    template <typename T>
    typename Slice<T>::iterator &Slice<T>::iterator::operator+=(
        typename Slice<T>::iterator::difference_type n) noexcept
    {
      this->pos = static_cast<char *>(this->pos) + this->stride * n;
      return *this;
    }

    template <typename T>
    typename Slice<T>::iterator &Slice<T>::iterator::operator-=(
        typename Slice<T>::iterator::difference_type n) noexcept
    {
      this->pos = static_cast<char *>(this->pos) - this->stride * n;
      return *this;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::iterator::operator+(
        typename Slice<T>::iterator::difference_type n) const noexcept
    {
      auto ret = iterator(*this);
      ret.pos = static_cast<char *>(this->pos) + this->stride * n;
      return ret;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::iterator::operator-(
        typename Slice<T>::iterator::difference_type n) const noexcept
    {
      auto ret = iterator(*this);
      ret.pos = static_cast<char *>(this->pos) - this->stride * n;
      return ret;
    }

    template <typename T>
    typename Slice<T>::iterator::difference_type
    Slice<T>::iterator::operator-(const iterator &other) const noexcept
    {
      auto diff = std::distance(static_cast<char *>(other.pos),
                                static_cast<char *>(this->pos));
      return diff / this->stride;
    }

    template <typename T>
    bool Slice<T>::iterator::operator==(const iterator &other) const noexcept
    {
      return this->pos == other.pos;
    }

    template <typename T>
    bool Slice<T>::iterator::operator!=(const iterator &other) const noexcept
    {
      return this->pos != other.pos;
    }

    template <typename T>
    bool Slice<T>::iterator::operator<(const iterator &other) const noexcept
    {
      return this->pos < other.pos;
    }

    template <typename T>
    bool Slice<T>::iterator::operator<=(const iterator &other) const noexcept
    {
      return this->pos <= other.pos;
    }

    template <typename T>
    bool Slice<T>::iterator::operator>(const iterator &other) const noexcept
    {
      return this->pos > other.pos;
    }

    template <typename T>
    bool Slice<T>::iterator::operator>=(const iterator &other) const noexcept
    {
      return this->pos >= other.pos;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::begin() const noexcept
    {
      iterator it;
      it.pos = slicePtr(this);
      it.stride = size_of<T>();
      return it;
    }

    template <typename T>
    typename Slice<T>::iterator Slice<T>::end() const noexcept
    {
      iterator it = this->begin();
      it.pos = static_cast<char *>(it.pos) + it.stride * this->size();
      return it;
    }

    template <typename T>
    void Slice<T>::swap(Slice &rhs) noexcept
    {
      std::swap(*this, rhs);
    }
#endif // CXXBRIDGE1_RUST_SLICE

#ifndef CXXBRIDGE1_RUST_BITCOPY_T
#define CXXBRIDGE1_RUST_BITCOPY_T
    struct unsafe_bitcopy_t final
    {
      explicit unsafe_bitcopy_t() = default;
    };
#endif // CXXBRIDGE1_RUST_BITCOPY_T

#ifndef CXXBRIDGE1_RUST_VEC
#define CXXBRIDGE1_RUST_VEC
    template <typename T>
    class Vec final
    {
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
    Vec<T>::Vec(std::initializer_list<T> init) : Vec{}
    {
      this->reserve_total(init.size());
      std::move(init.begin(), init.end(), std::back_inserter(*this));
    }

    template <typename T>
    Vec<T>::Vec(const Vec &other) : Vec()
    {
      this->reserve_total(other.size());
      std::copy(other.begin(), other.end(), std::back_inserter(*this));
    }

    template <typename T>
    Vec<T>::Vec(Vec &&other) noexcept : repr(other.repr)
    {
      new (&other) Vec();
    }

    template <typename T>
    Vec<T>::~Vec() noexcept
    {
      this->drop();
    }

    template <typename T>
    Vec<T> &Vec<T>::operator=(Vec &&other) &noexcept
    {
      this->drop();
      this->repr = other.repr;
      new (&other) Vec();
      return *this;
    }

    template <typename T>
    Vec<T> &Vec<T>::operator=(const Vec &other) &
    {
      if (this != &other)
      {
        this->drop();
        new (this) Vec(other);
      }
      return *this;
    }

    template <typename T>
    bool Vec<T>::empty() const noexcept
    {
      return this->size() == 0;
    }

    template <typename T>
    T *Vec<T>::data() noexcept
    {
      return const_cast<T *>(const_cast<const Vec<T> *>(this)->data());
    }

    template <typename T>
    const T &Vec<T>::operator[](std::size_t n) const noexcept
    {
      assert(n < this->size());
      auto data = reinterpret_cast<const char *>(this->data());
      return *reinterpret_cast<const T *>(data + n * size_of<T>());
    }

    template <typename T>
    const T &Vec<T>::at(std::size_t n) const
    {
      if (n >= this->size())
      {
        panic<std::out_of_range>("rust::Vec index out of range");
      }
      return (*this)[n];
    }

    template <typename T>
    const T &Vec<T>::front() const noexcept
    {
      assert(!this->empty());
      return (*this)[0];
    }

    template <typename T>
    const T &Vec<T>::back() const noexcept
    {
      assert(!this->empty());
      return (*this)[this->size() - 1];
    }

    template <typename T>
    T &Vec<T>::operator[](std::size_t n) noexcept
    {
      assert(n < this->size());
      auto data = reinterpret_cast<char *>(this->data());
      return *reinterpret_cast<T *>(data + n * size_of<T>());
    }

    template <typename T>
    T &Vec<T>::at(std::size_t n)
    {
      if (n >= this->size())
      {
        panic<std::out_of_range>("rust::Vec index out of range");
      }
      return (*this)[n];
    }

    template <typename T>
    T &Vec<T>::front() noexcept
    {
      assert(!this->empty());
      return (*this)[0];
    }

    template <typename T>
    T &Vec<T>::back() noexcept
    {
      assert(!this->empty());
      return (*this)[this->size() - 1];
    }

    template <typename T>
    void Vec<T>::reserve(std::size_t new_cap)
    {
      this->reserve_total(new_cap);
    }

    template <typename T>
    void Vec<T>::push_back(const T &value)
    {
      this->emplace_back(value);
    }

    template <typename T>
    void Vec<T>::push_back(T &&value)
    {
      this->emplace_back(std::move(value));
    }

    template <typename T>
    template <typename... Args>
    void Vec<T>::emplace_back(Args &&...args)
    {
      auto size = this->size();
      this->reserve_total(size + 1);
      ::new (reinterpret_cast<T *>(reinterpret_cast<char *>(this->data()) +
                                   size * size_of<T>()))
          T(std::forward<Args>(args)...);
      this->set_len(size + 1);
    }

    template <typename T>
    typename Vec<T>::iterator Vec<T>::begin() noexcept
    {
      return Slice<T>(this->data(), this->size()).begin();
    }

    template <typename T>
    typename Vec<T>::iterator Vec<T>::end() noexcept
    {
      return Slice<T>(this->data(), this->size()).end();
    }

    template <typename T>
    typename Vec<T>::const_iterator Vec<T>::begin() const noexcept
    {
      return this->cbegin();
    }

    template <typename T>
    typename Vec<T>::const_iterator Vec<T>::end() const noexcept
    {
      return this->cend();
    }

    template <typename T>
    typename Vec<T>::const_iterator Vec<T>::cbegin() const noexcept
    {
      return Slice<const T>(this->data(), this->size()).begin();
    }

    template <typename T>
    typename Vec<T>::const_iterator Vec<T>::cend() const noexcept
    {
      return Slice<const T>(this->data(), this->size()).end();
    }

    template <typename T>
    void Vec<T>::swap(Vec &rhs) noexcept
    {
      using std::swap;
      swap(this->repr, rhs.repr);
    }

    template <typename T>
    Vec<T>::Vec(unsafe_bitcopy_t, const Vec &bits) noexcept : repr(bits.repr) {}
#endif // CXXBRIDGE1_RUST_VEC

#ifndef CXXBRIDGE1_IS_COMPLETE
#define CXXBRIDGE1_IS_COMPLETE
    namespace detail
    {
      namespace
      {
        template <typename T, typename = std::size_t>
        struct is_complete : std::false_type
        {
        };
        template <typename T>
        struct is_complete<T, decltype(sizeof(T))> : std::true_type
        {
        };
      } // namespace
    }   // namespace detail
#endif  // CXXBRIDGE1_IS_COMPLETE

#ifndef CXXBRIDGE1_LAYOUT
#define CXXBRIDGE1_LAYOUT
    class layout
    {
      template <typename T>
      friend std::size_t size_of();
      template <typename T>
      friend std::size_t align_of();
      template <typename T>
      static typename std::enable_if<std::is_base_of<Opaque, T>::value,
                                     std::size_t>::type
      do_size_of()
      {
        return T::layout::size();
      }
      template <typename T>
      static typename std::enable_if<!std::is_base_of<Opaque, T>::value,
                                     std::size_t>::type
      do_size_of()
      {
        return sizeof(T);
      }
      template <typename T>
      static
          typename std::enable_if<detail::is_complete<T>::value, std::size_t>::type
          size_of()
      {
        return do_size_of<T>();
      }
      template <typename T>
      static typename std::enable_if<std::is_base_of<Opaque, T>::value,
                                     std::size_t>::type
      do_align_of()
      {
        return T::layout::align();
      }
      template <typename T>
      static typename std::enable_if<!std::is_base_of<Opaque, T>::value,
                                     std::size_t>::type
      do_align_of()
      {
        return alignof(T);
      }
      template <typename T>
      static
          typename std::enable_if<detail::is_complete<T>::value, std::size_t>::type
          align_of()
      {
        return do_align_of<T>();
      }
    };

    template <typename T>
    std::size_t size_of()
    {
      return layout::size_of<T>();
    }

    template <typename T>
    std::size_t align_of()
    {
      return layout::align_of<T>();
    }
#endif // CXXBRIDGE1_LAYOUT

    namespace
    {
      namespace repr
      {
        struct PtrLen final
        {
          void *ptr;
          ::std::size_t len;
        };
      } // namespace repr

      template <bool>
      struct deleter_if
      {
        template <typename T>
        void operator()(T *) {}
      };

      template <>
      struct deleter_if<true>
      {
        template <typename T>
        void operator()(T *ptr) { ptr->~T(); }
      };
    } // namespace
  }   // namespace cxxbridge1

  namespace behavior
  {
    class missing
    {
    };
    missing trycatch(...);

    template <typename Try, typename Fail>
    static typename ::std::enable_if<
        ::std::is_same<decltype(trycatch(::std::declval<Try>(), ::std::declval<Fail>())),
                       missing>::value>::type
    trycatch(Try &&func, Fail &&fail) noexcept
    try
    {
      func();
    }
    catch (const ::std::exception &e)
    {
      fail(e.what());
    }
  } // namespace behavior
} // namespace rust

extern "C"
{
  const char *cxxbridge1$exception(const char *, ::std::size_t);
} // extern "C"

namespace zenoh
{
  namespace flow
  {
    struct Context;
    enum class TokenStatus : ::std::uint8_t;
    enum class TokenAction : ::std::uint8_t;
    struct Token;
    struct Input;
    struct Output;
    struct Data;
    struct Configuration;
    struct ConfigurationMap;
    using State = ::zenoh::flow::State;
  }
}

namespace zenoh
{
  namespace flow
  {
#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Context
#define CXXBRIDGE1_STRUCT_zenoh$flow$Context
    struct Context final
    {
      ::std::size_t mode;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Context

#ifndef CXXBRIDGE1_ENUM_zenoh$flow$TokenStatus
#define CXXBRIDGE1_ENUM_zenoh$flow$TokenStatus
    enum class TokenStatus : ::std::uint8_t
    {
      Pending = 0,
      Ready = 1,
      DeadlineMiss = 2,
    };
#endif // CXXBRIDGE1_ENUM_zenoh$flow$TokenStatus

#ifndef CXXBRIDGE1_ENUM_zenoh$flow$TokenAction
#define CXXBRIDGE1_ENUM_zenoh$flow$TokenAction
    enum class TokenAction : ::std::uint8_t
    {
      Consume = 0,
      Drop = 1,
      Keep = 2,
      Postpone = 3,
      Wait = 4,
    };
#endif // CXXBRIDGE1_ENUM_zenoh$flow$TokenAction

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Token
#define CXXBRIDGE1_STRUCT_zenoh$flow$Token
    struct Token final
    {
      ::zenoh::flow::TokenStatus status;
      ::zenoh::flow::TokenAction action;
      ::rust::String port_id;
      ::rust::Vec<::std::uint8_t> data;
      ::std::uint64_t timestamp;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Token

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Input
#define CXXBRIDGE1_STRUCT_zenoh$flow$Input
    struct Input final
    {
      ::rust::String port_id;
      ::rust::Vec<::std::uint8_t> data;
      ::std::uint64_t timestamp;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Input

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Output
#define CXXBRIDGE1_STRUCT_zenoh$flow$Output
    struct Output final
    {
      ::rust::String port_id;
      ::rust::Vec<::std::uint8_t> data;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Output

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Data
#define CXXBRIDGE1_STRUCT_zenoh$flow$Data
    struct Data final
    {
      ::rust::Vec<::std::uint8_t> bytes;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Data

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$Configuration
#define CXXBRIDGE1_STRUCT_zenoh$flow$Configuration
    struct Configuration final
    {
      ::rust::String key;
      ::rust::String value;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$Configuration

#ifndef CXXBRIDGE1_STRUCT_zenoh$flow$ConfigurationMap
#define CXXBRIDGE1_STRUCT_zenoh$flow$ConfigurationMap
    struct ConfigurationMap final
    {
      ::rust::Vec<::zenoh::flow::Configuration> map;

      using IsRelocatable = ::std::true_type;
    };
#endif // CXXBRIDGE1_STRUCT_zenoh$flow$ConfigurationMap

    extern "C"
    {
      ::zenoh::flow::State *zenoh$flow$cxxbridge1$initialize(const ::zenoh::flow::ConfigurationMap &configuration) noexcept
      {
        ::std::unique_ptr<::zenoh::flow::State> (*initialize$)(const ::zenoh::flow::ConfigurationMap &) = ::zenoh::flow::initialize;
        return initialize$(configuration).release();
      }

      ::rust::repr::PtrLen zenoh$flow$cxxbridge1$run(::zenoh::flow::Context &context, ::std::unique_ptr<::zenoh::flow::State> &state, ::rust::Vec<::zenoh::flow::Output> *return$) noexcept
      {
        ::rust::Vec<::zenoh::flow::Output> (*run$)(::zenoh::flow::Context &, ::std::unique_ptr<::zenoh::flow::State> &) = ::zenoh::flow::run;
        ::rust::repr::PtrLen throw$;
        ::rust::behavior::trycatch(
            [&]
            {
              new (return$)::rust::Vec<::zenoh::flow::Output>(run$(context, state));
              throw$.ptr = nullptr;
            },
            [&](const char *catch$) noexcept
            {
              throw$.len = ::std::strlen(catch$);
              throw$.ptr = const_cast<char *>(::cxxbridge1$exception(catch$, throw$.len));
            });
        return throw$;
      }
    } // extern "C"
  }   // namespace flow
} // namespace zenoh

extern "C"
{
  void cxxbridge1$rust_vec$zenoh$flow$Configuration$new(const ::rust::Vec<::zenoh::flow::Configuration> *ptr) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Configuration$drop(::rust::Vec<::zenoh::flow::Configuration> *ptr) noexcept;
  ::std::size_t cxxbridge1$rust_vec$zenoh$flow$Configuration$len(const ::rust::Vec<::zenoh::flow::Configuration> *ptr) noexcept;
  ::std::size_t cxxbridge1$rust_vec$zenoh$flow$Configuration$capacity(const ::rust::Vec<::zenoh::flow::Configuration> *ptr) noexcept;
  const ::zenoh::flow::Configuration *cxxbridge1$rust_vec$zenoh$flow$Configuration$data(const ::rust::Vec<::zenoh::flow::Configuration> *ptr) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Configuration$reserve_total(::rust::Vec<::zenoh::flow::Configuration> *ptr, ::std::size_t new_cap) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Configuration$set_len(::rust::Vec<::zenoh::flow::Configuration> *ptr, ::std::size_t len) noexcept;

  static_assert(::rust::detail::is_complete<::zenoh::flow::State>::value, "definition of State is required");
  static_assert(sizeof(::std::unique_ptr<::zenoh::flow::State>) == sizeof(void *), "");
  static_assert(alignof(::std::unique_ptr<::zenoh::flow::State>) == alignof(void *), "");
  void cxxbridge1$unique_ptr$zenoh$flow$State$null(::std::unique_ptr<::zenoh::flow::State> *ptr) noexcept
  {
    ::new (ptr)::std::unique_ptr<::zenoh::flow::State>();
  }
  void cxxbridge1$unique_ptr$zenoh$flow$State$raw(::std::unique_ptr<::zenoh::flow::State> *ptr, ::zenoh::flow::State *raw) noexcept
  {
    ::new (ptr)::std::unique_ptr<::zenoh::flow::State>(raw);
  }
  const ::zenoh::flow::State *cxxbridge1$unique_ptr$zenoh$flow$State$get(const ::std::unique_ptr<::zenoh::flow::State> &ptr) noexcept
  {
    return ptr.get();
  }
  ::zenoh::flow::State *cxxbridge1$unique_ptr$zenoh$flow$State$release(::std::unique_ptr<::zenoh::flow::State> &ptr) noexcept
  {
    return ptr.release();
  }
  void cxxbridge1$unique_ptr$zenoh$flow$State$drop(::std::unique_ptr<::zenoh::flow::State> *ptr) noexcept
  {
    ::rust::deleter_if<::rust::detail::is_complete<::zenoh::flow::State>::value>{}(ptr);
  }

  void cxxbridge1$rust_vec$zenoh$flow$Output$new(const ::rust::Vec<::zenoh::flow::Output> *ptr) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Output$drop(::rust::Vec<::zenoh::flow::Output> *ptr) noexcept;
  ::std::size_t cxxbridge1$rust_vec$zenoh$flow$Output$len(const ::rust::Vec<::zenoh::flow::Output> *ptr) noexcept;
  ::std::size_t cxxbridge1$rust_vec$zenoh$flow$Output$capacity(const ::rust::Vec<::zenoh::flow::Output> *ptr) noexcept;
  const ::zenoh::flow::Output *cxxbridge1$rust_vec$zenoh$flow$Output$data(const ::rust::Vec<::zenoh::flow::Output> *ptr) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Output$reserve_total(::rust::Vec<::zenoh::flow::Output> *ptr, ::std::size_t new_cap) noexcept;
  void cxxbridge1$rust_vec$zenoh$flow$Output$set_len(::rust::Vec<::zenoh::flow::Output> *ptr, ::std::size_t len) noexcept;
} // extern "C"

namespace rust
{
  inline namespace cxxbridge1
  {
    template <>
    Vec<::zenoh::flow::Configuration>::Vec() noexcept
    {
      cxxbridge1$rust_vec$zenoh$flow$Configuration$new(this);
    }
    template <>
    void Vec<::zenoh::flow::Configuration>::drop() noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$drop(this);
    }
    template <>
    ::std::size_t Vec<::zenoh::flow::Configuration>::size() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$len(this);
    }
    template <>
    ::std::size_t Vec<::zenoh::flow::Configuration>::capacity() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$capacity(this);
    }
    template <>
    const ::zenoh::flow::Configuration *Vec<::zenoh::flow::Configuration>::data() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$data(this);
    }
    template <>
    void Vec<::zenoh::flow::Configuration>::reserve_total(::std::size_t new_cap) noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$reserve_total(this, new_cap);
    }
    template <>
    void Vec<::zenoh::flow::Configuration>::set_len(::std::size_t len) noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Configuration$set_len(this, len);
    }
    template <>
    Vec<::zenoh::flow::Output>::Vec() noexcept
    {
      cxxbridge1$rust_vec$zenoh$flow$Output$new(this);
    }
    template <>
    void Vec<::zenoh::flow::Output>::drop() noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$drop(this);
    }
    template <>
    ::std::size_t Vec<::zenoh::flow::Output>::size() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$len(this);
    }
    template <>
    ::std::size_t Vec<::zenoh::flow::Output>::capacity() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$capacity(this);
    }
    template <>
    const ::zenoh::flow::Output *Vec<::zenoh::flow::Output>::data() const noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$data(this);
    }
    template <>
    void Vec<::zenoh::flow::Output>::reserve_total(::std::size_t new_cap) noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$reserve_total(this, new_cap);
    }
    template <>
    void Vec<::zenoh::flow::Output>::set_len(::std::size_t len) noexcept
    {
      return cxxbridge1$rust_vec$zenoh$flow$Output$set_len(this, len);
    }
  } // namespace cxxbridge1
} // namespace rust
