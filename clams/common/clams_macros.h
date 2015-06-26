#pragma once
#pragma warning(disable : 4005) // macro redefinition
#pragma warning(disable : 4251) // needs to have dll-interface for class members
#define FWDECL_STRUCT_SHARED_PTR(T)                                            \
                                                                               \
  struct T;                                                                    \
                                                                               \
  typedef std::shared_ptr<T> T##Ptr;

#define FWDECL_CLASS_SHARED_PTR(T)                                             \
                                                                               \
  class T;                                                                     \
                                                                               \
  typedef std::shared_ptr<T> T##Ptr;

#define DECLARE_WEAK_PTR(T)                                                    \
                                                                               \
  struct T;                                                                    \
                                                                               \
  typedef std::weak_ptr<T> T##Ptr;

#define FWDECL_STRUCT_SHARED_PTR_AND_VECTOR(T)                                 \
                                                                               \
  struct T;                                                                    \
                                                                               \
  typedef std::shared_ptr<T> T##Ptr;                                           \
                                                                               \
  typedef std::vector<T##Ptr> T##PtrVector;

#define CLAMS_INSTANTIATE_SERIALIZATION_T(T)                                   \
  template void T::serialize(boost::archive::binary_iarchive &,                \
                             const unsigned int);                              \
  template void T::serialize(boost::archive::binary_oarchive &,                \
                             const unsigned int);                              \
                                                                               \
  template void T::serialize(boost::archive::text_iarchive &,                  \
                             const unsigned int);                              \
  template void T::serialize(boost::archive::text_oarchive &,                  \
                             const unsigned int);

#if defined(SM_SHARED_LIBS) && defined(WIN32)
#define CLAMS_INSTANTIATE_SERIALIZATION(EXTERN, API, T)                        \
  EXTERN template API void T::serialize(boost::archive::binary_iarchive &,     \
                                        const unsigned int);                   \
  EXTERN template API void T::serialize(boost::archive::binary_oarchive &,     \
                                        const unsigned int);                   \
                                                                               \
  EXTERN template API void T::serialize(boost::archive::text_iarchive &,       \
                                        const unsigned int);                   \
  EXTERN template API void T::serialize(boost::archive::text_oarchive &,       \
                                        const unsigned int);
#else
#define CLAMS_INSTANTIATE_SERIALIZATION(EXTERN, API, T)
#endif

#if defined(SM_SHARED_LIBS) && defined(WIN32)
#define CLAMS_INSTANTIATE_SERIALIZATION_FUNCTION(EXTERN, API, T)               \
  EXTERN template API void serialize(boost::archive::binary_iarchive &,        \
                                     T &obj, const unsigned int);              \
  EXTERN template API void serialize(boost::archive::binary_oarchive &,        \
                                     T &obj, const unsigned int);              \
                                                                               \
  EXTERN template API void serialize(boost::archive::text_iarchive &, T &obj,  \
                                     const unsigned int);                      \
  EXTERN template API void serialize(boost::archive::text_oarchive &, T &obj,  \
                                     const unsigned int);
#else
#define CLAMS_INSTANTIATE_SERIALIZATION_FUNCTION(EXTERN, API, T)
#endif

#define CLAMS_INSTANTIATE_SERIALIZATION_FUNCTION_T(T)                          \
  template void serialize(boost::archive::binary_iarchive &, T &obj,           \
                          const unsigned int);                                 \
  template void serialize(boost::archive::binary_oarchive &, T &obj,           \
                          const unsigned int);                                 \
                                                                               \
  template void serialize(boost::archive::text_iarchive &, T &obj,             \
                          const unsigned int);                                 \
  template void serialize(boost::archive::text_oarchive &, T &obj,             \
                          const unsigned int);