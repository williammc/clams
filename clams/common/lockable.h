// #pragma once

// #include <mutex>

// namespace clams {
// #define scopeLockRead                                                          \
//   std::unique_lock<std::mutex> lockable_shared_lock(mutex_)
// #define scopeLockWrite                                                         \
//   std::unique_lock<std::mutex> lockable_unique_lock(mutex_)

// /* SharedLockable is based on the uncopyable std::mutex.
//    This presents a dilemma when assigning or copy constructing.
//    Right now, the state of the mutex in the other SharedLockable
//    does not get copied to the target SharedLockable.
//    I'm not sure yet if this is the desired behavior.
// */
// class SharedLockable {
// public:
//   SharedLockable() {}
//   //! Copy constructor will make a new mutex that is unlocked.
//   SharedLockable(const SharedLockable &other) {}
//   //! Assignment operator will *not* copy the mutex_ or the state of
//   //mutex_ from other.
//   SharedLockable &operator=(const SharedLockable &other) { return *this; }

//   void lockWrite();
//   void unlockWrite();
//   bool trylockWrite();

//   void lockRead();
//   void unlockRead();
//   bool trylockRead();

// protected:
//   //! For the first time ever, I'm tempted to make this mutable.
//   //! It'd make user methods still be able to be const even if they are locking.
//   std::mutex mutex_;
// };


// class Lockable {
// public:
//   std::mutex mutex_;
//   std::unique_lock<std::mutex> lock_;

//   Lockable();
//   void lock();
//   void unlock();
//   bool trylock();
// };

// } // namespace clams