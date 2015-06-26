// #include "clams/common/lockable.h"

// namespace clams {
// void SharedLockable::lockWrite() { mutex_.lock(); }

// void SharedLockable::unlockWrite() { mutex_.unlock(); }

// bool SharedLockable::trylockWrite() { return mutex_.try_lock(); }

// void SharedLockable::lockRead() { mutex_.lock(); }

// void SharedLockable::unlockRead() { mutex_.unlock(); }

// bool SharedLockable::trylockRead() { return mutex_.try_lock(); }


// Lockable::Lockable() : lock_(mutex_) {lock_.unlock();}

// void Lockable::lock() { lock_.lock(); }

// void Lockable::unlock() { lock_.unlock(); }

// bool Lockable::trylock() {
//   return lock_.try_lock();
// }

// } // namespace clams