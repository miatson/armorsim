#pragma once
// -----------------------------------------------------------------------------
// ThreadPool.h - run a loop over particles on all CPU cores.
//
// parallelFor(n, body) splits [0, n) into one contiguous chunk per thread and
// calls body(begin, end) for each chunk at the same time.
//
// DETERMINISM: in every loop we parallelise, iteration i only WRITES data
// belonging to particle i (it may read anything). So the result can't depend
// on which thread ran which chunk, or in what order - every run gives
// bit-identical results, whatever the core count.
//
// The worker threads are created once and sleep between loops, because
// creating threads for each of the ~4 loops per step (×10,000s of steps)
// would waste a lot of time.
// -----------------------------------------------------------------------------
#include <algorithm>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <type_traits>
#include <vector>

namespace armor {

class ThreadPool {
public:
    explicit ThreadPool(unsigned threadCount = 0) {
        unsigned n = threadCount != 0 ? threadCount : std::thread::hardware_concurrency();
        n = std::max(1u, n);
        for (unsigned k = 1; k < n; ++k) {
            workers_.emplace_back([this, k] { workerLoop(k); });
        }
    }

    ~ThreadPool() {
        {
            std::lock_guard lock(mutex_);
            stop_ = true;
        }
        wake_.notify_all();
        for (std::thread& t : workers_) {
            t.join();
        }
    }

    ThreadPool(const ThreadPool&) = delete;
    ThreadPool& operator=(const ThreadPool&) = delete;

    unsigned size() const { return static_cast<unsigned>(workers_.size()) + 1; }

    template <class F>
    void parallelFor(std::size_t n, F&& body) {
        if (n == 0) {
            return;
        }
        // Tiny loops aren't worth waking threads for.
        if (workers_.empty() || n < 512) {
            body(std::size_t{0}, n);
            return;
        }
        using Fn = std::remove_reference_t<F>;
        {
            std::lock_guard lock(mutex_);
            context_ = const_cast<void*>(static_cast<const void*>(&body));
            invoke_ = [](void* ctx, std::size_t b, std::size_t e) { (*static_cast<Fn*>(ctx))(b, e); };
            count_ = n;
            remaining_ = static_cast<unsigned>(workers_.size());
            ++generation_;
        }
        wake_.notify_all();

        const auto [b, e] = chunk(0, n);
        body(b, e); // the calling thread does chunk 0

        std::unique_lock lock(mutex_);
        done_.wait(lock, [this] { return remaining_ == 0; });
    }

private:
    std::pair<std::size_t, std::size_t> chunk(unsigned index, std::size_t n) const {
        const std::size_t chunks = size();
        return {n * index / chunks, n * (index + 1) / chunks};
    }

    void workerLoop(unsigned index) {
        std::uint64_t seen = 0;
        for (;;) {
            std::unique_lock lock(mutex_);
            wake_.wait(lock, [&] { return stop_ || generation_ != seen; });
            if (stop_) {
                return;
            }
            seen = generation_;
            void* ctx = context_;
            auto* invoke = invoke_;
            const std::size_t n = count_;
            lock.unlock();

            const auto [b, e] = chunk(index, n);
            if (b < e) {
                invoke(ctx, b, e);
            }

            lock.lock();
            if (--remaining_ == 0) {
                done_.notify_one();
            }
        }
    }

    std::vector<std::thread> workers_;
    std::mutex mutex_;
    std::condition_variable wake_, done_;
    void* context_ = nullptr;
    void (*invoke_)(void*, std::size_t, std::size_t) = nullptr;
    std::size_t count_ = 0;
    unsigned remaining_ = 0;
    std::uint64_t generation_ = 0;
    bool stop_ = false;
};

} // namespace armor
