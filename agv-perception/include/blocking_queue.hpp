#pragma once

#include <mutex>
#include <condition_variable>
#include <deque>
#include <cstddef>
#include <optional>
/**
 * @brief A blocking queue with fixed capacity that drops the oldest element when full.
 *
 * push(T item):
 *   - If the queue is at capacity, pop the oldest element.
 *   - Push the new item and notify one waiting thread.
 *
 * pop():
 *   - Block until there's at least one element.
 *   - Pop and return the most recent element, clearing any older ones.
 */
template<typename T>
class BlockingQueue {
public:
    /**
     * @param capacity Maximum number of items to keep. When full, oldest items are dropped.
     */
    explicit BlockingQueue(std::size_t capacity)
        : cap_(capacity) {}

    // Push a new item; if full, drop the oldest.
    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.size() >= cap_) {
                queue_.pop_front();
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
    }

    /**
     * Block until at least one element is available, then return the latest.
     * Clears any older elements.
     */
    T pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]{ return !queue_.empty(); });
        T item = std::move(queue_.back());
        queue_.clear();
        return item;
    }

    // Try to pop the latest element without blocking. Returns std::nullopt if empty.
    std::optional<T> try_pop() {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) return std::nullopt;
        T item = std::move(queue_.back());
        queue_.clear();
        return item;
    }

    // Get current size (for diagnostics).
    std::size_t size() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.size();
    }

    // Check if empty.
    bool empty() const {
        std::lock_guard<std::mutex> lock(mtx_);
        return queue_.empty();
    }

private:
    const std::size_t cap_;
    mutable std::mutex mtx_;
    std::condition_variable cv_;
    std::deque<T> queue_;
};
