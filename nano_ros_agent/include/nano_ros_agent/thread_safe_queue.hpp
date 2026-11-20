#pragma once
#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <functional>

template <typename T>
class ThreadSafeQueue {
public:
    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (queue_.size() >= max_queue_size_) {
                queue_.pop_front();
            }
            queue_.push_back(std::move(item));
        }
        cv_.notify_one();
        if (on_push_) {
            on_push_();
        }
    }

    // returns false right away if the queue is empty
    bool try_pop(T &item) {
        std::lock_guard<std::mutex> lock(mtx_);
        if (queue_.empty()) {
            return false;
        }
        item = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

    // called after every push, outside the lock. Set it before any other thread can push
    void set_on_push(std::function<void()> cb) {
        on_push_ = std::move(cb);
    }

    // blocks by sleeping thread if nothing in queue
    T wait_and_pop() {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [this]{ return !queue_.empty() || stopping_; });
        T item = std::move(queue_.front());
        queue_.pop_front();
        return item;
    }

    size_t size() {
        std::unique_lock<std::mutex> lock(mtx_);
        return queue_.size();
    }

    void stop() {
        stopping_ = true;
        cv_.notify_all();
    }
private:
    static constexpr size_t max_queue_size_ = 100;
    std::mutex mtx_;
    std::deque<T> queue_;
    std::condition_variable cv_;
    std::atomic<bool> stopping_{false};
    std::function<void()> on_push_;
};