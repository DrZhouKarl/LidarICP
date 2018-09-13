#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>


template <typename T>
class ThreadSafeQueue
{
public:
    void push(T data)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_dataQueue.push(std::move(data));
        m_condition.notify_one();
    }

    void waitAndPop(T& value)
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_condition.wait(lock, [this] { return !m_dataQueue.empty(); });
        value = std::move(m_dataQueue.front());
        m_dataQueue.pop();
    }

    T waitAndPop()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        m_condition.wait(lock, [this] { return !m_dataQueue.empty(); });
        T value = std::move(m_dataQueue.front());
        m_dataQueue.pop();
        return value;
    }

    bool isEmpty() const
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_dataQueue.empty();
    }

    size_t size()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        return m_dataQueue.size();
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(m_mutex);
        std::queue<T> empty;
        std::swap(m_dataQueue, empty);
    }

private:
    mutable std::mutex m_mutex;
    std::queue<T> m_dataQueue;
    std::condition_variable m_condition;
};
