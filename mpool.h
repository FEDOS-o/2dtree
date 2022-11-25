#pragma once

#include <assert.h>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <new>
#include <type_traits>

namespace pool {

class Pool
{
public:
    Pool(const size_t obj_size, const size_t obj_count)
        : m_obj_size(obj_size)
        , m_storage(obj_size * obj_count)
        , m_used_map(obj_count)
    {
    }

    size_t get_obj_size() const
    {
        return m_obj_size;
    }

    void * allocate(const size_t n)
    {
        const size_t pos = find_empty_place(n);
        if (pos != npos) {
            for (size_t i = pos, end = pos + n; i < end; ++i) {
                m_used_map[i] = true;
            }
            return &m_storage[pos * m_obj_size];
        }
        throw std::bad_alloc{};
    }

    void deallocate(void * ptr, const size_t n)
    {
        auto b_ptr = static_cast<const std::byte *>(ptr);
        const auto begin = &m_storage[0];
        if (b_ptr >= begin) {
            const size_t offset = (b_ptr - begin) / m_obj_size;
            //assert(((b_ptr - begin) % m_obj_size) == 0);
            if (offset < m_used_map.size()) {
                const size_t end_delete = offset + std::min(n, m_used_map.size() - offset);
                for (size_t i = offset; i < end_delete; ++i) {
                    m_used_map[i] = false;
                }
            }
        }
    }

private:
    static constexpr size_t npos = static_cast<size_t>(-1);

    size_t find_empty_place(const size_t n) const
    {
        if (n > m_used_map.size()) {
            return npos;
        }
        for (size_t i = 0; i < m_used_map.size(); ++i) {
            if (m_used_map[i]) {
                continue;
            }
            size_t j = i;
            for (size_t k = 0; k < n && j < m_used_map.size(); ++k, ++j) {
                if (m_used_map[j]) {
                    break;
                }
            }
            if (n == j - i) {
                return i;
            }
            i = j;
        }
        return npos;
    }

    const size_t m_obj_size;
    std::vector<std::byte> m_storage;
    std::vector<bool> m_used_map;
};

inline Pool * create_pool(const size_t obj_size, const size_t obj_count)
{
    return new Pool(obj_size, obj_count);
}

inline void destroy_pool(Pool * pool)
{
    delete pool;
}

inline size_t pool_obj_size(const Pool & pool)
{
    return pool.get_obj_size();
}

inline void * allocate(Pool & pool, const size_t n)
{
    return pool.allocate(n);
}

inline void deallocate(Pool & pool, void * ptr, const size_t n)
{
    pool.deallocate(ptr, n);
}

} // namespace pool

template <class T>
class PoolAllocator
{
public:
    using value_type = T;

    static inline pool::Pool * create_pool(const std::size_t obj_count)
    {
        return pool::create_pool(sizeof(T), obj_count);
    }
    static inline void destroy_pool(pool::Pool * pool)
    {
        pool::destroy_pool(pool);
    }

    PoolAllocator(const std::reference_wrapper<pool::Pool> & pool)
        : m_pool(pool)
    {
    }

    template <class U>
    PoolAllocator(const PoolAllocator<U> & other)
        : m_pool(other.m_pool)
    {
    }

    inline T * allocate(const std::size_t n)
    {
        return static_cast<T *>(pool::allocate(m_pool.get(), n));
    }
    inline void deallocate(T * ptr, const std::size_t n)
    {
        pool::deallocate(m_pool.get(), ptr, n);
    }
    std::reference_wrapper<pool::Pool> m_pool;

private:
};
