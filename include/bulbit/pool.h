#pragma once

#include "hash_map.h"

namespace bulbit
{

template <typename Key, typename Type, typename Hash = std::hash<Key>>
class Pool
{
public:
    Pool();
    ~Pool();

    template <typename... Args>
    Type* Create(const Key& key, Args&&... args);
    bool Delete(const Key& key);
    void Clear();

    HashMap<Key, Type*, Hash>& GetObjectMap();
    int32 PoolCount() const;

private:
    std::pmr::monotonic_buffer_resource buffer;
    std::pmr::polymorphic_allocator<std::byte> allocator;

    int32 count = 0;
    HashMap<Key, Type*, Hash> objects;
};

template <typename Key, typename Type, typename Hash>
inline Pool<Key, Type, Hash>::Pool()
    : buffer{}
    , allocator{ &buffer }
{
}

template <typename Key, typename Type, typename Hash>
inline Pool<Key, Type, Hash>::~Pool()
{
    Clear();
}

template <typename Key, typename Type, typename Hash>
template <typename... Args>
inline Type* Pool<Key, Type, Hash>::Create(const Key& key, Args&&... args)
{
    if (const auto* entry = objects.Contains(key))
    {
        // Already in pool
        return entry->value;
    }

    // Create new object using pool allocator
    Type* object = allocator.new_object<Type>(std::forward<Args>(args)...);

    objects.Insert(key, object);
    ++count;

    return object;
}

template <typename Key, typename Type, typename Hash>
inline bool Pool<Key, Type, Hash>::Delete(const Key& key)
{
    if (const auto* entry = objects.Contains(key))
    {
        allocator.delete_object(entry->value);
        objects.Erase(key);

        return true;
    }

    return false;
}

template <typename Key, typename Type, typename Hash>
inline void Pool<Key, Type, Hash>::Clear()
{
    for (auto& entry : objects)
    {
        allocator.delete_object(entry.value);
    }

    objects.Clear();
}

template <typename Key, typename Type, typename Hash>
inline HashMap<Key, Type*, Hash>& Pool<Key, Type, Hash>::GetObjectMap()
{
    return objects;
}

template <typename Key, typename Type, typename Hash>
inline int32 Pool<Key, Type, Hash>::PoolCount() const
{
    return count;
}

} // namespace bulbit
