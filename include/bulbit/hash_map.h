#pragma once

#include "common.h"

namespace bulbit
{

template <typename K, typename V, typename Hasher>
class HashMapIterator;

enum class BucketState
{
    empty,
    occupied,
    deleted
};

template <typename K, typename V, typename Hasher = std::hash<K>>
class HashMap
{
public:
    struct Entry
    {
        K key;
        V value;
    };

    HashMap()
        : table{ 32 }
        , states{ 32, BucketState::empty }
        , occupied{ 0 }
    {
    }

    bool Insert(K key, V value)
    {
        constexpr float max_load = 0.7f;
        if (float(occupied) / table.size() > max_load)
        {
            Rehash();
        }

        size_t capacity = table.size();
        size_t base = HashKey(key) & (capacity - 1);
        size_t i = 0;

        while (i < capacity)
        {
            size_t probe = (base + (i * i + i) / 2) & (capacity - 1);
            if (states[probe] == BucketState::empty || states[probe] == BucketState::deleted)
            {
                table[probe] = Entry{ std::move(key), std::move(value) };
                states[probe] = BucketState::occupied;
                ++occupied;

                return true;
            }

            if (table[probe].key == key && states[probe] != BucketState::deleted)
            {
                table[probe].value = value; // update

                return false;
            }

            ++i;
        }

        return false;
    }

    const Entry* Contains(const K& key) const
    {
        size_t capacity = table.size();
        size_t base = HashKey(key) & (capacity - 1);
        size_t i = 0;

        while (i < capacity)
        {
            size_t probe = (base + (i * i + i) / 2) & (capacity - 1);
            if (states[probe] == BucketState::empty)
            {
                return nullptr;
            }

            if (states[probe] == BucketState::occupied && table[probe].key == key)
            {
                return &table[probe];
            }

            ++i;
        }

        return nullptr;
    }

    const V& At(const K& key) const
    {
        size_t capacity = table.size();
        size_t base = HashKey(key) & (capacity - 1);
        size_t i = 0;

        while (i < capacity)
        {
            size_t probe = (base + (i * i + i) / 2) & (capacity - 1);
            if (states[probe] == BucketState::empty)
            {
                BulbitAssert(false);
                return table[0].value;
            }

            if (states[probe] == BucketState::occupied && table[probe].key == key)
            {
                return table[probe].value;
            }

            ++i;
        }

        BulbitAssert(false);
        return table[0].value;
    }

    bool Erase(const K& key)
    {
        size_t capacity = table.size();
        size_t base = HashKey(key) & (capacity - 1);
        size_t i = 0;

        while (i < capacity)
        {
            size_t probe = (base + (i * i + i) / 2) & (capacity - 1);
            if (states[probe] == BucketState::empty)
            {
                return false;
            }

            if (states[probe] == BucketState::occupied && table[probe].key == key)
            {
                --occupied;
                states[probe] = BucketState::deleted;
                return true;
            }
            ++i;
        }

        return false;
    }

    void Clear()
    {
        table.clear();
        states.clear();
        occupied = 0;
    }

    HashMapIterator<K, V, Hasher> begin()
    {
        size_t capacity = table.size();
        HashMapIterator<K, V, Hasher> it(table.data(), table.data() + capacity, states.data(), states.data() + capacity);
        while (it.ptr < it.end && *it.ptr_s != BucketState::occupied)
        {
            ++it.ptr;
            ++it.ptr_s;
        }

        return it;
    }

    HashMapIterator<K, V, Hasher> end()
    {
        size_t capacity = table.size();
        return HashMapIterator<K, V, Hasher>(
            table.data() + capacity, table.data() + capacity, states.data() + capacity, states.data() + capacity
        );
    }

private:
    friend class HashMapIterator<K, V, Hasher>;

    std::vector<Entry> table;
    std::vector<BucketState> states;
    size_t occupied;

    size_t HashKey(const K& key) const
    {
        return Hasher{}(key);
    }

    void Rehash()
    {
        std::vector<Entry> old_table = std::move(table);
        std::vector<BucketState> old_states = std::move(states);
        table.clear();
        states.clear();

        size_t capacity = old_table.size();
        table.resize(2 * capacity);
        states.resize(2 * capacity, BucketState::empty);

        occupied = 0;

        for (size_t i = 0; i < capacity; ++i)
        {
            if (old_states[i] == BucketState::occupied)
            {
                Insert(std::move(old_table[i].key), std::move(old_table[i].value));
            }
        }
    }
};

template <typename K, typename V, typename Hasher>
class HashMapIterator
{
public:
    using Entry = HashMap<K, V, Hasher>::Entry;

    HashMapIterator(Entry* ptr, Entry* end, BucketState* ptr_s, BucketState* end_s)
        : ptr{ ptr }
        , end{ end }
        , ptr_s{ ptr_s }
        , end_s{ end_s }
    {
    }

    HashMapIterator& operator++()
    {
        do
        {
            ++ptr;
            ++ptr_s;
        } while (ptr < end && *ptr_s != BucketState::occupied);
        return *this;
    }

    HashMapIterator operator++(int)
    {
        HashMapIterator old = *this;
        operator++();
        return old;
    }

    bool operator==(const HashMapIterator& iter) const
    {
        return ptr == iter.ptr;
    }

    bool operator!=(const HashMapIterator& iter) const
    {
        return ptr != iter.ptr;
    }

    Entry& operator*()
    {
        return *ptr;
    }

    const Entry& operator*() const
    {
        return *ptr;
    }

private:
    friend HashMap<K, V, Hasher>;

    HashMap<K, V, Hasher>::Entry* ptr;
    HashMap<K, V, Hasher>::Entry* end;
    BucketState* ptr_s;
    BucketState* end_s;
};

} // namespace bulbit
