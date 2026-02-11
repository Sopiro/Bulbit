#pragma once

#include "common.h"

namespace bulbit
{

template <typename K, typename V, typename Hasher>
class HashMapIterator;
template <typename K, typename V, typename Hasher>
class HashMapConstIterator;

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
        : occupied{ 0 }
    {
        // Initial size must be power of two
        Resize(32);
    }

    bool Insert(K key, V value)
    {
        if (occupied >= table.size() * 0.7f)
        {
            Rehash(table.size() * 2);
        }

        size_t mask = table.size() - 1;

        size_t hash = HashKey(key);
        size_t current_dist = 0; // Distance from initial bucket
        size_t index = hash & mask;

        Entry entry = { std::move(key), std::move(value) };
        while (true)
        {
            // Found empty bucket
            if (dists[index] == -1)
            {
                table[index] = std::move(entry);
                dists[index] = int32(current_dist);
                ++occupied;
                return true;
            }

            // Update value
            if (table[index].key == entry.key)
            {
                table[index].value = std::move(entry.value);
                return false;
            }

            // The Robin Hood step
            if (int32(current_dist) > dists[index])
            {
                std::swap(entry, table[index]);

                int32 old_dist = dists[index];
                dists[index] = int32(current_dist);
                current_dist = old_dist;
            }

            // Move to next bucket (linear probing)
            ++current_dist;
            index = (index + 1) & mask;
        }
    }

    const Entry* Contains(const K& key) const
    {
        size_t mask = table.size() - 1;
        size_t hash = HashKey(key);

        size_t index = hash & mask;
        size_t dist = 0;

        while (true)
        {
            if (dists[index] == -1)
            {
                return nullptr;
            }

            if (dist > size_t(dists[index]))
            {
                return nullptr;
            }

            if (table[index].key == key)
            {
                return &table[index];
            }

            index = (index + 1) & mask;
            ++dist;
        }
    }

    V& At(const K& key)
    {
        size_t capacity = table.size();
        size_t mask = capacity - 1;

        size_t hash = HashKey(key);
        size_t index = hash & mask;
        size_t dist = 0;

        while (true)
        {
            if (table[index].key == key)
            {
                return table[index].value;
            }

            index = (index + 1) & mask;
            ++dist;

            // Safety break
            if (dist > capacity)
            {
                BulbitAssert(false && "Use Contains() instead!");
                break;
            }
        }

        return table[capacity].value;
    }

    const V& At(const K& key) const
    {
        size_t capacity = table.size();
        size_t mask = capacity - 1;

        size_t hash = HashKey(key);
        size_t index = hash & mask;
        size_t dist = 0;

        while (true)
        {
            if (table[index].key == key)
            {
                return table[index].value;
            }

            index = (index + 1) & mask;
            ++dist;

            // Safety break
            if (dist > capacity)
            {
                BulbitAssert(false && "Use Contains() instead!");
                break;
            }
        }

        return table[capacity].value;
    }

    bool Erase(const K& key)
    {
        size_t mask = table.size() - 1;
        size_t hash = HashKey(key);

        size_t index = hash & mask;
        size_t dist = 0;

        while (true)
        {
            if (dists[index] == -1 || dist > size_t(dists[index]))
            {
                return false; // Not found
            }

            if (table[index].key == key)
            {
                Remove(index);
                --occupied;
                return true;
            }

            index = (index + 1) & mask;
            ++dist;
        }
    }

    void Clear()
    {
        occupied = 0;
        size_t capacity = table.size();

        table.clear();
        table.resize(capacity);

        std::fill(dists.begin(), dists.end(), -1);
    }

    size_t Size() const
    {
        return occupied;
    }

    HashMapIterator<K, V, Hasher> begin()
    {
        size_t first_index = 0;
        while (first_index < table.size() && dists[first_index] == -1)
        {
            ++first_index;
        }
        return HashMapIterator<K, V, Hasher>(this, first_index);
    }

    HashMapIterator<K, V, Hasher> end()
    {
        return HashMapIterator<K, V, Hasher>(this, table.size());
    }

    HashMapConstIterator<K, V, Hasher> begin() const
    {
        size_t first_index = 0;
        while (first_index < table.size() && dists[first_index] == -1)
        {
            ++first_index;
        }
        return HashMapConstIterator<K, V, Hasher>(this, first_index);
    }

    HashMapConstIterator<K, V, Hasher> end() const
    {
        return HashMapConstIterator<K, V, Hasher>(this, table.size());
    }

private:
    friend class HashMapIterator<K, V, Hasher>;
    friend class HashMapConstIterator<K, V, Hasher>;

    std::vector<Entry> table;
    std::vector<int32> dists; // probe segement lengths. -1 means empty.

    size_t occupied;

    size_t HashKey(const K& key) const
    {
        return Hasher{}(key);
    }

    void Rehash(size_t new_capacity)
    {
        std::vector<Entry> old_table = std::move(table);
        std::vector<int32> old_dists = std::move(dists);

        Resize(new_capacity);

        for (size_t i = 0; i < old_table.size(); ++i)
        {
            if (old_dists[i] != -1)
            {
                Insert(std::move(old_table[i].key), std::move(old_table[i].value));
            }
        }
    }

    void Resize(size_t new_capacity)
    {
        table.resize(new_capacity);
        dists.assign(new_capacity, -1);
        occupied = 0;
    }

    void Remove(size_t index)
    {
        size_t mask = table.size() - 1;
        size_t curr = index;
        size_t next = (curr + 1) & mask;

        while (true)
        {
            if (dists[next] == -1 || dists[next] == 0)
            {
                dists[curr] = -1;
                return;
            }

            // Back shift deletion
            table[curr] = std::move(table[next]);
            dists[curr] = dists[next] - 1;

            curr = next;
            next = (curr + 1) & mask;
        }
    }
};

template <typename K, typename V, typename Hasher>
class HashMapIterator
{
public:
    using Entry = typename HashMap<K, V, Hasher>::Entry;

    HashMapIterator(HashMap<K, V, Hasher>* map, size_t index)
        : map{ map }
        , index{ index }
    {
    }

    HashMapIterator& operator++()
    {
        do
        {
            ++index;
        } while (index < map->table.size() && map->dists[index] == -1);

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
        return index == iter.index && map == iter.map;
    }

    bool operator!=(const HashMapIterator& iter) const
    {
        return !(*this == iter);
    }

    Entry& operator*()
    {
        return map->table[index];
    }

    const Entry& operator*() const
    {
        return map->table[index];
    }

    Entry* operator->()
    {
        return &map->table[index];
    }

private:
    HashMap<K, V, Hasher>* map;
    size_t index;
};

template <typename K, typename V, typename Hasher>
class HashMapConstIterator
{
public:
    using Entry = typename HashMap<K, V, Hasher>::Entry;

    HashMapConstIterator(const HashMap<K, V, Hasher>* map, size_t index)
        : map{ map }
        , index{ index }
    {
    }

    HashMapConstIterator& operator++()
    {
        do
        {
            ++index;
        } while (index < map->table.size() && map->dists[index] == -1);

        return *this;
    }

    HashMapConstIterator operator++(int)
    {
        HashMapConstIterator old = *this;
        operator++();
        return old;
    }

    bool operator==(const HashMapConstIterator& iter) const
    {
        return index == iter.index && map == iter.map;
    }

    bool operator!=(const HashMapConstIterator& iter) const
    {
        return !(*this == iter);
    }

    const Entry& operator*() const
    {
        return map->table[index];
    }

    const Entry* operator->() const
    {
        return &map->table[index];
    }

private:
    const HashMap<K, V, Hasher>* map;
    size_t index;
};

} // namespace bulbit
