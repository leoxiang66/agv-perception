#ifndef SIZEDDICT_HPP
#define SIZEDDICT_HPP

#include <unordered_map>
#include <deque>
#include <optional>

template <typename K, typename V>
class SizedDict {
private:
    std::unordered_map<K, V> data;
    std::deque<K> keys;
    size_t max_size;

public:
    explicit SizedDict(size_t size) : max_size(size) {}

    void insert(const K& key, const V& value) {
        if (data.size() >= max_size) {
            K oldest_key = keys.front();
            keys.pop_front();
            data.erase(oldest_key);
        }
        keys.push_back(key);
        data[key] = value;
    }

    V get(const K& key) const {
        return data.at(key);
    }

    std::deque<K> get_keys() const {
        return keys;
    }

    bool contains(const K& key) const {
        return std::find(keys.begin(), keys.end(), key) != keys.end();
    }
};

#endif // SIZEDDICT_HPP
