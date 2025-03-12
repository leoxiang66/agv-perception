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

    std::optional<V> get(const K& key) const {
        auto it = data.find(key);
        if (it != data.end()) {
            return it->second;
        }
        return std::nullopt;
    }
};

#endif // SIZEDDICT_HPP
