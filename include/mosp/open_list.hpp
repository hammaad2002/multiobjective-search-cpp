// Binary min-heap used as the OPEN list of the searches.
//
// Entries are small PODs {key, label id}: the key is the label's f-vector
// (compared lexicographically), the id indexes the search's label pool. The
// heap's storage is a std::vector that is cleared, not freed, between queries,
// so after warm-up pushes and pops do not allocate.
#pragma once

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace mosp {

template <typename Key>
class BinaryHeap {
 public:
  struct Entry {
    Key key;
    std::uint32_t id;
  };

  void reserve(std::size_t n) { data_.reserve(n); }
  void clear() noexcept { data_.clear(); }
  [[nodiscard]] bool empty() const noexcept { return data_.empty(); }
  [[nodiscard]] std::size_t size() const noexcept { return data_.size(); }
  [[nodiscard]] const Entry& top() const noexcept { return data_.front(); }

  void push(const Key& key, std::uint32_t id) {
    data_.push_back(Entry{key, id});
    sift_up(data_.size() - 1);
  }

  Entry pop() {
    Entry result = data_.front();
    data_.front() = data_.back();
    data_.pop_back();
    if (!data_.empty()) sift_down(0);
    return result;
  }

 private:
  // Strict lexicographic order on keys (std::array provides operator<).
  static bool less(const Entry& a, const Entry& b) noexcept { return a.key < b.key; }

  void sift_up(std::size_t i) noexcept {
    Entry e = data_[i];
    while (i > 0) {
      const std::size_t parent = (i - 1) / 2;
      if (!less(e, data_[parent])) break;
      data_[i] = data_[parent];
      i = parent;
    }
    data_[i] = e;
  }

  void sift_down(std::size_t i) noexcept {
    const std::size_t n = data_.size();
    Entry e = data_[i];
    for (;;) {
      std::size_t child = 2 * i + 1;
      if (child >= n) break;
      if (child + 1 < n && less(data_[child + 1], data_[child])) ++child;
      if (!less(data_[child], e)) break;
      data_[i] = data_[child];
      i = child;
    }
    data_[i] = e;
  }

  std::vector<Entry> data_;
};

}  // namespace mosp
