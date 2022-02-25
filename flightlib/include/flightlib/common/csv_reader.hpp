#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace flightlib {

class CSVRow {
 public:
  std::string_view operator[](std::size_t index) const {
    return std::string_view(&m_line[m_data[index] + 1],
                            m_data[index + 1] - (m_data[index] + 1));
  }
  std::size_t size() const { return m_data.size() - 1; }
  void readNextRow(std::istream& str) {
    std::getline(str, m_line);

    m_data.clear();
    m_data.emplace_back(-1);
    std::string::size_type pos = 0;
    while ((pos = m_line.find(',', pos)) != std::string::npos) {
      m_data.emplace_back(pos);
      ++pos;
    }
    // This checks for a trailing comma with no data after it.
    pos = m_line.size();
    m_data.emplace_back(pos);
  }

 private:
  std::string m_line;
  std::vector<int> m_data;
};

inline std::istream& operator>>(std::istream& str, CSVRow& data) {
  data.readNextRow(str);
  return str;
}


class CSVIterator {
 public:
  using iterator_category = std::input_iterator_tag;
  using value_type = CSVRow;
  using difference_type = std::size_t;
  using pointer = CSVRow*;
  using reference = CSVRow&;

  CSVIterator(std::istream& str) : m_str(str.good() ? &str : nullptr) {
    ++(*this);
  }
  CSVIterator() : m_str(nullptr) {}

  // Pre Increment
  CSVIterator& operator++() {
    if (m_str) {
      if (!((*m_str) >> m_row)) {
        m_str = nullptr;
      }
    }
    return *this;
  }
  // Post increment
  CSVIterator operator++(int) {
    CSVIterator tmp(*this);
    ++(*this);
    return tmp;
  }
  CSVRow const& operator*() const { return m_row; }
  CSVRow const* operator->() const { return &m_row; }

  bool operator==(CSVIterator const& rhs) {
    return ((this == &rhs) ||
            ((this->m_str == nullptr) && (rhs.m_str == nullptr)));
  }
  bool operator!=(CSVIterator const& rhs) { return !((*this) == rhs); }

 private:
  std::istream* m_str;
  CSVRow m_row;
};

class CSVRange {
  std::istream& stream;

 public:
  CSVRange(std::istream& str) : stream(str) {}
  CSVIterator begin() const { return CSVIterator{stream}; }
  CSVIterator end() const { return CSVIterator{}; }
};


}  // namespace flightlib