#pragma once

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "src/problem/problem.hpp"
#include "src/type_defs.hpp"

struct CompactState {
  inline static variable_t numFinDomain;
  std::vector<value_t> finDomain{};
  std::vector<bool> preposition{};
  CompactState(const State &other) {
    finDomain.clear();
    finDomain.reserve(numFinDomain);
    preposition.clear();
    preposition.reserve(other.size() - numFinDomain);
    for (size_t i = 0; i < numFinDomain; ++i) {
      finDomain.push_back(other[i]);
    }
    for (size_t i = numFinDomain; i < other.size(); ++i) {
      assert(other[i] < 2);
      preposition.push_back(other[i]);
    }
  }

  inline value_t set(variable_t i, value_t value) {
    value_t oldValue;
    if (i < numFinDomain) {
      oldValue     = finDomain[i];
      finDomain[i] = value;
    } else {
      oldValue = preposition[static_cast<size_t>(i - numFinDomain)];
      preposition[static_cast<size_t>(i - numFinDomain)] = value;
    }
    return oldValue;
  }
};

struct StateHash {
  // random number for each value of a finite domain variable
  // used for fast hashing
  inline static std::vector<std::vector<size_t>> zobrist;

  size_t operator()(const CompactState &s) const {
    // The Zobrist hash function is only used for the finite domain variables.
    // It is way faster than computing the hash of single variables but it
    // cannot offset the cost of iterating over (compact) bool vectors.
    assert(zobrist.size() == CompactState::numFinDomain);
    size_t hash = 0;
    for (size_t i = 0; i < s.finDomain.size(); ++i) {
      hash ^= zobrist[i][s.finDomain[i]];
    }
    hash ^= std::hash<std::vector<bool>>()(s.preposition);
    return hash;
  }
};

struct StateComp {
  bool operator()(const CompactState s1, const CompactState s2) const {
    return s1.finDomain == s2.finDomain && s1.preposition == s2.preposition;
  }
};

// TODO States are big. they are moved here on resize
using StateHashMap =
    std::unordered_map<CompactState, size_t, StateHash, StateComp>;

using StateHashSet = std::unordered_set<CompactState, StateHash, StateComp>;
