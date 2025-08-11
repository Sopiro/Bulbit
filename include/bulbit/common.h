#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <format>
#include <functional>
#include <iostream>
#include <latch>
#include <limits>
#include <memory>
#include <memory_resource>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include "allocator.h"
#include "math.h"

#define BulbitAssert(A) assert(A)
#define BulbitNotUsed(x) ((void)(x))