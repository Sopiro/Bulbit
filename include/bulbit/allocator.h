#pragma once

#include <memory>
#include <memory_resource>

using BufferResource = std::pmr::monotonic_buffer_resource;
using PoolResource = std::pmr::unsynchronized_pool_resource;
using Allocator = std::pmr::polymorphic_allocator<std::byte>;
