#include <cstddef>       // 用于 size_t
#include <algorithm>     // 用于 std::equal
#include "functional"    // std:func
#include "Preferences.h" // 储存

template <typename T, std::size_t N>
bool areArraysEqual(const T (&a)[N], const T (&b)[N])
{
  return std::equal(a, a + N, b);
};

/**
 * @brief 简化运行 Preferences 上下文
 * @param name_space 命名空间
 * @param cb_fn 执行回调
 */
void nvs_call(const char *name_space, std::function<void(Preferences &)> cb_fn);

void setPowerCheck();
