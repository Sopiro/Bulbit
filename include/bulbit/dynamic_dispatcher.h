#pragma once

#include <type_traits>

namespace detail
{

template <typename T, typename... Types>
struct IndexOf;

// When the first element of the list matches the type
template <typename T, typename... Types>
struct IndexOf<T, T, Types...>
{
    static constexpr int value = 0;
};

// When the first element of the list does not match the type
template <typename T, typename First, typename... Types>
struct IndexOf<T, First, Types...>
{
    static constexpr int value = 1 + IndexOf<T, Types...>::value;
};

// Error case
template <typename T>
struct IndexOf<T>
{
    static_assert(sizeof(T) == 0, "Type not found in the provided type list");
};

template <typename... Ts>
struct IsSameType;

template <>
struct IsSameType<>
{
    static constexpr bool value = true;
};
template <typename T>
struct IsSameType<T>
{
    static constexpr bool value = true;
};

template <typename T, typename U, typename... Ts>
struct IsSameType<T, U, Ts...>
{
    static constexpr bool value = (std::is_same_v<T, U> && IsSameType<U, Ts...>::value);
};

template <typename... Ts>
struct SameType;

template <typename T, typename... Ts>
struct SameType<T, Ts...>
{
    using type = T;
    static_assert(IsSameType<T, Ts...>::value, "Not all types in pack are the same");
};

template <typename F, typename... Ts>
struct ReturnType
{
    using type = typename SameType<std::invoke_result_t<F, Ts*>...>::type;
};

template <typename F, typename... Ts>
struct ReturnTypeConst
{
    using type = typename SameType<std::invoke_result_t<F, const Ts*>...>::type;
};

} // namespace detail

template <typename... Types>
class DynamicDispatcher
{
protected:
    const uint8_t type_index;

    DynamicDispatcher(uint8_t type_index)
        : type_index{ type_index }
    {
        assert(type_index < sizeof...(Types));
    }

public:
    template <typename T>
    bool Is() const
    {
        return TypeIndex<T>() == type_index;
    }

    template <typename T>
    T* Cast()
    {
        assert(Is<T>());
        return reinterpret_cast<T*>(this);
    }

    template <typename T>
    const T* Cast() const
    {
        assert(Is<T>());
        return reinterpret_cast<const T*>(this);
    }

    template <typename Func>
    auto Dispatch(Func&& func)
    {
        using R = detail::ReturnType<Func, Types...>::type;
        using Handler = R (*)(void*, Func&&);

        static constexpr Handler handlers[] = { [](void* p, Func&& f) -> R {
            return f(static_cast<std::add_pointer_t<Types>>(p));
        }... };

        return handlers[type_index]((void*)this, std::forward<Func>(func));
    }

    template <typename Func>
    auto Dispatch(Func&& func) const
    {
        using R = detail::ReturnType<Func, Types...>::type;
        using Handler = R (*)(void*, Func&&);

        static constexpr Handler handlers[] = { [](void* p, Func&& f) -> R {
            return f(static_cast<std::add_pointer_t<Types>>(p));
        }... };

        return handlers[type_index]((void*)this, std::forward<Func>(func));
    }

    template <typename T>
    static constexpr uint8_t TypeIndex()
    {
        using Type = typename std::remove_cv_t<T>;
        if constexpr (std::is_same_v<Type, std::nullptr_t>)
            return 0;
        else
            return 1 + detail::IndexOf<Type, Types>::count;
    }
};