#pragma once

#include <array>
#include <cassert>
#include <cstddef>

template<typename T, unsigned int N>
class CircularBuffer {
    std::array<T, N> mData = {};

    size_t Head = 0;
    size_t Tail = 0;

public:
    void Push(T Value)
    {
        mData[Head] = Value;

        Head = (Head + 1) % N;
        assert(Head != Tail);
    }

    T Pop()
    {
        T Value = mData[Tail];
        Tail = (Tail + 1) % N;

        assert(Head != Tail);
        return Value;
    }
};
