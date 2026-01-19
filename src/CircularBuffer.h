#pragma once

#include <array>
#include <cassert>
#include <cstddef>

template<typename T, size_t N>
struct CircularBuffer {
    std::array<T, N> mData = {};

    size_t Head = 0;
    size_t Tail = 0;

public:
    void Push(T Value)
    {
        mData[Head] = Value;

        Head = (Head + 1) % N;
        //assert(Head != Tail);
    }

    T Pop()
    {
        T Value = mData[Tail];
        Tail = (Tail + 1) % N;

        //assert(Tail != Head);
        return Value;
    }

    T operator [] (size_t Index)
    {
        return mData[Index];
    }

    // End may be less than requested when the distance to the end of the buffer is less than Count
    // You'll need to request multiple times when this happens
    void GetBucket(const size_t Count, size_t& Start, size_t& End)
    {
        Start = Tail;
        End = std::min((Start + Count), N-1);

        Tail = (End + 1) % N;
    }

    // Returns the distance between Head and Tail as a percentage
    // Values approaching 0 signal underrunning, while values approaching 1 signal overrunning.
    float GetPercentageFilled() const
    {
        if (Head < Tail)
        {
            return float(Head + (N - Tail)) / N;
        }
        else
        {
            return float(Head - Tail) / N;
        }
    }
};
