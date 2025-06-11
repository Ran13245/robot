// Copyright 2025 Chuangye Liu <chuangyeliu0206@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!abandon!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#pragma once
#ifndef __RING_BUF_H
#define __RING_BUF_H
#include <RingBuf.hpp>
class IMsgQueue
{
public:
    virtual ~IMsgQueue() = default;

    virtual std::shared_ptr<void> getRawBuffer() = 0;
    virtual bool enqueue(void *msg) = 0;
    virtual bool dequeue(void *out) = 0;
    virtual bool isEmpty() const = 0;
    virtual bool isFull() const = 0;
    virtual const std::type_info &type() const = 0;
};

template <typename T>
class MsgQueueImpl : public IMsgQueue
{
public:
    explicit MsgQueueImpl(size_t capacity)
        : capacity_(capacity), queue_(std::make_shared<RingBuffer<T>>(capacity)) {}

    std::shared_ptr<void> getRawBuffer() override
    {
        return queue_;
    }

    bool enqueue(void *msg) override
    {
        T *typed_msg = static_cast<T *>(msg);
        if (!typed_msg)
            return false;
        queue_->push(*typed_msg);
        return true;
    }

    bool dequeue(void *out) override
    {
        T *typed_out = static_cast<T *>(out);
        if (!typed_out)
            return false;
        return queue_->try_pop(*typed_out);
    }

    bool isEmpty() const override
    {
        return queue_->empty();
    }

    bool isFull() const override
    {
        return queue_->size() == capacity_;
    }

    const std::type_info &type() const override
    {
        return typeid(T);
    }

private:
    std::shared_ptr<RingBuffer<T>> queue_;
    size_t capacity_;
};

#endif