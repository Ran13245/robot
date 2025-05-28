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

#pragma once

#include <vector>
#include <thread>
#include <unordered_map>
#include "MsgQueue.hpp"
namespace Schedule
{

    class TaskBase
    {
    public:
        TaskBase() = default;
        virtual ~TaskBase() = default;

        virtual void start()
        {
            thread_ = std::thread(&TaskBase::task, this);
            return;
        }

        virtual void stop()
        {

            if (thread_.joinable())
                thread_.join();
            return;
        }

        int bind_msg_queue(const std::string &name, IMsgQueue *msg_queue)
        {
            auto it = name_to_index.find(name);
            if (it != name_to_index.end())
            {
                return it->second;
            }

            int index = static_cast<int>(msg_queues.size());
            msg_queues.push_back(msg_queue);
            name_to_index[name] = index;

            return index;
        }

        IMsgQueue *quiry_msg_queue(const std::string &name)
        {
            auto it = name_to_index.find(name);
            if (it != name_to_index.end())
            {
                return msg_queues[it->second];
            }
            return nullptr;
        }

    protected:
        virtual void task() = 0;

    private:
        std::thread thread_;
        std::vector<IMsgQueue *> msg_queues;
        std::unordered_map<std::string, int> name_to_index;
    };
}; // namespace Schedule