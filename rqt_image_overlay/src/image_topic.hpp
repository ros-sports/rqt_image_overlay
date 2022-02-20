// Copyright 2022 Kenji Brameld
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

#ifndef IMAGE_TOPIC_HPP_
#define IMAGE_TOPIC_HPP_

class ImageTopic
{
public:
  ImageTopic(const std::string topic, const std::string transport)
  : topic(topic), transport(transport) {}

  const std::string topic;
  const std::string transport;
};

#endif  // IMAGE_TOPIC_HPP_
