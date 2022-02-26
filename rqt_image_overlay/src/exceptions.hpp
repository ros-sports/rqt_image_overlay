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

#ifndef EXCEPTIONS_HPP_
#define EXCEPTIONS_HPP_

namespace rqt_image_overlay
{

class StorageEmptyException : public std::runtime_error
{
public:
  StorageEmptyException()
  : std::runtime_error("zero messages in storage") {}
};

}  // namespace rqt_image_overlay

#endif  // EXCEPTIONS_HPP_
