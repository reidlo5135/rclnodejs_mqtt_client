// Copyright [2023] [wavem-reidlo]
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

'strict mode';

/**
 * .type.ts for sort MQTT request
 * @author wavem-reidlo
 * @version 1.0.0
 * @since 2023/03/31
*/  

/**
 * type JSON for define rcl connection type by MQTT request type
 */
export type MQTTRequest = {
    pub: string
    sub: string
    action: string
    service: string
};