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

import process from 'process';
import winston from 'winston';
import winstonDaily from 'winston-daily-rotate-file';

/**
 * const instance for select winston's functions
 */
const { combine, timestamp, label, printf } = winston.format;

/**
 * const instance for define log files' directory
 */
const logDir = `${process.cwd()}/logs`;

/**
 * const instance for define logging message format
 */
const logFormat = printf(({ level, message, label, timestamp }) => {
   return `${timestamp} [${label}] ${level}: ${message}`;
});

/**
 * const instance for create winston logger
 * @see winston
 */
export const log = winston.createLogger({
    format: combine(
       timestamp({ format: 'YYYY-MM-DD HH:mm:ss' }),
       label({ label: 'ros2_mqtt_client' }),
       logFormat,
    ),
    transports: [
        new winstonDaily({
           level: 'info',
           datePattern: 'YYYY-MM-DD',
           dirname: logDir,
           filename: `%DATE%.log`,
           maxFiles: 30,
           zippedArchive: true,
        }),
        new winstonDaily({
           level: 'error',
           datePattern: 'YYYY-MM-DD',
           dirname: logDir + '/error',
           filename: `%DATE%.error.log`,
           maxFiles: 30,
           zippedArchive: true,
        }),
        new winstonDaily({
         level: 'debug',
         datePattern: 'YYYY-MM-DD',
         dirname: logDir + '/debug',
         filename: `%DATE%.debug.log`,
         maxFiles: 30,
         zippedArchive: true,
      }),
     ],
     exceptionHandlers: [
        new winstonDaily({
           level: 'error',
           datePattern: 'YYYY-MM-DD',
           dirname: logDir,
           filename: `%DATE%.exception.log`,
           maxFiles: 30,
           zippedArchive: true,
        }),
     ],
});

if (process.env.NODE_ENV !== 'production') {
    log.add(
       new winston.transports.Console({
          format: winston.format.combine(
             winston.format.colorize(),
             winston.format.simple(),
          ),
       }),
    );
 };