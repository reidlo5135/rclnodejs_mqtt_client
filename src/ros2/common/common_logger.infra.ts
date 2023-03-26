import winston from 'winston';
import process from 'process';
import winstonDaily from 'winston-daily-rotate-file';

const { combine, timestamp, label, printf } = winston.format;

const logDir = `${process.cwd()}/logs`;

const logFormat = printf(({ level, message, label, timestamp }) => {
   return `${timestamp} [${label}] ${level}: ${message}`;
});

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
      })
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