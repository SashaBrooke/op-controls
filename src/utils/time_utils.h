#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#define FREQ2PERIOD(freq) ((freq) != 0 ? (1.0f / (freq)) : 0.0f) // Converts a frequency in Hz to a period in seconds
#define SECS2MSECS(secs)  ((secs) * 1000)                        // Converts a time from seconds to milliseconds
#define SECS2USECS(secs)  ((secs) * 1000000)                     // Converts a time from seconds to micro-seconds

#endif // TIME_UTILS_H
