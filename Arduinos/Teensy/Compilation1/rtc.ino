/**
 * Wrapper for time provider
 */
time_t getTeensy3Time()
{
  return Teensy3Clock.get();
}

/**
 * Prepare time library to use Teensy RTC
 */
void setupRTC()
{
  // set the Time library to use Teensy 3.0's RTC to keep time
  setSyncProvider(getTeensy3Time);
}

/**
 * Syncrhonize RTC clock to given timestamp
 */
void syncRTC(time_t stamp)
{

  Teensy3Clock.set(stamp); // set the RTC
  setTime(stamp);
}

