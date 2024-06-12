from datetime import datetime, timedelta, timezone
import pytz

# Constants
GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)
BDS_EPOCH = datetime(2006, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

# Example leap seconds (add actual leap seconds here)
# Leap seconds are given as UTC datetime when they were added
LEAP_SECONDS = [
    datetime(1981, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1982, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1983, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1985, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1987, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1989, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1990, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1992, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1993, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1994, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1995, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1997, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(1998, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(2005, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(2008, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
    datetime(2012, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(2015, 6, 30, 23, 59, 59, tzinfo=timezone.utc),
    datetime(2016, 12, 31, 23, 59, 59, tzinfo=timezone.utc),
]
def count_leap_seconds(up_to_time):
    return sum(1 for leap_second in LEAP_SECONDS if leap_second <= up_to_time)

# ==================================================================
def gps_to_unix(gps_time):
    utc_time = gps_to_utc(gps_time)
    return int(utc_time.timestamp())


def gps_to_unix(gps_time):
    utc_time = gps_to_utc(gps_time)
    return int(utc_time.timestamp())
  
def gps_to_utc(gps_time):
    return GPS_EPOCH + timedelta(seconds=gps_time)  
def gps_to_utc_leap(gps_time):
  # GPS time started without leap seconds
  leap_seconds = count_leap_seconds(GPS_EPOCH + timedelta(seconds=gps_time))
  return GPS_EPOCH + timedelta(seconds=gps_time + leap_seconds)
# ==================================================================
def bds_to_unix(bds_time):
    utc_time = bds_to_utc(bds_time)
    return int(utc_time.timestamp())  
  
def bds_to_utc(bds_time):
    return BDS_EPOCH + timedelta(seconds=bds_time)
def bds_to_utc_leap(bds_time):
    # BDS time started without leap seconds
    leap_seconds = count_leap_seconds(BDS_EPOCH + timedelta(seconds=bds_time))
    return BDS_EPOCH + timedelta(seconds=bds_time + leap_seconds)
  
  
# ==================================================================
# def utc_to_gps(utc_time):
#     return int((utc_time - GPS_EPOCH).total_seconds())
def utc_to_gps_leap(utc_time):
  leap_seconds = count_leap_seconds(utc_time)
  return int((utc_time - GPS_EPOCH).total_seconds()) - leap_seconds
def utc_to_bds_leap(utc_time):
    leap_seconds = count_leap_seconds(utc_time)
    return int((utc_time - BDS_EPOCH).total_seconds()) - leap_seconds
# def utc_to_bds(utc_time):
#     return int((utc_time - BDS_EPOCH).total_seconds())
def utc_to_unix(utc_time):
    return int(utc_time.timestamp())
  
def display_times(time):
    utc_0 = time.astimezone(pytz.utc)
    utc_8 = time.astimezone(pytz.timezone('Asia/Shanghai'))
    unix_time = utc_to_unix(time)
    print(f"Total seconds (UNIX time): {unix_time}")
    print(f"UTC Time (+8): {utc_8.strftime('%Y-%m-%d %H:%M:%S %Z%z')}")
    print(f"UTC Time (0): {utc_0.strftime('%Y-%m-%d %H:%M:%S %Z%z')}")
# ==================================================================
def unix_to_gps(unix_time):
    utc_time = datetime.fromtimestamp(unix_time, tz=timezone.utc)
    return utc_to_gps_leap(utc_time)
def unix_to_bds(unix_time):
    utc_time = datetime.fromtimestamp(unix_time, tz=timezone.utc)
    return utc_to_bds_leap(utc_time)
def unix_to_utc(unix_time):
    return datetime.fromtimestamp(unix_time, tz=timezone.utc)



# Example times
gps_time = 1000000000  # Example GPS time in seconds
bds_time = 1000000000  # Example BDS time in seconds
unix_time = 1717567180  # Example UNIX time in seconds

# # Convert and display
# print("From GPS Time:")
# utc_time_from_gps = gps_to_utc(gps_time)
# display_times(utc_time_from_gps)

# print("\nFrom BDS Time:")
# utc_time_from_bds = bds_to_utc(bds_time)
# display_times(utc_time_from_bds)

# print("\nFrom UNIX Time:")
# utc_time_from_unix = unix_to_utc(unix_time)
# display_times(utc_time_from_unix)

# print("\nConvert UTC Time to other times:")
# current_utc_time = datetime.now(timezone.utc)
# print(f"Current UTC Time: {current_utc_time.strftime('%Y-%m-%d %H:%M:%S %Z%z')}")
# print(f"GPS Time: {utc_to_gps(current_utc_time)}")
# print(f"BDS Time: {utc_to_bds(current_utc_time)}")
# print(f"UNIX Time: {utc_to_unix(current_utc_time)}")


# Convert and display
print("From UNIX Time:")
utc_time_from_unix = unix_to_utc(unix_time)
display_times(utc_time_from_unix)
print(f"GPS Time: {unix_to_gps(unix_time)}")
print(f"BDS Time: {unix_to_bds(unix_time)}")

print("\nConvert Current UTC Time to other times:")
current_utc_time = datetime.now(timezone.utc)
print(f"Current UTC Time: {current_utc_time.strftime('%Y-%m-%d %H:%M:%S %Z%z')}")
print(f"GPS Time: {utc_to_gps_leap(current_utc_time)}")
print(f"BDS Time: {utc_to_bds_leap(current_utc_time)}")
print(f"UNIX Time: {utc_to_unix(current_utc_time)}")


print("\nConvert GPS and BDS Time to UNIX:")
print(f"GPS Time: {gps_time} -> UNIX Time: {gps_to_unix(gps_time)}")
print(f"BDS Time: {bds_time} -> UNIX Time: {bds_to_unix(bds_time)}")

