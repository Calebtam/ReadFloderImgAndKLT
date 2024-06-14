from datetime import datetime, timedelta, timezone
import pytz

# Constants
# 
GPS_EPOCH = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc)

# 
BDS_EPOCH = datetime(2006, 1, 1, 0, 0, 0, tzinfo=timezone.utc)

# UTC(SU) 与UTC 在 1983年1月1日00:00:00对齐 
# GLONASS_EPOCH 定义了GLONASS系统的起始时间点，即1983年1月1日的00:00:00 UTC(SU)。
# GLONASS_OFFSET 定义了GLONASS时间相对于UTC(SU)时间的固定偏差，即3小时。
GLONASS_EPOCH = datetime(1983, 1, 1, 0, 0, 0, tzinfo=timezone.utc)
GLONASS_OFFSET = timedelta(hours=3)

# Galileo时间系统从1999年8月22日的UTC时间开始，没有考虑润秒。为了处理Galileo时间的转换，需要定义Galileo时间的起始时间并编写相应的转换函数。
GALILEO_EPOCH = datetime(1999, 8, 22, 0, 0, 0, tzinfo=timezone.utc)


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
def glonass_to_utc_su(glonass_time):
    return GLONASS_EPOCH + timedelta(seconds=glonass_time)
def glonass_to_utc(glonass_time):
    utc_su_time = glonass_to_utc_su(glonass_time)
    return utc_su_time - GLONASS_OFFSET

def glonass_to_unix(glonass_time):
    utc_time = glonass_to_utc(glonass_time)
    return int(utc_time.timestamp())
def glonass_to_gps(glonass_time):
    utc_time = glonass_to_utc(glonass_time)
    return utc_to_gps_leap(utc_time)
# ==================================================================
def galileo_to_utc(galileo_time):
    return GALILEO_EPOCH + timedelta(seconds=galileo_time)

def galileo_to_unix(galileo_time):
    utc_time = galileo_to_utc(galileo_time)
    return int(utc_time.timestamp())
# ==================================================================
# def utc_to_gps(utc_time):
#     return int((utc_time - GPS_EPOCH).total_seconds())
def utc_to_gps_leap(utc_time):
  leap_seconds = count_leap_seconds(utc_time)
  return int((utc_time - GPS_EPOCH).total_seconds()) - leap_seconds
# def utc_to_bds(utc_time):
#     return int((utc_time - BDS_EPOCH).total_seconds())
def utc_to_bds_leap(utc_time):
    leap_seconds = count_leap_seconds(utc_time)
    return int((utc_time - BDS_EPOCH).total_seconds()) - leap_seconds
def utc_su_to_glonass(utc_su_time):
    return int((utc_su_time - GLONASS_EPOCH).total_seconds())
def utc_to_glonass(utc_time):
    utc_su_time = utc_time + GLONASS_OFFSET
    return utc_su_to_glonass(utc_su_time)
def utc_to_galileo(utc_time):
    return int((utc_time - GALILEO_EPOCH).total_seconds())
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
def unix_to_glonass(unix_time):
    utc_time = datetime.fromtimestamp(unix_time, tz=timezone.utc)
    return utc_to_glonass(utc_time)
def unix_to_galileo(unix_time):
    utc_time = datetime.fromtimestamp(unix_time, tz=timezone.utc)
    return utc_to_galileo(utc_time)

# Example times

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
print(f"GLONASS Time: {unix_to_glonass(unix_time)}")
print(f"Galileo Time: {unix_to_galileo(unix_time)}")


print("\nConvert Current UTC Time to other times:")
current_utc_time = datetime.now(timezone.utc)
print(f"Current UTC Time: {current_utc_time.strftime('%Y-%m-%d %H:%M:%S %Z%z')}")
print(f"GPS Time: {utc_to_gps_leap(current_utc_time)}")
print(f"BDS Time: {utc_to_bds_leap(current_utc_time)}")
print(f"GLONASS Time: {utc_to_glonass(current_utc_time)}")
print(f"Galileo Time: {utc_to_galileo(current_utc_time)}")
print(f"UNIX Time: {utc_to_unix(current_utc_time)}")


gps_time = 1000000000  # Example GPS time in seconds
bds_time = 1000000000  # Example BDS time in seconds
glonass_time = 1200000000  # Example GLONASS time in seconds
galileo_time = 800000000  # Example Galileo time in seconds

print("\nConvert GPS and BDS Time to UNIX:")
print(f"GPS Time: {gps_time} -> UNIX Time: {gps_to_unix(gps_time)}")
print(f"BDS Time: {bds_time} -> UNIX Time: {bds_to_unix(bds_time)}")
print(f"GLONASS Time: {glonass_time} -> UNIX Time: {glonass_to_unix(glonass_time)}")
print(f"Galileo Time: {galileo_time} -> UNIX Time: {galileo_to_unix(galileo_time)}")

