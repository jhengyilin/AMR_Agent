need to rewrite to fit ROS commnad (rclpy, PoseStamped, etc.)


# from reflow_oven_def import *
# import time

# if not FAKE_I2C:
#     import smbus


# class MessageHandler:

#     def __init__(self, reflow_oven_control: ReflowOvenControl):
#         self.__reflow_oven_control = reflow_oven_control

#         if not FAKE_I2C:
#             self.__i2c = smbus.SMBus(1)

#     def run(self):
#         while not self.__reflow_oven_control.reset_event.is_set():
#             if FAKE_I2C:
#                 self.__reflow_oven_control.temp.put(40)
#                 self.__reflow_oven_control.time.put(self.__fake_time)
#                 if self.__started:
#                     self.__fake_time += 10
#                     if self.__fake_time > 100:
#                         self.__fake_time = 0
#             else:
#                 try:
#                     self.__reflow_oven_control.temp.put(
#                         self.__i2c.read_word_data(SLAVE_ADDRESS, TEMP_ADDRESS)
#                         / 100)
#                     self.__reflow_oven_control.time.put(
#                         self.__i2c.read_word_data(SLAVE_ADDRESS, TIME_ADDRESS))
#                 except Exception as e:
#                     print(f"Error reading data from I2C: {e}")

#             if self.__reflow_oven_control.start_event.is_set(
#             ) and not self.__started:
#                 self.__started = True

#                 if FAKE_I2C:
#                     print("Message Handler: temp_settings: " +
#                           str(self.__reflow_oven_control.temp_setting.get()))
#                     print("Message Handler: time_setting: " +
#                           str(self.__reflow_oven_control.time_setting.get()))

#                 else:
#                     temp_setting = self.__reflow_oven_control.temp_setting.get(
#                     )
#                     time_setting = self.__reflow_oven_control.time_setting.get(
#                     )

#                     try:
#                         for i in range(5):
#                             self.__i2c.write_byte_data(
#                                 SLAVE_ADDRESS, TEMP_SETTING_ADDRESS + 2 * i,
#                                 (int(temp_setting[i]) // 256) & 0xFF)
#                             self.__i2c.write_byte_data(
#                                 SLAVE_ADDRESS,
#                                 TEMP_SETTING_ADDRESS + 2 * i + 1,
#                                 (int(temp_setting[i]) % 256) & 0xFF)
#                         for i in range(5):
#                             self.__i2c.write_byte_data(
#                                 SLAVE_ADDRESS, TIME_SETTING_ADDRESS + 2 * i,
#                                 (int(time_setting[i]) // 256) & 0xFF)
#                             self.__i2c.write_byte_data(
#                                 SLAVE_ADDRESS,
#                                 TIME_SETTING_ADDRESS + 2 * i + 1,
#                                 (int(time_setting[i]) % 256) & 0xFF)

#                         self.__i2c.write_byte_data(SLAVE_ADDRESS,
#                                                    profile_update,
#                                                    profile_update)
#                         self.__i2c.write_byte_data(SLAVE_ADDRESS,
#                                                    START_COMMAND,
#                                                    START_COMMAND)
#                     except Exception as e:
#                         print(f"Error writing data to I2C: {e}")

#             time.sleep(1)

#         if FAKE_I2C:
#             print("Message Handler: Resetting...")
#         else:
#             self.__i2c.write_byte_data(SLAVE_ADDRESS, RESET_COMMAND,
#                                        RESET_COMMAND)

#     __reflow_oven_control = None

#     __i2c = None
#     __started = False
#     __fake_time = 0
